#!/usr/bin/env python3
"""终端手动控制相机pitch，同时保持夹爪张开（不启动主程序）。

用法：
  pitch_ctl.py status                 查看当前动作编号/done/pitch读回
  pitch_ctl.py set <度>               夹爪张开并转到目标角，读回到位后退出
  pitch_ctl.py repl                   交互模式，终端输入角度实时调整
  pitch_ctl.py watch                  只读：持续显示下位机上传的pitch，不发任何包
公共参数：[--port /dev/ttyACM0] [--baud 115200] [--timeout 3] [--tol 3] [--dry-run]
         [--watch 秒]（set 到位/超时后继续发包并显示读回的时长）

set/repl 下发后会在终端实时显示A6上传的pitch读回（变化时立即打印，不变时每0.5s一行），
含原始cdeg值、目标的小端字节和整帧A6字节（pitch字节3–4用[]标出），便于核对下位机是否
刷新该字段；结束时汇总读回出现过的值。
A6帧：A6 | gripper_open | 编号 | pitch低 pitch高 | CRC低 CRC高 | 0A
运动包：56 | vx(4) | wz(4) | gripper_open | 编号 | pitch低 pitch高 | CRC低 CRC高

角度单位为度：0平视，正值向下，负值向上；暂时限位±40°，超出会截断。
收发线上pitch均为整数度；工具内部换算为cdeg，支持±40°。
启动时先读A6反馈拿到当前动作编号，用“当前编号+1”发一次张开并等状态与目标匹配，
之后所有包沿用这个编号和gripper_open=1，调pitch不会再触发夹爪动作。
速度固定为0。退出后下位机200ms超时停车，相机保持当前角度。
"""
import argparse, os, struct, sys, termios, threading, time, tty

INVALID_PITCH = -32768
PITCH_LIMIT = 4000  # 内部cdeg；线上整数度，机械限位±40°



def decode_pitch(deg):
    return deg * 100 if -40 <= deg <= 40 else INVALID_PITCH


def crc16(data):
    c = 0xFFFF
    for x in data:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def motion_packet(gripper_open, action_id, pitch):
    b = struct.pack('<BffBBh', 0x56, 0.0, 0.0, gripper_open, action_id, max(-40, min(40, int(round(pitch / 100)))))
    return b + struct.pack('<H', crc16(b))


class Feedback:
    """按帧头+8字节+CRC+帧尾0x0A解析A6，不按换行切分。帧为 (gripper_open, 编号, pitch, 原始8字节)。"""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []
        while len(self.buf) >= 8:
            if self.buf[0] != 0xA6:
                del self.buf[0]
                continue
            f = bytes(self.buf[:8])
            if crc16(f[:5]) == struct.unpack('<H', f[5:7])[0] and f[7] == 0x0A:
                frames.append((f[1], f[2], decode_pitch(struct.unpack('<h', f[3:5])[0]), f))
                del self.buf[:8]
            else:
                del self.buf[0]
        return frames


def open_port(port, baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    a = termios.tcgetattr(fd)
    a[4] = a[5] = getattr(termios, 'B%d' % baud)
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def read_frames(fd, parser):
    try:
        return parser.feed(os.read(fd, 512))
    except BlockingIOError:
        return []


def wait_feedback(fd, parser, timeout):
    end = time.time() + timeout
    while time.time() < end:
        frames = read_frames(fd, parser)
        if frames:
            return frames[-1]
        time.sleep(0.01)
    return None


def pitch_txt(p):
    return '无效' if p == INVALID_PITCH else '%.2f°' % (p / 100)


def frame_hex(raw):
    """整帧A6的8个字节，pitch所在的字节3–4用[]标出，如 A6 01 08 [19 00] 41 D3 0A。"""
    h = raw.hex(' ').upper().split()
    return ' '.join(h[:3] + ['[' + h[3], h[4] + ']'] + h[5:])


def cdeg_hex(p):
    """int16小端两个字节，如 1200 -> B0 04。"""
    return struct.pack('<h', INVALID_PITCH if p == INVALID_PITCH else int(round(p / 100))).hex(' ').upper()


def pitch_detail(p, raw):
    """读回的度数、原始cdeg和整帧字节，如 0.25° (25 cdeg)  RX: A6 01 08 [19 00] 41 D3 0A。"""
    return '%s (%d cdeg)  RX: %s' % (pitch_txt(p), p, frame_hex(raw))


class Tracer:
    """把下位机上传的pitch读回实时打印到终端：变化时立即打印，不变时每 every 秒一行。"""
    def __init__(self, target, every=0.5):
        self.target, self.every = target, every
        self.t0 = time.time()
        self.last_pitch = None
        self.last_print = 0.0
        self.frames = 0
        self.seen = []  # 按出现顺序记录读回过的值
        self.changed_at = self.t0

    def frame(self, f):
        done, action_id, p, raw = f
        now = time.time()
        self.frames += 1
        if p not in self.seen:
            self.seen.append(p)
        changed = p != self.last_pitch
        if changed:
            self.changed_at = now
        if changed or now - self.last_print >= self.every:
            err = '' if p == INVALID_PITCH or self.target is None else '  差 %+.2f°' % ((p - self.target) / 100)
            goal = '' if self.target is None else '目标 %.2f° [%s]  ' % (self.target / 100, cdeg_hex(self.target))
            print('  +%5.2fs  %s读回 %s (%d cdeg)%s  编号=%d gripper_open=%d  RX: %s%s' % (
                  now - self.t0, goal, pitch_txt(p), p, err, action_id, done, frame_hex(raw),
                  '  *变化' if changed and self.last_pitch is not None else ''), flush=True)
            self.last_print = now
        self.last_pitch = p

    def stable_for(self):
        return time.time() - self.changed_at

    def summary(self):
        elapsed = max(time.time() - self.t0, 1e-6)
        if not self.frames:
            print('  期间未收到A6反馈（下位机应≥20Hz上报）')
            return
        values = ', '.join('%d[%s]' % (v, cdeg_hex(v)) for v in self.seen[:12]) + (' ...' if len(self.seen) > 12 else '')
        print('  反馈 %d 帧，%.1f Hz；读回出现过 %d 个值(cdeg): %s' % (self.frames, self.frames / elapsed,
              len(self.seen), values))
        if (self.target is not None and len(self.seen) == 1 and self.seen[0] != INVALID_PITCH
                and abs(self.seen[0] - self.target) > 50):
            print('  注意: 读回始终为 %d cdeg（字节 %s），下发目标为 %d cdeg（字节 %s），读回未随目标变化；'
                  '若相机实际已转动，说明下位机没有刷新该字段' % (self.seen[0], cdeg_hex(self.seen[0]),
                  self.target, cdeg_hex(self.target)))


def to_cdeg(deg):
    if not __import__('math').isfinite(deg):
        raise ValueError('角度必须为有限数值')
    actual = max(-40, min(40, int(round(deg))))
    if actual != deg:
        print('提示: 按整数度与±40°限位，实际目标为 %d°' % actual)
    return actual * 100


class Link:
    """保存最近反馈，负责发包和等待条件。"""
    def __init__(self, fd, dry_run):
        self.fd, self.dry_run = fd, dry_run
        self.parser = Feedback()
        self.last = None  # (done, id, pitch, raw)
        self.on_frame = None  # 可选回调，每帧反馈调用一次（用于实时显示读回）

    def poll(self):
        frames = read_frames(self.fd, self.parser)
        if frames:
            self.last = frames[-1]
            if self.on_frame:
                for f in frames:
                    self.on_frame(f)
        return frames

    def send(self, pkt):
        if not self.dry_run:
            os.write(self.fd, pkt)

    def send_until(self, pkt, ok, timeout):
        """20Hz重复发同一包，直到某帧反馈满足ok(frame)或超时。"""
        end = time.time() + timeout
        while time.time() < end:
            self.send(pkt)
            for f in self.poll():
                if ok(f):
                    return True
            time.sleep(0.05)
        return False


def open_gripper(link, cur_id, pitch, timeout):
    new_id = 1 if cur_id >= 255 else cur_id + 1
    pkt = motion_packet(1, new_id, pitch)
    print('TX (张开, 编号%d): %s' % (new_id, pkt.hex(' ').upper()))
    if link.dry_run:
        return new_id
    if link.send_until(pkt, lambda f: f[1] == new_id and f[0] == 1, timeout):
        print('夹爪已张开: 编号%d' % new_id)
    else:
        print('警告: %.1fs内未确认夹爪张开完成，继续调pitch' % timeout)
    return new_id


def move_pitch(link, action_id, target, timeout, tol, watch=0.0):
    pkt = motion_packet(1, action_id, target)
    print('TX (pitch %.2f°, 编号%d): %s' % (target / 100, action_id, pkt.hex(' ').upper()))
    if link.dry_run:
        return True
    print('下位机上传的pitch读回：')
    tracer = Tracer(target)
    link.on_frame = tracer.frame
    try:
        ok = link.send_until(pkt, lambda f: f[2] != INVALID_PITCH and abs(f[2] - target) <= tol, timeout)
        if ok:  # 进容差后再发0.5s，让舵机走完剩余行程
            link.send_until(pkt, lambda f: False, 0.5)
        got = pitch_detail(link.last[2], link.last[3]) if link.last else '无反馈'
        print(('到位: 读回 %s' if ok else '超时%.1fs未到位: 读回 %%s' % timeout) % got)
        if watch > 0:
            print('继续保持目标并显示读回 %.1fs（Ctrl+C 结束）' % watch)
            try:
                link.send_until(pkt, lambda f: False, watch)
            except KeyboardInterrupt:
                print()
    finally:
        link.on_frame = None
        tracer.summary()
    return ok


REPL_HELP = """输入角度(度)直接转，如 10 / -5 / 0；正值向下，暂时限位±25，固件档位 -25/0/25
  输入角度后实时显示读回，到位且稳定0.3s或超时后回到提示符（Ctrl+C 提前返回）
  s  查看读回    m  持续显示读回直到 Ctrl+C    x  打印当前包    h  帮助    q  退出"""


def trace(link, target, timeout, tol):
    """前台显示读回（发包与收包仍在后台线程）。timeout为None时一直显示到Ctrl+C。"""
    tracer = Tracer(target)
    link.on_frame = tracer.frame
    end = None if timeout is None else time.time() + timeout
    reached = False
    try:
        while end is None or time.time() < end:
            p = tracer.last_pitch
            if (timeout is not None and p is not None and p != INVALID_PITCH and abs(p - target) <= tol
                    and tracer.stable_for() >= 0.3):
                reached = True
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        link.on_frame = None
    if timeout is not None:
        print('到位' if reached else '%.1fs内未到位（容差±%.2f°）' % (timeout, tol / 100))
    tracer.summary()


def repl(link, action_id, pitch, timeout, tol):
    """后台20Hz持续发当前目标，前台读输入改目标。"""
    state = {'pitch': pitch}
    lock = threading.Lock()
    stop = threading.Event()

    def tx():
        while not stop.is_set():
            with lock:
                pkt = motion_packet(1, action_id, state['pitch'])
            link.send(pkt)
            link.poll()
            time.sleep(0.05)
    if link.dry_run:
        timeout = 0  # 不写串口时没有到位过程可显示

    t = threading.Thread(target=tx, daemon=True)
    t.start()
    print(REPL_HELP)
    try:
        while True:
            line = input('pitch> ').strip().lower()
            if not line:
                continue
            if line == 'q':
                break
            if line == 'h':
                print(REPL_HELP)
            elif line == 's':
                if link.last:
                    d, i, p, raw = link.last
                    print('目标 %.2f°  读回 %s  编号=%d gripper_open=%d' % (state['pitch'] / 100, pitch_detail(p, raw), i, d))
                else:
                    print('目标 %.2f°  尚无反馈' % (state['pitch'] / 100))
            elif line == 'm':
                print('持续显示读回，Ctrl+C 返回提示符')
                trace(link, state['pitch'], None, tol)
            elif line == 'x':
                with lock:
                    print(motion_packet(1, action_id, state['pitch']).hex(' ').upper())
            else:
                try:
                    target = to_cdeg(float(line))
                except ValueError:
                    print('未知输入，h 查看帮助')
                    continue
                with lock:
                    state['pitch'] = target
                print('目标 -> %.2f°（夹爪保持张开，编号%d）' % (target / 100, action_id))
                if timeout > 0:
                    trace(link, target, timeout, tol)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        stop.set()
        t.join()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', choices=['status', 'set', 'repl', 'watch'])
    ap.add_argument('deg', nargs='?', type=float, help='set 的目标角（度，正值向下）')
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=3.0, help='等待夹爪完成/pitch到位的秒数')
    ap.add_argument('--tol', type=float, default=3.0, help='pitch到位容差（度），读回精度约1~3°')
    ap.add_argument('--dry-run', action='store_true', help='只读反馈并打印将发送的包，不写串口')
    ap.add_argument('--watch', type=float, default=0.0, metavar='秒',
                    help='set 到位/超时后继续保持目标并显示读回的秒数')
    args = ap.parse_args()
    if args.action == 'set' and args.deg is None:
        ap.error('set 需要目标角，例如: pitch_ctl.py set 10')

    try:
        fd = open_port(args.port, args.baud)
    except OSError as e:
        sys.exit('打开 %s 失败：%s（网页串口助手或主程序是否占用？）' % (args.port, e))
    link = Link(fd, args.dry_run)
    try:
        fb = wait_feedback(fd, link.parser, 0.5)
        if fb is None:
            sys.exit('0.5s内未收到A6反馈，检查下位机与串口')
        link.last = fb
        done, cur_id, cur_pitch, raw = fb
        print('当前: 动作编号=%d gripper_open=%d pitch=%s' % (cur_id, done, pitch_detail(cur_pitch, raw)))
        if args.action == 'status':
            return
        if args.action == 'watch':
            # 只读：不发任何包，夹爪和相机都不会动
            print('只读显示下位机上传的pitch，Ctrl+C 结束')
            tracer = Tracer(None)
            link.on_frame = tracer.frame
            tracer.frame(fb)
            try:
                while True:
                    link.poll()
                    time.sleep(0.01)
            except KeyboardInterrupt:
                print()
            tracer.summary()
            return

        # 张开时先保持当前角，避免夹爪和相机同时动
        keep = 0 if cur_pitch == INVALID_PITCH else max(-PITCH_LIMIT, min(PITCH_LIMIT, cur_pitch))
        action_id = open_gripper(link, cur_id, keep, args.timeout)
        if args.action == 'set':
            ok = move_pitch(link, action_id, to_cdeg(args.deg), args.timeout, int(round(args.tol * 100)), args.watch)
            if not ok:
                sys.exit(1)
        else:
            repl(link, action_id, keep, args.timeout, int(round(args.tol * 100)))
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
