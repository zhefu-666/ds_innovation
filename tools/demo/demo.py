#!/usr/bin/env python3
"""Independent robot demonstrations. Default is file/command inspection, with no device access."""
import argparse
from contextlib import contextmanager, redirect_stdout, redirect_stderr
import fcntl
import json
import math
import os
import re
from pathlib import Path
import shlex
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import time
import threading

ROOT = Path(__file__).resolve().parents[2]
ACTIONS = ('recognize', 'search', 'carry', 'open', 'close', 'camera', 'frame-status',
           'move-forward', 'move-backward', 'turn-left', 'turn-right',
           'commission-open', 'commission-close', 'stop', 'status', 'web', 'replay', 'preflight')
WHEELS = {'move-forward', 'move-backward', 'turn-left', 'turn-right'}
DECISIONS = {'search', 'carry'}
LEGACY_SOCKET = Path('/tmp/rescue-match.sock')
DEFAULT_RUNTIME = Path('/tmp') / ('rescue-demo-' + str(os.getuid()))


def json_file(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError('Expected a JSON object: ' + str(path))
    return value


def rooted(base, value):
    p = Path(value)
    return p if p.is_absolute() else (base / p).resolve()


def load_profile(path):
    profile = json_file(path)
    if profile.get('schema_version') != 1:
        raise ValueError('Unsupported demo profile version')
    for key in ('binary', 'frame_config', 'acceptance'):
        profile[key] = str(rooted(ROOT, profile[key]))
    resources = rooted(ROOT, profile['resource_root'])
    for key in ('model', 'pose_model', 'rknn_library', 'camera_calibration', 'zone_geometry'):
        profile[key] = str(rooted(resources, profile[key]))
    if profile['team'] not in ('blue', 'red'):
        raise ValueError('team must be blue or red')
    return profile


def mapping_issues(profile):
    data = json_file(profile['frame_config'])
    issues = []
    if data.get('action_mapping_verified') != 1:
        issues.append('frame_config.action_mapping_verified：尚未验收0开20关与A6映射')
    if data.get('a6_semantics') == 'done_flag':
        settle = data.get('a6_done_settle_ms')
        if type(settle) is not int or not 100 <= settle <= 3000:
            issues.append('frame_config.a6_done_settle_ms：done_flag须填100..3000整数毫秒')
        return issues
    values = [data.get('a6_open_state'), data.get('a6_close_state')]
    if any(type(v) is not int for v in values) or set(values) != {0, 1}:
        issues.append('frame_config.a6_open_state/a6_close_state：须填实测的不同0/1值')
    return issues


def done_flag_config(profile):
    try:
        return json_file(profile['frame_config']).get('a6_semantics') == 'done_flag'
    except (OSError, ValueError):
        return False


KNOWN_FILE = DEFAULT_RUNTIME/'known_frame.json'


def known_frame(args):
    """TEMP_ASSUMPTION done_flag: explicit CLI wins; else the angle/id recorded by this launcher's last open/close."""
    if args.known_frame_angle is not None or args.known_frame_id is not None:
        if args.known_frame_angle not in (0, 20) or type(args.known_frame_id) is not int or not 1 <= args.known_frame_id <= 255:
            raise ValueError('--known-frame-angle 0|20 与 --known-frame-id 1..255 须同时给出')
        return args.known_frame_angle, args.known_frame_id, 'cli'
    try:
        data = json_file(KNOWN_FILE)
    except (OSError, ValueError):
        return None
    if data.get('angle') in (0, 20) and type(data.get('id')) is int and 1 <= data['id'] <= 255:
        return data['angle'], data['id'], 'recorded ' + str(data.get('recorded_at'))
    return None


def acceptance_issues(profile, action):
    data = json_file(profile['acceptance'])
    issues = []
    keys = ['stationary_mapping_trials_passed']
    if action in WHEELS | DECISIONS:
        keys.append('watchdog_verified')
        delay = data.get('watchdog_max_ms')
        if type(delay) not in (int, float) or not math.isfinite(delay) or not 0 < delay <= 200:
            issues.append('watchdog_max_ms：须有<=200ms的实测看门狗记录')
    if action in DECISIONS:
        keys += ['manual_wheel_test_passed', 'ground_geometry_verified']
    if action == 'carry':
        keys.append('multi_view_dataset_passed')
    for key in keys:
        if data.get(key) is not True:
            issues.append('acceptance.' + key + '：缺少实机验收记录')
    if not data.get('operator') or not data.get('verified_date') or not data.get('evidence_record'):
        issues.append('acceptance：须记录操作人、日期和证据路径')
    return issues


def make_plan(args, p):
    action = args.action
    runtime = DEFAULT_RUNTIME
    seconds = args.seconds if args.seconds is not None else (.5 if action in WHEELS else 3600 if action == 'web' else 60)
    if not math.isfinite(seconds) or seconds <= 0 or seconds > (1 if action in WHEELS else 3600 if action == 'web' else 60):
        raise ValueError('底盘点动时长须在(0,1]秒；其他演示须在(0,60]秒')
    if not math.isfinite(args.speed) or not 0 < args.speed <= .10:
        raise ValueError('--speed must be in (0,0.10] m/s')
    if not math.isfinite(args.wz) or not 0 < args.wz <= .25:
        raise ValueError('--wz must be in (0,0.25] rad/s')
    if args.preview and action != 'search':
        raise ValueError('--preview仅用于寻找决策只读预览；搬运回放请用replay.sh')
    if action in DECISIONS | {'recognize'} and seconds != int(seconds):
        raise ValueError('识别/寻找/搬运时长使用整数秒')
    required = []
    command = None
    native_check = None
    issues = []
    py = p.get('python', 'python3')
    if action in {'recognize', 'search', 'carry'}:
        required = ['binary', 'model', 'rknn_library']
        command = [p['binary'], '--demo-mode', {'carry': 'carry_once'}.get(action, action),
                   '--demo-seconds', str(int(seconds)), '--no-show', '--model', p['model'],
                   '--rknn-library', p['rknn_library'], '--camera', str(p['camera']),
                   '--width', str(p['width']), '--height', str(p['height']), '--fps', str(p['fps']),
                   '--startup-advance-ms', '0', '--startup-speed', '.10', '--scan-wz', '.25', '--turn-wz', '.25',
                   '--match-seconds', str(int(seconds)), '--telemetry',
                   '--telemetry-file', str(runtime/'snapshot.bin').replace('/tmp/', '/dev/shm/'),
                   '--match-socket', str(runtime/'match.sock')]
        # tmpfs snapshot must have an existing parent; keep it directly in /dev/shm.
        command[command.index('--telemetry-file')+1] = '/dev/shm/rescue-demo-%d.bin' % os.getuid()
        if action == 'recognize':
            command += ['--dry-run']
        else:
            required += ['pose_model', 'camera_calibration', 'zone_geometry']
            command += ['--imu', '--imu-port', p['imu_port'], '--imu-baud', str(p['imu_baud']),
                        '--port', p['port'], '--baud', str(p['baud']), '--team', p['team'],
                        '--calibration', p['camera_calibration'], '--zone-geometry', p['zone_geometry'],
                        '--pose-model', p['pose_model']]
            if args.preview:
                command += ['--dry-run', '--pitch-feedback']
            else:
                command += ['--task-calibration', p['frame_config'], '--hardware', '--auto-run', '--controlled-empty-field']
                if action == 'carry' and getattr(args, 'assume_all_safe', False):
                    # TEMP_ASSUMPTION（2026-10-08用户要求“默认全都安全”）：仅用于跑通决策链，非验收。
                    command += ['--assume-all-safe']
                    if getattr(args, 'assume_injured_trip', False):
                        command += ['--assume-injured-trip']
                    # 目标贴近时按 5°→20°→40° 下压相机继续跟踪；几何为机械假设模型（未实测验证），同属临时假设。
                    mech = Path(p['camera_calibration']).with_name('camera.mechanical_5_40.yaml')
                    if not mech.is_file():
                        issues.append('assume模式缺少机械俯仰模型文件：' + str(mech))
                    command[command.index('--calibration') + 1] = str(mech)
                    command += ['--allow-mechanical-pitch-model', '--pitch-presets', '500,4000,4000']
                native_check = command + ['--check-config']
                required += ['frame_config', 'acceptance']
    elif action in {'open', 'close', 'frame-status'}:
        command = [py, str(ROOT/'tools/frame/frame_ctl.py'), 'status' if action == 'frame-status' else action,
                   '--port', p['port'], '--baud', str(p['baud']), '--timeout', '2']
        if action != 'frame-status':
            required = ['frame_config', 'acceptance']
            command += ['--frame-config', p['frame_config']]
    elif action == 'camera':
        if not -40 <= args.offset_deg <= 40:
            raise ValueError('相机内部/TX偏移角须为-40..40°')
        required = ['frame_config', 'acceptance']
        command = [py, str(ROOT/'tools/camera_pitch/pitch_ctl.py'), 'set', str(args.offset_deg+40),
                   '--port', p['port'], '--baud', str(p['baud']), '--frame-config', p['frame_config'], '--timeout', '2']
    elif action in WHEELS:
        required = ['frame_config', 'acceptance']
    elif action == 'web':
        command = [py, str(ROOT/'tools/remote_camera_telemetry/project_bridge.py'),
                   '--snapshot', '/dev/shm/rescue-demo-%d.bin' % os.getuid(), '--host', '127.0.0.1',
                   '--http-port', str(p['http_port']), '--ws-port', str(p['ws_port'])]
    elif action == 'replay':
        required = ['binary']
        command = [p['binary'], '--push-replay', str(ROOT/'tests/fixtures/push_delivery.json')]
    known = None
    if command and not args.preview and action in DECISIONS | {'camera'} and done_flag_config(p):
        known = known_frame(args)
        if known is None:
            issues.append('done_flag：未知方框当前角度；先用本脚本 open/close，或给 --known-frame-angle/--known-frame-id')
        elif action in DECISIONS:
            command += ['--known-frame-angle', str(known[0]), '--known-frame-id', str(known[1])]
            native_check = command + ['--check-config']
        else:
            command += ['--known-frame-angle', str(known[0]), '--known-frame-id', str(known[1])]
    if action in WHEELS and done_flag_config(p):
        known = known_frame(args)
        if known is None:
            issues.append('done_flag：未知方框当前角度；先用本脚本 open/close，或给 --known-frame-angle/--known-frame-id')
    for key in required:
        if not Path(p[key]).is_file():
            issues.append('缺少文件 %s: %s' % (key, p[key]))
    mapped_action = action in WHEELS | DECISIONS | {'open', 'close', 'camera'} and not args.preview
    if mapped_action and Path(p['frame_config']).is_file():
        issues += mapping_issues(p)
        if action in DECISIONS:
            frame_data = json_file(p['frame_config'])
            if frame_data.get('box_area_accepted') != 1:
                issues.append('当前检测模型为box模式；尚未明确接受检测框占比')
    if mapped_action and Path(p['acceptance']).is_file():
        issues += acceptance_issues(p, action)
    return {'action': action, 'seconds': seconds, 'command': command, 'native_check': native_check,
            'issues': issues, 'runtime': runtime, 'known': known}


def start_ticks(pid):
    try:
        text = Path('/proc/%d/stat' % pid).read_text()
        return text[text.rfind(')')+2:].split()[19]
    except (OSError, IndexError):
        return None


def private_runtime():
    path = DEFAULT_RUNTIME
    path.mkdir(mode=0o700, exist_ok=True)
    info = path.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise ValueError('演示运行目录必须是当前用户私有目录：' + str(path))
    return path


@contextmanager
def demo_lock(action):
    runtime = private_runtime()
    if LEGACY_SOCKET.exists():
        if not stat.S_ISSOCK(LEGACY_SOCKET.lstat().st_mode):
            raise ValueError('Unexpected legacy match socket path; inspect existing host first')
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as probe:
            try:
                probe.connect(str(LEGACY_SOCKET))
            except ConnectionRefusedError:
                pass  # Leave another program's stale path untouched.
            else:
                raise ValueError('旧主程序仍运行；请先正常停止旧入口，演示不会抢占设备')
    # Share the legacy launch lock; never kill or silently replace an existing host.
    fd = os.open('/tmp/rescue-match-launch.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    state_path = runtime/'session.json'
    try:
        if os.fstat(fd).st_uid != os.getuid():
            raise ValueError('Launch lock belongs to another user')
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        sock = runtime/'match.sock'
        if sock.exists():
            st = sock.lstat()
            if not stat.S_ISSOCK(st.st_mode) or st.st_uid != os.getuid():
                raise ValueError('Unexpected object at demo match socket')
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
                try:
                    client.connect(str(sock))
                except ConnectionRefusedError:
                    sock.unlink()
                else:
                    raise ValueError('Demo match socket is still active')
        state = {'pid': os.getpid(), 'start_ticks': start_ticks(os.getpid()), 'action': action,
                 'match_socket': str(sock), 'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
        tmp = runtime/'session.tmp'
        with open(tmp, 'w') as f:
            json.dump(state, f)
        os.replace(tmp, state_path)
        yield
    finally:
        if state_path.is_file():
            try:
                if json_file(state_path).get('pid') == os.getpid():
                    state_path.unlink()
            except (OSError, ValueError):
                pass
        os.close(fd)


def control(stop=False):
    runtime = private_runtime()
    file = runtime/'session.json'
    if not file.exists():
        print('没有由本套脚本启动的演示；这不代表其他程序或实物已经停止。')
        return 0
    s = json_file(file)
    pid = s.get('pid')
    if type(pid) is not int or pid <= 1 or start_ticks(pid) != s.get('start_ticks'):
        print('演示记录已失效；未向任何进程发送信号。')
        return 2
    print(json.dumps(s, ensure_ascii=False, indent=2))
    if not stop:
        return 0
    if Path(s['match_socket']).exists():
        try:
            with tempfile.TemporaryDirectory(dir=runtime) as folder:
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
                    client.bind(str(Path(folder)/'reply.sock'));client.settimeout(.5)
                    client.sendto(b'stop', s['match_socket'])
                    print('控制器响应：' + client.recv(4096).decode())
        except OSError as exc:
            print('控制器STOP未确认：' + str(exc))
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic()+5
    while time.monotonic() < deadline and start_ticks(pid) == s['start_ticks']:
        time.sleep(.05)
    if start_ticks(pid) == s['start_ticks']:
        print('停止请求已发送，但进程未退出；现场急停并核对机构。')
        return 2
    print('演示进程已退出；仍需观察实物停止，软件回执不是实物停车证据。')
    return 0


class Tee:
    def __init__(self, console, file):self.console=console;self.file=file
    def write(self, text):self.console.write(text);self.file.write(text);self.file.flush();return len(text)
    def flush(self):self.console.flush();self.file.flush()


class Interrupted(Exception):
    pass


def interrupt(signum, _frame):
    raise Interrupted('收到停止信号 ' + str(signum))


def child_run(command, seconds, lines=None):
    process = subprocess.Popen(command, cwd=ROOT, start_new_session=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    def forward():
        for line in process.stdout:
            print(line, end='', flush=True)
            if lines is not None:
                lines.append(line)
    reader=threading.Thread(target=forward,daemon=True);reader.start()
    try:
        return process.wait(timeout=seconds)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL);process.wait()
                print('子进程未正常退出，已强制终止；零速发送未确认，须现场急停。')
        reader.join(timeout=1)
        process.stdout.close()


def run(args, p, plan):
    action = args.action
    if action in WHEELS | DECISIONS | {'open', 'close', 'camera', 'commission-open', 'commission-close'} and not args.preview:
        if not args.site_clear:
            raise ValueError('执行动作须显式带 --site-clear，确认本轮人员/机构/路径已清空')
    if action.startswith('commission-') and not args.wheels_disabled:
        raise ValueError('首次映射取证须带 --wheels-disabled，并在实物上禁用车轮运动输出')
    if plan['issues']:
        raise ValueError('缺少前提，拒绝打开设备：\n- ' + '\n- '.join(plan['issues']))
    if plan['native_check']:
        result = subprocess.run(plan['native_check'], cwd=ROOT, timeout=10)
        if result.returncode:
            return result.returncode
    if action == 'web':
        return child_run(plan['command'], plan['seconds'])
    with demo_lock(action):
        if action in WHEELS or action.startswith('commission-'):
            import demo_serial
            if action.startswith('commission-'):
                demo_serial.commission(p['port'], p['baud'], 0 if action.endswith('open') else 20)
            else:
                from frame_semantics import load_mapping
                vx = args.speed if action == 'move-forward' else -args.speed if action == 'move-backward' else 0
                wz = args.wz if action == 'turn-left' else -args.wz if action == 'turn-right' else 0
                known = plan['known'][:2] if plan['known'] else None
                demo_serial.manual_motion(p['port'], p['baud'], load_mapping(p['frame_config']), vx, wz, plan['seconds'], known=known)
            return 0
        if action in ('open', 'close') or action in DECISIONS:
            KNOWN_FILE.unlink(missing_ok=True)  # 动作期间/决策结束后方框角度未知，须重新open/close
        lines = []
        result = child_run(plan['command'], plan['seconds']+3 if action in DECISIONS | {'recognize'} else plan['seconds'], lines)
        if action in ('open', 'close') and result == 0:
            for line in lines:
                if line.startswith('KNOWN_FRAME '):
                    # frame_ctl在编号后紧接中文说明，按数字前缀解析
                    fields = dict(re.findall(r'\b(angle|id)=(-?\d+)', line))
                    record = {'angle': int(fields['angle']), 'id': int(fields['id']), 'recorded_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                              'note': 'TEMP_ASSUMPTION done_flag; valid only while A6 id is unchanged'}
                    KNOWN_FILE.write_text(json.dumps(record) + '\n')
                    print('已记录方框已知状态：' + json.dumps(record, ensure_ascii=False))
        return result


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=ACTIONS)
    ap.add_argument('--profile', default=os.environ.get('RESCUE_DEMO_PROFILE',str(ROOT/'config/demo_20261007.json')))
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--run', action='store_true', help='执行本脚本的动作；默认仅检查')
    mode.add_argument('--preview', action='store_true', help='寻找只读预览：相机+IMU+MCU RX，无TX')
    mode.add_argument('--check', action='store_true', help='仅检查文件和前提，不打开设备（默认）')
    ap.add_argument('--site-clear', action='store_true', help='确认机构/运动扫掠空间已清空；寻找搬运额外确认场内仅己方区域且无人/对手进入')
    ap.add_argument('--wheels-disabled', action='store_true', help='静止映射取证：现场已经禁用车轮输出')
    ap.add_argument('--seconds', type=float)
    ap.add_argument('--speed', type=float, default=.05)
    ap.add_argument('--wz', type=float, default=.20)
    ap.add_argument('--known-frame-angle', type=int, choices=[0, 20], help='done_flag临时假设：方框当前已知角度（默认取本脚本上次open/close记录）')
    ap.add_argument('--known-frame-id', type=int, help='done_flag临时假设：该角度对应的A6动作编号')
    ap.add_argument('--assume-all-safe', action='store_true', help='TEMP_ASSUMPTION：搬运时所有安全/净空/区域/确认证据视为成立，仅跑通决策链，非验收')
    ap.add_argument('--assume-injured-trip', action='store_true', help='TEMP_ASSUMPTION：配合--assume-all-safe，视为已送达首个普通物资并优先抓伤员块（演示右半区），非验收')
    ap.add_argument('--offset-deg', type=int, default=5, help='相机内部/TX角度，5对应A6 RX45；范围-40..40')
    return ap


def main(argv=None):
    args = parser().parse_args(argv)
    signal.signal(signal.SIGTERM, interrupt);signal.signal(signal.SIGINT, interrupt)
    try:
        if args.action in ('stop', 'status'):
            return control(args.action == 'stop')
        p = load_profile(args.profile)
        if args.action == 'preflight':
            any_blocked = False
            for action in ('recognize', 'frame-status', 'open', 'camera', 'move-forward', 'search', 'carry'):
                checked = argparse.Namespace(**vars(args));checked.action=action;checked.preview=False
                plan = make_plan(checked, p)
                print('\n[%s] %s' % (action, 'BLOCKED' if plan['issues'] else 'FILES_READY'))
                for issue in plan['issues']:print('- ' + issue)
                any_blocked |= bool(plan['issues'])
            return 2 if any_blocked else 0
        plan = make_plan(args, p)
        print('动作：%s；预算：%ss；模式：%s' % (args.action, plan['seconds'], 'PREVIEW_RX_ONLY' if args.preview else 'EXECUTE' if args.run else 'CHECK_NO_DEVICES'))
        if plan['command']:print('命令：' + shlex.join(plan['command']))
        if '--assume-all-safe' in (plan['command'] or []):print('[TEMP_ASSUMPTION] assume_all_safe=1：净空/区域/抓取/投放证据默认成立，结果不构成验收。')
        if args.action in WHEELS:print('底盘点动：%s，速度<=0.10m/s、角速度<=0.25rad/s；结束补零速且保持机构。' % args.action)
        if args.action.startswith('commission-'):print('车速固定0；仅取证原始角度/动作编号/A6状态，不判断机构已到位，不自动写验收结果。')
        if not args.run and not args.preview:
            for issue in plan['issues']:print('BLOCKED: ' + issue)
            if plan['native_check'] and not plan['issues']:
                return subprocess.run(plan['native_check'], cwd=ROOT, timeout=10).returncode
            return 2 if plan['issues'] else 0
        # Log headers and commands per invocation. Child stdout remains visible on the terminal.
        logs = ROOT/'telemetry_logs/demo';logs.mkdir(parents=True, exist_ok=True)
        path = logs/('%s-%s-%d.json' % (time.strftime('%Y%m%d-%H%M%S'), args.action, os.getpid()))
        path.write_text(json.dumps({'action': args.action, 'command': plan['command'], 'profile': args.profile,
                                    'seconds': plan['seconds'], 'site_clear': args.site_clear}, ensure_ascii=False, indent=2)+'\n')
        print('调用记录：' + str(path), flush=True)
        log_path=path.with_suffix('.log')
        print('完整输出日志：' + str(log_path), flush=True)
        with log_path.open('w') as log, redirect_stdout(Tee(sys.stdout,log)), redirect_stderr(Tee(sys.stderr,log)):
            try:
                result=run(args,p,plan)
            except Exception as exc:
                print('DEMO_EXCEPTION: '+str(exc),flush=True)
                raise
            print('DEMO_EXIT_CODE: '+str(result),flush=True)
            return result
    except Interrupted as exc:
        print(str(exc));return 130
    except subprocess.TimeoutExpired:
        print('演示时限到，已请求子进程退出；请核对实物停止。');return 2
    except (OSError, ValueError, KeyError, TypeError, TimeoutError) as exc:
        print('拒绝或中止：' + str(exc), file=sys.stderr);return 2


if __name__ == '__main__':
    raise SystemExit(main())
