#!/usr/bin/env python3
"""Send a velocity command using the current MCU motion protocol.

The default protocol is the current 15-byte packet used by the LubanCat
workspace. Pass --protocol legacy6 only for the older six-byte packet in the
root workspace's historical VELOCITY_PROTOCOL.md.

Preview mode is the default. The serial port is opened only with --send.
"""

import argparse
import math
import os
import select
import struct
import termios
import threading
import time
from typing import Optional


HEADER = 0x56
MAX_LINEAR_SPEED_MPS = 0.2
MAX_PITCH_DEG = 40
BAUDRATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


class FeedbackParser:
    """Parse the current eight-byte A6 actuator feedback frame."""

    def __init__(self, show_rx: bool = False) -> None:
        self.show_rx = show_rx
        self.started = time.monotonic()
        self.rx_bytes = 0
        self.valid_frames = 0
        self.buffer = bytearray()
        self._last_actuator = None

    def feed(self, data: bytes) -> None:
        self.rx_bytes += len(data)
        if self.show_rx and data:
            print(f"RX RAW +{time.monotonic() - self.started:.3f}s: {packet_text(data)}", flush=True)
        self.buffer.extend(data)
        while self.buffer:
            first = self.buffer.find(b"\xA6")
            if first < 0:
                self.buffer.clear()
                return
            if first:
                del self.buffer[:first]
            frame_size = 8
            if len(self.buffer) < frame_size:
                return
            frame = bytes(self.buffer[:frame_size])
            valid = (frame[7] == 0x0A and
                     crc16_modbus(frame[:5]) == struct.unpack_from("<H", frame, 5)[0])
            if not valid:
                del self.buffer[:1]
                continue
            del self.buffer[:frame_size]
            self.valid_frames += 1
            self._report_actuator(frame)

    def _report_actuator(self, frame: bytes) -> None:
        gripper = frame[1]
        action_id = frame[2]
        pitch_wire = struct.unpack_from("<h", frame, 3)[0]
        state = (gripper, action_id, pitch_wire)
        if not self.show_rx and state == self._last_actuator:
            return
        self._last_actuator = state
        pitch = "invalid" if not -40 <= pitch_wire <= 40 else f"{pitch_wire} deg"
        gripper_text = {0: "closed", 1: "open"}.get(gripper, f"invalid({gripper})")
        if self.show_rx:
            print(f"RX A6 [CRC OK]: {packet_text(frame)}", flush=True)
        print(f"反馈执行器: gripper={gripper_text}, "
              f"action_id={action_id}, pitch={pitch}", flush=True)

def feedback_loop(fd: int, stop_event: threading.Event, parser: FeedbackParser) -> None:
    while not stop_event.is_set():
        try:
            readable, _, _ = select.select([fd], [], [], 0.1)
        except (OSError, ValueError):
            return
        if not readable:
            continue
        try:
            data = os.read(fd, 256)
        except (OSError, InterruptedError):
            continue
        if not data:
            return
        parser.feed(data)


def parse_finite_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("must be a finite number")
    return number


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def rounded_integer(value: float) -> int:
    """Round halves away from zero, matching C++ std::round."""
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def build_latest_packet(
    vx_mps: float,
    wz_rps: float,
    gripper: str,
    action_id: int,
    pitch_deg: float,
) -> bytes:
    if not math.isfinite(vx_mps) or not math.isfinite(wz_rps):
        vx_mps = wz_rps = 0.0
    vx_mps = max(-MAX_LINEAR_SPEED_MPS, min(MAX_LINEAR_SPEED_MPS, vx_mps))
    pitch_deg = max(-MAX_PITCH_DEG, min(MAX_PITCH_DEG, pitch_deg))
    pitch_wire = rounded_integer(pitch_deg)
    payload = struct.pack(
        "<BffBBh",
        HEADER,
        vx_mps,
        wz_rps,
        1 if gripper == "open" else 0,
        action_id,
        pitch_wire,
    )
    return payload + struct.pack("<H", crc16_modbus(payload))


def build_legacy6_packet(vx_mps: float, wz_rps: float, gripper: str) -> bytes:
    if not math.isfinite(vx_mps) or not math.isfinite(wz_rps):
        vx_mmps = wz_mradps = 0
    else:
        vx_mmps = math.trunc(max(-32768.0, min(32767.0, vx_mps * 1000.0)))
        wz_mradps = math.trunc(max(-32768.0, min(32767.0, wz_rps * 1000.0)))
    return struct.pack("<BhhB", HEADER, vx_mmps, wz_mradps,
                       1 if gripper == "closed" else 0)


def build_packet(args: argparse.Namespace, vx_mps: float, wz_rps: float) -> bytes:
    if args.protocol == "latest15":
        return build_latest_packet(vx_mps, wz_rps, args.gripper,
                                   args.action_id, args.pitch)
    return build_legacy6_packet(vx_mps, wz_rps, args.gripper)


def baud_constant(baudrate: int) -> int:
    try:
        return BAUDRATES[baudrate]
    except KeyError as exc:
        supported = ", ".join(str(value) for value in sorted(BAUDRATES))
        raise ValueError(f"unsupported baudrate {baudrate}; choose one of {supported}") from exc


def open_serial(port: str, baudrate: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    try:
        attrs = termios.tcgetattr(fd)
        speed = baud_constant(baudrate)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 5
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd
    except Exception:
        os.close(fd)
        raise


def write_all(fd: int, packet: bytes) -> None:
    written = 0
    while written < len(packet):
        try:
            count = os.write(fd, packet[written:])
        except InterruptedError:
            continue
        if count <= 0:
            raise OSError("serial write returned no progress")
        written += count


def packet_text(packet: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in packet)


def transmit(args: argparse.Namespace, packet: bytes, stop_packet: bytes) -> None:
    fd: Optional[int] = None
    reader: Optional[threading.Thread] = None
    reader_stop = threading.Event()
    feedback = FeedbackParser(show_rx=args.show_rx)
    sent = False
    started = time.monotonic()
    interval = 1.0 / args.rate
    deadline = None if args.duration == 0 else started + args.duration
    try:
        fd = open_serial(args.port, args.baud)
        print(f"已打开串口 {args.port}@{args.baud}，发送频率 {args.rate:g} Hz")
        reader = threading.Thread(target=feedback_loop, args=(fd, reader_stop, feedback), daemon=True)
        reader.start()
        next_send = started
        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break
            if now < next_send:
                wait = next_send - now
                if deadline is not None:
                    wait = min(wait, deadline - now)
                time.sleep(wait)
                continue
            write_all(fd, packet)
            sent = True
            next_send += interval
            if next_send < time.monotonic() - interval:
                next_send = time.monotonic() + interval
        if args.stop_on_exit:
            print("发送结束，补发零速度停车帧")
        else:
            print("发送结束，不发送停车帧")
    finally:
        if fd is not None:
            if sent and args.stop_on_exit:
                try:
                    write_all(fd, stop_packet)
                except OSError:
                    pass
            reader_stop.set()
            if reader is not None:
                reader.join(timeout=0.3)
            os.close(fd)
            if args.show_rx:
                print(f"RX汇总: {feedback.rx_bytes} 字节，{feedback.valid_frames} 个有效A6帧", flush=True)
                if not feedback.rx_bytes:
                    print("发送期间未收到下位机数据；不代表速度已执行。", flush=True)
                elif not feedback.valid_frames:
                    print("收到字节但未解析出有效A6帧，请检查协议、波特率与CRC。", flush=True)


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="向下位机发送当前 15 字节速度/夹爪/pitch 帧"
    )
    parser.add_argument("--protocol", choices=("latest15", "legacy6"), default="latest15",
                        help="协议版本，默认 latest15；旧六字节协议用 legacy6")
    parser.add_argument("--port", default="/dev/ttyACM0", help="串口设备，默认 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--vx", type=parse_finite_float, default=0.0,
                        help="线速度 m/s，默认 0；latest15 限幅 ±0.2")
    parser.add_argument("--wz", type=parse_finite_float, default=0.0,
                        help="角速度 rad/s，默认 0")
    parser.add_argument("--gripper", choices=("open", "closed"), default="closed",
                        help="夹爪目标状态，默认 closed")
    parser.add_argument("--action-id", type=int, default=0,
                        help="latest15 夹爪动作编号 0..255；0 表示不触发新动作")
    parser.add_argument("--pitch", type=parse_finite_float, default=0.0,
                        help="latest15 相机目标角度，真实度数，限幅 ±40，默认 0")
    parser.add_argument("--rate", type=parse_finite_float, default=25.0,
                        help="重复发送频率 Hz，默认 25")
    parser.add_argument("--duration", type=parse_finite_float, default=1.0,
                        help="发送时长秒；0 表示持续到 Ctrl+C，默认 1")
    stop_group = parser.add_mutually_exclusive_group()
    stop_group.add_argument("--stop-on-exit", dest="stop_on_exit", action="store_true",
                            default=True, help="结束或 Ctrl+C 时发送零速度停车帧（默认）")
    stop_group.add_argument("--no-stop-on-exit", dest="stop_on_exit", action="store_false",
                            help="结束时不额外发送零速度停车帧")
    parser.add_argument("--show-rx", action="store_true",
                        help="发送期间逐次打印原始RX字节、有效A6帧及解析状态；不会请求固件新增速度反馈")
    parser.add_argument("--send", action="store_true",
                        help="真正打开串口并发送；不加此项只打印帧")
    return parser


def main() -> int:
    args = make_parser().parse_args()
    if args.rate <= 0:
        raise SystemExit("--rate must be greater than 0")
    if args.duration < 0:
        raise SystemExit("--duration must be zero or greater")
    if not 0 <= args.action_id <= 255:
        raise SystemExit("--action-id must be in 0..255")
    try:
        baud_constant(args.baud)
        packet = build_packet(args, args.vx, args.wz)
        stop_packet = build_packet(args, 0.0, 0.0)
        print(f"协议: {args.protocol}; 帧 ({len(packet)} bytes): {packet_text(packet)}")
        print(f"请求: vx={args.vx:g} m/s, wz={args.wz:g} rad/s, gripper={args.gripper}")
        if args.protocol == "latest15":
            print(f"动作编号: {args.action_id}; pitch={args.pitch:g} deg")
        if not args.send:
            print("预览模式：未打开串口。需要实际下发时显式添加 --send。")
            return 0
        transmit(args, packet, stop_packet)
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在停车。")
        return 130
    except (OSError, OverflowError, ValueError, termios.error) as exc:
        print(f"发送失败: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
