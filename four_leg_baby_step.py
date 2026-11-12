from argparse import ArgumentParser
from time import sleep

import serial


PORT = "/dev/ttyUSB0"
STEP_DEGREES = 30
MOVE_MS = 3000
DEGREES_PER_UNIT = 0.24
STEP_UNITS = round(STEP_DEGREES / DEGREES_PER_UNIT)
TOLERANCE = round(3 / DEGREES_PER_UNIT)

# Pairs of diagonal legs. Each tuple holds the name, upper arm ID and
# forward sign, and lower arm ID and backward sign.
DIAGONALS = (
    (("左前", 1, -1, 2, -1), ("右后", 5, -1, 6, -1)),
    (("右前", 3, 1, 4, -1), ("左后", 7, -1, 8, 1)),
)


def send(bus, servo_id, command, data=b""):
    body = bytes((servo_id, len(data) + 3, command)) + data
    bus.write(b"\x55\x55" + body + bytes([(~sum(body)) & 0xFF]))
    bus.flush()


def read(bus, servo_id, command, size):
    bus.reset_input_buffer()
    send(bus, servo_id, command)
    reply = bus.read(32)
    header = bytes((0x55, 0x55, servo_id, size + 3, command))
    length = size + 6
    for index in range(len(reply) - length + 1):
        packet = reply[index:index + length]
        if packet[:5] == header and ((~sum(packet[2:-1])) & 0xFF) == packet[-1]:
            return packet[5:-1]
    raise RuntimeError(f"ID {servo_id} 没有回复命令 {command}")


def position(bus, servo_id):
    return int.from_bytes(read(bus, servo_id, 28, 2), "little", signed=True)


def move(bus, servo_id, target):
    send(bus, servo_id, 1, target.to_bytes(2, "little") + MOVE_MS.to_bytes(2, "little"))


def move_phase(bus, label, targets):
    print(label, flush=True)
    for servo_id, target in targets.items():
        move(bus, servo_id, target)
    sleep(MOVE_MS / 1000 + 0.3)
    for servo_id, target in targets.items():
        actual = position(bus, servo_id)
        if abs(actual - target) > TOLERANCE:
            raise RuntimeError(
                f"ID {servo_id} 目标 {target * DEGREES_PER_UNIT:.1f}°，"
                f"实测 {actual * DEGREES_PER_UNIT:.1f}°"
            )


def main():
    parser = ArgumentParser(description="两组对角腿交替迈步。")
    parser.add_argument("--cycles", type=int, default=1, help="完整循环次数，默认 1。")
    parser.add_argument("--lift-forward", action="store_true", help="用小腿向前摆的方向尝试抬脚。")
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error("--cycles 必须至少为 1。")

    print(f"关节指令幅度 {STEP_DEGREES}°，运行 {args.cycles} 轮。")
    if input("确认机身有防倾倒支撑、连杆有活动空间。输入 STEP 开始：").strip() != "STEP":
        return

    with serial.Serial(PORT, 115200, timeout=0.15, write_timeout=0.5) as bus:
        sleep(0.2)
        voltage_mv = int.from_bytes(read(bus, 1, 27, 2), "little")
        if not 9000 <= voltage_mv <= 12600:
            raise RuntimeError(f"舵机供电读数 {voltage_mv / 1000:.2f} V 超出工作范围")
        home = {}
        targets = {}
        torque = {}
        for pair in DIAGONALS:
            for _, upper, forward, lower, backward in pair:
                lift = -backward if args.lift_forward else backward
                for servo_id, direction in ((upper, forward), (lower, lift)):
                    home[servo_id] = position(bus, servo_id)
                    targets[servo_id] = home[servo_id] + direction * STEP_UNITS
                    limits = read(bus, servo_id, 21, 4)
                    low = int.from_bytes(limits[:2], "little")
                    high = int.from_bytes(limits[2:], "little")
                    torque[servo_id] = read(bus, servo_id, 32, 1)[0]
                    if not (
                        0 <= low <= high <= 1000
                        and low <= home[servo_id] <= high
                        and low <= targets[servo_id] <= high
                        and read(bus, servo_id, 30, 4)[0] == 0
                        and torque[servo_id] in (0, 1)
                    ):
                        raise RuntimeError(f"ID {servo_id} 的位置、限位或工作模式异常")

        active = []
        try:
            for servo_id, angle in home.items():
                if abs(position(bus, servo_id) - angle) > TOLERANCE:
                    raise RuntimeError(f"ID {servo_id} 的起始位置发生变化")
                active.append(servo_id)
                if not torque[servo_id]:
                    move(bus, servo_id, angle)
                    send(bus, servo_id, 31, b"\x01")
                    if read(bus, servo_id, 32, 1) != b"\x01":
                        raise RuntimeError(f"ID {servo_id} 未确认扭矩开启")

            planted = ()
            for cycle in range(1, args.cycles + 1):
                print(f"第 {cycle}/{args.cycles} 轮", flush=True)
                for pair in DIAGONALS:
                    names = " + ".join(leg[0] for leg in pair)
                    uppers = [leg[1] for leg in pair]
                    lowers = [leg[3] for leg in pair]
                    move_phase(bus, f"{names} 抬脚指令", {i: targets[i] for i in lowers})
                    swing_and_push = {i: targets[i] for i in uppers}
                    swing_and_push.update({i: home[i] for i in planted})
                    move_phase(bus, f"{names} 前伸，支撑腿回摆", swing_and_push)
                    move_phase(bus, f"{names} 放脚指令", {i: home[i] for i in lowers})
                    planted = uppers
        except (RuntimeError, serial.SerialException, KeyboardInterrupt):
            for servo_id in active:
                try:
                    send(bus, servo_id, 12)
                except serial.SerialException:
                    print(f"ID {servo_id} 无法发送停止指令。")
            raise

    print("动作完成，保持最后落脚姿势。舵机仍可能保持扭矩。")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(f"已停止：{error}") from None
    except KeyboardInterrupt:
        raise SystemExit("已取消。") from None
