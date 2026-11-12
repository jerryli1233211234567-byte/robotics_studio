from time import sleep

import serial


PORT = "/dev/ttyUSB0"
STEP_DEGREES = 20
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


def move_pair(bus, label, targets):
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
    if input("支稳机身、四脚悬空、确认连杆有空间。输入 STEP 开始一轮：").strip() != "STEP":
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
                for servo_id, direction in ((upper, forward), (lower, backward)):
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

        for pair in DIAGONALS:
            print(" + ".join(leg[0] for leg in pair), flush=True)
            uppers = [leg[1] for leg in pair]
            lowers = [leg[3] for leg in pair]
            active = []
            completed = False
            try:
                for servo_id in uppers + lowers:
                    if abs(position(bus, servo_id) - home[servo_id]) > TOLERANCE:
                        raise RuntimeError(f"ID {servo_id} 的起始位置发生变化")
                    active.append(servo_id)
                    if not torque[servo_id]:
                        move(bus, servo_id, home[servo_id])
                        send(bus, servo_id, 31, b"\x01")
                        if read(bus, servo_id, 32, 1) != b"\x01":
                            raise RuntimeError(f"ID {servo_id} 未确认扭矩开启")

                move_pair(bus, "小腿向后收", {i: targets[i] for i in lowers})
                move_pair(bus, "大腿向前摆", {i: targets[i] for i in uppers})
                move_pair(bus, "小腿回位", {i: home[i] for i in lowers})
                move_pair(bus, "大腿回位", {i: home[i] for i in uppers})
                completed = True
            finally:
                for servo_id in active:
                    if not completed or not torque[servo_id]:
                        try:
                            send(bus, servo_id, 31, b"\x00")
                        except serial.SerialException:
                            print(f"ID {servo_id} 无法发送关闭扭矩指令，请断开舵机电池。")

    print("四条腿完成一轮。")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(f"已停止：{error}") from None
    except KeyboardInterrupt:
        raise SystemExit("已取消。") from None
