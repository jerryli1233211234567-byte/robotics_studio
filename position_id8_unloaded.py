from time import sleep

import serial

from servo_id_setup import PORT, send

SERVO_ID = 8
CENTER = 500  # 120 degrees
TOLERANCE = 8  # About 1.9 degrees


def read(bus, command, size):
    bus.reset_input_buffer()
    send(bus, SERVO_ID, command)
    reply = bus.read(64)
    header = bytes([0x55, 0x55, SERVO_ID, size + 3, command])
    length = size + 6
    for index in range(len(reply) - length + 1):
        packet = reply[index:index + length]
        if packet[:5] == header and (~sum(packet[2:-1]) & 0xFF) == packet[-1]:
            return packet[5:-1]
    raise RuntimeError(f"ID 8：读取命令 {command} 没有有效回复。")


def position(bus):
    return int.from_bytes(read(bus, 28, 2), "little", signed=True)


def set_target(bus, target, duration_ms):
    data = target.to_bytes(2, "little") + duration_ms.to_bytes(2, "little")
    send(bus, SERVO_ID, 1, data)
    actual_target = int.from_bytes(read(bus, 2, 4)[:2], "little")
    if actual_target != target:
        raise RuntimeError("舵机没有确认目标角度。")


def verify_position(bus, target):
    actual = position(bus)
    print(f"实际位置：{actual * 0.24:.2f}°")
    if abs(actual - target) > TOLERANCE:
        raise RuntimeError("未到达目标附近。")


def main():
    with serial.Serial(PORT, 115200, timeout=0.2, write_timeout=0.5) as bus:
        sleep(0.2)
        current = position(bus)
        voltage = int.from_bytes(read(bus, 27, 2), "little") / 1000
        mode = read(bus, 30, 4)[0]
        torque = read(bus, 32, 1)[0]
        limits = read(bus, 21, 4)
        lower = int.from_bytes(limits[:2], "little")
        upper = int.from_bytes(limits[2:], "little")
        print(f"ID 8：当前位置 {current * 0.24:.2f}°，供电 {voltage:.2f} V。")
        if not (9 <= voltage <= 12.6 and mode == 0 and torque in (0, 1)):
            raise RuntimeError("电压、工作模式或扭矩状态异常。")
        if not 0 <= lower <= CENTER <= upper <= 1000:
            raise RuntimeError("配置的限位不允许使用 120°。")

        initial_target = min(max(current, lower), upper)
        print(f"预计先到 {initial_target * 0.24:.2f}°，再到 120°。")
        answer = input(
            "确认只接 ID 8、舵机已固定、输出臂与连杆分离且手已移开。"
            "输入 CENTER8 开始，或按回车退出："
        )
        if answer.strip() != "CENTER8":
            print("已退出。")
            return

        if abs(position(bus) - current) > 2 or read(bus, 32, 1)[0] != torque:
            raise RuntimeError("确认期间位置或扭矩状态发生变化，请重新运行。")

        completed = False
        try:
            set_target(bus, initial_target, 5000)
            if torque == 0:
                send(bus, SERVO_ID, 31, [1])
                if read(bus, 32, 1)[0] != 1:
                    raise RuntimeError("未能确认扭矩开启。")
            sleep(5.3)
            verify_position(bus, initial_target)

            if initial_target != CENTER:
                print("正在转到 120°，约 10 秒。", flush=True)
                set_target(bus, CENTER, 10000)
                sleep(10.3)
                verify_position(bus, CENTER)
            completed = True
        finally:
            if not completed:
                try:
                    send(bus, SERVO_ID, 31, [0])
                except serial.SerialException:
                    print("无法确认扭矩关闭，请断开电池。")

        print("定位完成。舵机仍在保持位置，请断开电池和 USB 后再装回连杆。")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(f"已停止：{error}") from None
    except KeyboardInterrupt:
        raise SystemExit("已取消。") from None
