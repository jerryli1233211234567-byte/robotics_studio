from argparse import ArgumentParser
from time import sleep

import serial

from four_leg_baby_step import DEGREES_PER_UNIT, PORT, TOLERANCE, move, move_phase, position, read, send


STAND_DEGREES = {
    1: 111.60, 2: 100.08,
    3: 119.52, 4: 90.96,
    5: 112.32, 6: 89.76,
    7: 111.84, 8: 157.44,
}


def main():
    ArgumentParser(description="恢复四足机器人当前的默认站姿。").parse_args()
    targets = {
        servo_id: round(degrees / DEGREES_PER_UNIT)
        for servo_id, degrees in STAND_DEGREES.items()
    }

    with serial.Serial(PORT, 115200, timeout=0.15, write_timeout=0.5) as bus:
        sleep(0.2)
        voltage_mv = int.from_bytes(read(bus, 1, 27, 2), "little")
        if not 9000 <= voltage_mv <= 12600:
            raise RuntimeError(f"舵机供电读数 {voltage_mv / 1000:.2f} V 超出工作范围")

        pending = {}
        for servo_id, target in targets.items():
            current = position(bus, servo_id)
            print(f"ID {servo_id}: {current * DEGREES_PER_UNIT:.2f}° → {target * DEGREES_PER_UNIT:.2f}°")
            if abs(current - target) <= 1:
                continue

            limits = read(bus, servo_id, 21, 4)
            low = int.from_bytes(limits[:2], "little")
            high = int.from_bytes(limits[2:], "little")
            torque = read(bus, servo_id, 32, 1)[0]
            if not (
                0 <= low <= high <= 1000
                and low <= current <= high
                and low <= target <= high
                and read(bus, servo_id, 30, 4)[0] == 0
                and torque in (0, 1)
            ):
                raise RuntimeError(f"ID {servo_id} 的位置、限位或工作模式异常")
            pending[servo_id] = (current, target, torque)

        if not pending:
            print("所选舵机都已在目标角度附近。")
            return
        if input("支稳机身、四脚悬空，确认连杆有空间。输入 RESTORE 开始：").strip() != "RESTORE":
            return

        active = []
        try:
            for servo_id, (current, target, torque) in pending.items():
                actual = position(bus, servo_id)
                if abs(actual - current) > TOLERANCE:
                    raise RuntimeError(f"ID {servo_id} 的起始位置发生变化")
                active.append(servo_id)
                if not torque:
                    move(bus, servo_id, actual)
                    send(bus, servo_id, 31, b"\x01")
                    if read(bus, servo_id, 32, 1) != b"\x01":
                        raise RuntimeError(f"ID {servo_id} 未确认扭矩开启")
                move_phase(bus, f"ID {servo_id} 移到目标角度", {servo_id: target})

            for servo_id, target in targets.items():
                actual = position(bus, servo_id)
                print(f"ID {servo_id} 回读 {actual * DEGREES_PER_UNIT:.2f}°")
                if abs(actual - target) > TOLERANCE:
                    raise RuntimeError(f"ID {servo_id} 离目标超过允许偏差")
        except (RuntimeError, serial.SerialException, KeyboardInterrupt):
            for servo_id in active:
                try:
                    send(bus, servo_id, 12)
                except serial.SerialException:
                    print(f"ID {servo_id} 无法发送停止指令。")
            raise

    print("回站姿完成。")
    print("舵机仍可能保持扭矩。")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(f"已停止：{error}") from None
    except KeyboardInterrupt:
        raise SystemExit("已取消。") from None
