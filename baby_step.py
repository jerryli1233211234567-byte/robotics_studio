
from time import sleep

from pylx16a.lx16a import LX16A

PORT = "COM4"  # 改成设备管理器里确认的端口
SERVO_ID = 8
# 默认先试 1。如果已读出实际 ID，填实际值

STEP_DEGREES = 20.0
MOVE_MS = 3000

LX16A.initialize(PORT, timeout=0.1)
servo = LX16A(SERVO_ID)

start_angle = servo.get_physical_angle()
print(f"舵机 ID {SERVO_ID}，当前位置 {start_angle:.1f}°")

direction = input("回车退出。确认关节有空间后，输入 + 或 - 试动 20°：").strip()

if direction in ("+", "-"):
    start_angle = servo.get_physical_angle()
    target_angle = start_angle + (STEP_DEGREES if direction == "+" else -STEP_DEGREES)

    if not 0 <= target_angle <= 240:
        raise SystemExit("目标角度超出范围，已停止。")

    print(f"去程：{start_angle:.1f}° → {target_angle:.1f}°")
    servo.move(target_angle, time=MOVE_MS)
    sleep(3.3)
    print(f"试动后：{servo.get_physical_angle():.1f}°")

    sleep(2)
    print(f"开始回位，目标 {start_angle:.1f}°")
    servo.move(start_angle, time=MOVE_MS)
    sleep(3.3)
    print(f"回位后：{servo.get_physical_angle():.1f}°")