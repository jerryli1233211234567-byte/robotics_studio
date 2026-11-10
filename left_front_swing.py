from time import sleep

from pylx16a.lx16a import LX16A

SWING_DEGREES = 15.0
MOVE_MS = 7000
SETTLE_SECONDS = MOVE_MS / 1000 + 0.3

LX16A.initialize("COM4", timeout=0.3)
hip = LX16A(1)   # 左前大腿：角度减小时向前
knee = LX16A(2)  # 左前小腿：角度减小时向后

if not (hip.is_torque_enabled() and knee.is_torque_enabled()):
    raise SystemExit("有舵机的扭矩未开启，请先检查状态。")

if input("确认小腿向后有空间，输入 LIFT 开始，或按回车退出：").strip() != "LIFT":
    raise SystemExit("已退出。")

hip_start = hip.get_physical_angle()
knee_start = knee.get_physical_angle()
print(f"起点：髋 {hip_start:.1f}°，膝 {knee_start:.1f}°")

hip_target = hip_start - SWING_DEGREES
knee_target = knee_start - SWING_DEGREES
if not (0 <= hip_target <= 240 and 0 <= knee_target <= 240):
    raise SystemExit("目标超出舵机角度范围，已取消。")

knee.move(knee_target, time=MOVE_MS)
sleep(SETTLE_SECONDS)
print(f"小腿试动后：{knee.get_physical_angle():.1f}°")

choice = input(
    "脚尖抬高且大腿向前有空间时输入 SWING。直接回车让小腿回位："
).strip()

if choice == "SWING":
    hip.move(hip_target, time=MOVE_MS)
    sleep(SETTLE_SECONDS)
    print(f"大腿试动后：{hip.get_physical_angle():.1f}°")
    hip.move(hip_start, time=MOVE_MS)
    sleep(SETTLE_SECONDS)

knee.move(knee_start, time=MOVE_MS)
sleep(SETTLE_SECONDS)
print(f"回位：髋 {hip.get_physical_angle():.1f}°，膝 {knee.get_physical_angle():.1f}°")