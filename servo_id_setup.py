import time
import serial


def send(bus, servo_id, command, data=()):
    body = [servo_id, len(data) + 3, command, *data]
    bus.write(bytes([
        0x55, 0x55, *body,
        (~sum(body)) & 0xFF
    ]))
    bus.flush()


def read_position(bus, servo_id):
    bus.reset_input_buffer()
    send(bus, servo_id, 28)
    reply = bus.read(64)

    for i in range(len(reply) - 7):
        packet = reply[i:i + 8]

        if packet[:5] != bytes([
            0x55, 0x55, servo_id, 5, 28
        ]):
            continue

        if ((~sum(packet[2:-1])) & 0xFF) != packet[-1]:
            continue

        return int.from_bytes(
            packet[5:7], "little", signed=True
        )

    return None


PORT = "/dev/ttyUSB0"


def configure_leg(leg_name, upper_id, lower_id):
    targets = {"UPPER": upper_id, "LOWER": lower_id}
    print(f"{leg_name}：upper_arm = {upper_id}，lower_arm = {lower_id}")
    choice = input(
        "回车只读两个 ID。若要改号，请只接目标舵机，再输入 UPPER 或 LOWER："
    ).strip().upper()
    if choice and choice not in targets:
        print("已取消。")
        return

    try:
        with serial.Serial(PORT, 115200, timeout=0.5, write_timeout=0.5) as bus:
            time.sleep(0.2)
            if not choice:
                for arm, servo_id in targets.items():
                    position = read_position(bus, servo_id)
                    result = "无有效回复" if position is None else f"{position * 0.24:.2f}°"
                    print(f"{arm.lower()}_arm，ID {servo_id}：{result}")
                return

            arm_name = f"{choice.lower()}_arm"
            target_id = targets[choice]
            bus.timeout = 0.1
            print("正在只读扫描，约需半分钟。", flush=True)
            found = [
                servo_id for servo_id in range(254)
                if read_position(bus, servo_id) is not None
            ]
            print(f"有有效回复的 ID：{found}")
            if len(found) != 1:
                print("需要恰好一个有效 ID，已取消改号。")
                return

            old_id = found[0]
            if old_id == target_id:
                print(f"当前接入的舵机已经是 ID {target_id}。")
                return

            confirmation = f"SET{target_id}"
            answer = input(
                f"确认只接了{leg_name} {arm_name}，"
                f"输入 {confirmation} 将 ID {old_id} 改为 ID {target_id}："
            ).strip()
            if answer != confirmation:
                print("已取消。")
                return

            bus.timeout = 0.5
            if read_position(bus, target_id) is not None or read_position(bus, old_id) is None:
                print("目标 ID 已有回复，或原 ID 无法读取，已取消改号。")
                return

            send(bus, old_id, 13, [target_id])
            time.sleep(0.3)
            position = read_position(bus, target_id)
            if position is None or read_position(bus, old_id) is not None:
                print("未能确认改号成功。保持单独连接，再运行扫描脚本确认。")
                return

            print(f"改号成功：{leg_name} {arm_name}，ID {target_id}，位置 {position * 0.24:.2f}°。")
    except serial.SerialException as error:
        print("串口错误：", error)
