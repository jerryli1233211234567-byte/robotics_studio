import time
import serial

PORT = "/dev/ttyUSB0"
SERVO_ID = 1

# 位置读取指令：ID、长度、命令
body = [SERVO_ID, 3, 28]
checksum = (~sum(body)) & 0xFF
request = bytes([0x55, 0x55, *body, checksum])

try:
    with serial.Serial(
        PORT, 115200,
        timeout=0.5,
        write_timeout=0.5
    ) as bus:
        time.sleep(0.2)
        bus.reset_input_buffer()

        bus.write(request)
        bus.flush()
        reply = bus.read(64)

        print("收到的数据：", reply.hex(" ") or "无")

        # 查找完整、校验正确的位置回复，跳过可能的回显
        found = False

        for i in range(len(reply) - 7):
            packet = reply[i:i + 8]

            if packet[:2] != b"\x55\x55":
                continue

            if packet[2:5] != bytes([SERVO_ID, 5, 28]):
                continue

            if ((~sum(packet[2:-1])) & 0xFF) != packet[-1]:
                continue

            position = int.from_bytes(
                packet[5:7], "little", signed=True
            )

            print(f"ID {SERVO_ID} 有效位置回复：{position}")
            print("按老师示例协议换算角度：",
                  round(position * 240 / 1000, 2), "度")
            found = True

            # 从本次读取的位置增加 8 个单位，约 1.92°
            target = position + 8  # 约 1.92°
            duration_ms = 2000

            if not 0 <= target <= 1000:
                print("目标超出协议位置范围，取消动作。")
                break

            answer = input(
                "确认脚悬空、连杆有活动余量后，输入 MOVE 再按回车："
            )

            if answer.strip() != "MOVE":
                print("已取消。")
                break

            # 等待确认期间如果机构位置发生变化，取消本次测试
            bus.reset_input_buffer()
            bus.write(request)
            bus.flush()
            check = bus.read(64)

            valid_position = None

            for j in range(len(check) - 7):
                p = check[j:j + 8]
                if (
                        p[:5] == bytes([0x55, 0x55, SERVO_ID, 5, 28])
                        and ((~sum(p[2:-1])) & 0xFF) == p[-1]
                ):
                    valid_position = int.from_bytes(
                        p[5:7], "little", signed=True
                    )
                    break

            if valid_position is None or abs(valid_position - position) > 2:
                print("位置读取失败或姿态发生变化，取消动作，请重新运行。")
                break

            # 单次位置指令：用两秒移动约 2°
            move_body = [
                SERVO_ID, 7, 1,
                target & 0xFF, (target >> 8) & 0xFF,
                duration_ms & 0xFF, (duration_ms >> 8) & 0xFF,
            ]
            move_packet = bytes([
                0x55, 0x55,
                *move_body,
                (~sum(move_body)) & 0xFF
            ])

            bus.write(move_packet)
            bus.flush()
            print("已发送单次约 15° 的移动指令，不会循环。")
            time.sleep(2.5)
            break

        if not found:
            print("未收到 ID 1 的有效位置回复。")
            print("先检查供电、接线、ID 或协议，不运行运动程序。")

except serial.SerialException as error:
    print("串口错误：", error)