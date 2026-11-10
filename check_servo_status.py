import time
import serial

PORT = "COM4"
SERVO_ID = 8


def read_status(bus, command, data_size):
    """发送读取指令，并检查回复的 ID、命令、长度和校验和。"""
    body = [SERVO_ID, 3, command]
    request = bytes([
        0x55, 0x55, *body,
        (~sum(body)) & 0xFF
    ])

    bus.reset_input_buffer()
    bus.write(request)
    bus.flush()
    reply = bus.read(64)

    packet_size = data_size + 6

    for i in range(len(reply) - packet_size + 1):
        packet = reply[i:i + packet_size]

        expected_header = bytes([
            0x55, 0x55, SERVO_ID,
            data_size + 3, command
        ])

        if packet[:5] != expected_header:
            continue

        if ((~sum(packet[2:-1])) & 0xFF) != packet[-1]:
            continue

        return packet[5:-1]

    raise RuntimeError(
        f"读取命令 {command} 未收到有效回复："
        f"{reply.hex(' ') or '无数据'}"
    )


def show_result(bus, name, command, size, decode):
    try:
        data = read_status(bus, command, size)
        print(f"{name}：{decode(data)}")
    except RuntimeError as error:
        print(f"{name}：{error}")


def decode_torque(data):
    return {
        0: "关闭",
        1: "开启"
    }.get(data[0], f"未知值 {data[0]}")


def decode_mode(data):
    return {
        0: "位置控制模式",
        1: "连续旋转的电机模式"
    }.get(data[0], f"未知值 {data[0]}")


try:
    with serial.Serial(
        PORT,
        baudrate=115200,
        timeout=0.5,
        write_timeout=0.5
    ) as bus:
        time.sleep(0.2)

        print(f"检查 {PORT} 上的 ID {SERVO_ID}")
        print("本程序只读取状态。\n")

        show_result(
            bus, "当前位置", 28, 2,
            lambda d: (
                f"{int.from_bytes(d, 'little', signed=True)}"
                "（原始位置单位）"
            )
        )

        show_result(
            bus, "舵机反馈电压", 27, 2,
            lambda d: (
                f"{int.from_bytes(d, 'little') / 1000:.2f} V"
            )
        )

        show_result(
            bus, "扭矩状态", 32, 1, decode_torque
        )

        show_result(
            bus, "工作模式", 30, 4, decode_mode
        )


        show_result(
            bus, "舵机配置角度范围", 21, 4,
            lambda d: (
                f"{int.from_bytes(d[:2], 'little') * 0.24:.2f}° 至 "
                f"{int.from_bytes(d[2:], 'little') * 0.24:.2f}°"
            )
        )

        show_result(
            bus, "舵机角度偏移", 19, 1,
            lambda d: f"{int.from_bytes(d, 'little', signed=True) * 0.24:.2f}°"
        )

        def send_command(command, data):
            body = [SERVO_ID, len(data) + 3, command, *data]
            packet = bytes([
                0x55, 0x55, *body,
                (~sum(body)) & 0xFF
            ])
            bus.write(packet)
            bus.flush()


        answer = input(
            "\n确认机架固定、腿部有支撑、手已移开，"
            "输入 HOLD 开启当前位置保持："
        )

        if answer.strip() == "HOLD":
            try:
                voltage = int.from_bytes(
                    read_status(bus, 27, 2), "little"
                )
                mode = read_status(bus, 30, 4)[0]
                torque = read_status(bus, 32, 1)[0]
                position = int.from_bytes(
                    read_status(bus, 28, 2),
                    "little", signed=True
                )

                if not 9000 <= voltage <= 12600:
                    raise RuntimeError("电压不在预期范围，取消。")
                if mode != 0:
                    raise RuntimeError("不是位置控制模式，取消。")
                if torque != 0:
                    raise RuntimeError("扭矩已经开启，取消本次操作。")
                if not 0 <= position <= 1000:
                    raise RuntimeError("当前位置超出指令范围，取消。")

                # 先发送当前位置作为目标，覆盖旧目标
                send_command(1, [
                    position & 0xFF,
                    (position >> 8) & 0xFF,
                    0xD0, 0x07  # 2000 毫秒
                ])

                time.sleep(0.1)

                # 如果腿在支撑过程中移动了，先取消
                latest = int.from_bytes(
                    read_status(bus, 28, 2),
                    "little", signed=True
                )
                if abs(latest - position) > 2:
                    raise RuntimeError("腿的位置发生变化，取消，请重新运行。")

                send_command(31, [1])  # 开启扭矩
                time.sleep(0.5)

                show_result(
                    bus, "开启后的扭矩状态",
                    32, 1, decode_torque
                )
                show_result(
                    bus, "开启后的位置", 28, 2,
                    lambda d: str(
                        int.from_bytes(d, "little", signed=True)
                    )
                )
                print("测试结束。舵机仍可能保持用力，不要强扳。")

            except RuntimeError as error:
                print(error)
        else:
            print("已取消，没有开启扭矩。")

except serial.SerialException as error:
    print("串口错误：", error)