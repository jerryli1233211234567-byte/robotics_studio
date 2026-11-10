from time import sleep

import serial

with serial.Serial("COM4", 115200, timeout=0.1, write_timeout=0.5) as bus:
    sleep(0.2)
    print("开始扫描，只读取位置。", flush=True)
    found = []

    for servo_id in range(254):
        body = bytes([servo_id, 3, 28])
        request = b"\x55\x55" + body + bytes([(~sum(body)) & 0xFF])

        bus.reset_input_buffer()
        bus.write(request)
        bus.flush()
        reply = bus.read(32)

        header = bytes([0x55, 0x55, servo_id, 5, 28])
        for offset in range(len(reply) - 7):
            packet = reply[offset:offset + 8]
            if packet[:5] != header:
                continue
            if ((~sum(packet[2:-1])) & 0xFF) != packet[-1]:
                continue

            position = int.from_bytes(packet[5:7], "little", signed=True)
            print(f"ID {servo_id}：{position * 0.24:.2f}°", flush=True)
            found.append(servo_id)
            break

    print("有有效回复的 ID：", found)