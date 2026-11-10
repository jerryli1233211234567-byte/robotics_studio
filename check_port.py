import serial
from serial.tools import list_ports

ports = list(list_ports.comports())

print("当前串口：")
for port in ports:
    print(f"{port.device}: {port.description}")

PORT = "COM4"

try:
    with serial.Serial(
        PORT,
        baudrate=115200,
        timeout=0.5,
        write_timeout=0.5,
    ) as connection:
        print(f"\n成功打开 {connection.port}")
        print("没有发送任何舵机指令。")
        print("这只证明串口可打开，尚未确认舵机通信。")

except serial.SerialException as error:
    print(f"\n串口打开失败：{error}")