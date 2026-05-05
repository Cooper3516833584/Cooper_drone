import serial
import time

SERIAL_PORT = '/dev/ttySTM4'  # 替换为实际节点
BAUD_RATE = 230400

def verify_radar():
    print(f"[*] 正在接管 {SERIAL_PORT} @ {BAUD_RATE}bps...")
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            # 清空启动时可能存在的内核缓冲区脏数据
            ser.reset_input_buffer()
            
            while True:
                # 状态机：寻找双字节帧头 0x54 0x2C
                if ser.read(1) == b'\x54':
                    if ser.read(1) == b'\x2c':
                        # D500 定长帧，帧头后还有 45 字节
                        payload = ser.read(45)
                        if len(payload) == 45:
                            # 雷达转速 (Byte 0-1, 小端序)
                            speed_deg_s = int.from_bytes(payload[0:2], byteorder='little')
                            # 起始角度 (Byte 2-3, 小端序, 需除以100)
                            start_angle = int.from_bytes(payload[2:4], byteorder='little') / 100.0
                            
                            print("\n[+] 链路验证成功！成功解包首帧：")
                            print(f" ├─ 雷达物理转速 : {speed_deg_s} deg/s ({speed_deg_s/360:.1f} Hz)")
                            print(f" └─ 本帧起始角度 : {start_angle}°")
                            break
                        else:
                            print("[-] 帧残缺，可能发生 DMA 截断")
                            
    except Exception as e:
        print(f"[!] 致命异常: {e}")

if __name__ == '__main__':
    verify_radar()