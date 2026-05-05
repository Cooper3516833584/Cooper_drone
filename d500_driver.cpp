#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <cstring>

#define RADAR_PORT "/dev/ttySTM4"
#define BAUD_RATE B230400

// 利用 C++ struct 内存对齐特性，实现 Zero-copy (零拷贝) 极速解包
#pragma pack(push, 1) // 强制 1 字节对齐，严禁编译器填充
struct D500_Frame {
    uint8_t  header;        // 0x54
    uint8_t  ver_len;       // 0x2C
    uint16_t speed;         // 转速
    uint16_t start_angle;   // 起始角度
    uint8_t  point_data[36];// 12个点的数据 (距离+强度)
    uint16_t end_angle;     // 结束角度
    uint16_t timestamp;     // 时间戳
    uint8_t  crc8;          // 校验和
};
#pragma pack(pop)

int main() {
    // 1. 以后台非阻塞模式打开硬件节点
    int fd = open(RADAR_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        std::cerr << "[!] 致命错误: 无法接管 TTY 设备" << std::endl;
        return -1;
    }

    // 2. 注入 Raw Mode (原始透传模式)
    struct termios tty;
    tcgetattr(fd, &tty);
    cfmakeraw(&tty); // 斩断所有内核 TTY 控制字符转换
    cfsetispeed(&tty, BAUD_RATE);
    cfsetospeed(&tty, BAUD_RATE);
    tcsetattr(fd, TCSANOW, &tty);

    // 3. 阻塞读取与内存映射解包
    fcntl(fd, F_SETFL, 0); // 恢复阻塞模式降低 CPU 轮询占空比
    uint8_t buffer[47];
    uint8_t state = 0;
    int idx = 0;

    std::cout << "[*] 高性能 C++ 驱动点火，等待数据流..." << std::endl;

    while (true) {
        uint8_t byte;
        if (read(fd, &byte, 1) > 0) {
            // 极简状态机锁定帧头
            if (state == 0 && byte == 0x54) { state = 1; buffer[0] = byte; }
            else if (state == 1 && byte == 0x2C) { state = 2; buffer[1] = byte; idx = 2; }
            else if (state == 2) {
                buffer[idx++] = byte;
                if (idx == 47) {
                    // Zero-copy 瞬间强转解析
                    D500_Frame* frame = reinterpret_cast<D500_Frame*>(buffer);
                    float start_deg = frame->start_angle / 100.0f;
                    float end_deg = frame->end_angle / 100.0f;
                    
                    std::cout << "[+] 帧捕获 | 起始角: " << start_deg 
                              << "° | 结束角: " << end_deg 
                              << "° | 时间戳: " << frame->timestamp << "ms\n";
                    
                    state = 0; // 状态机复位
                }
            } else { state = 0; }
        }
    }
    close(fd);
    return 0;
}