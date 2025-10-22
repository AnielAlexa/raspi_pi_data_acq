/**
 * Simple serial diagnostic tool to check Pico data
 * Compile: g++ -o test_pico_serial test_pico_serial.cpp
 * Run: ./test_pico_serial
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdint>

#pragma pack(push, 1)
struct ImuPacket {
    uint16_t header;
    uint32_t timestamp_us;
    float ax, ay, az;
    float gx, gy, gz;
    uint16_t crc16;
};

struct TriggerPacket {
    uint16_t header;
    uint32_t timestamp_us;
    uint16_t frame_id;
    uint16_t reserved;
    uint16_t crc16;
};
#pragma pack(pop)

uint16_t calculate_crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int main()
{
    printf("Opening /dev/ttyACM0...\n");

    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("Error opening serial port\n");
        return 1;
    }

    struct termios tty;
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    tcsetattr(fd, TCSANOW, &tty);

    printf("Serial port configured. Reading data...\n");
    printf("Looking for headers: 0xAA55 (IMU) and 0xBB66 (Trigger)\n\n");

    uint8_t buffer[512];
    int buf_len = 0;
    int imu_count = 0;
    int trigger_count = 0;
    int crc_errors = 0;

    while (true) {
        uint8_t byte;
        int n = read(fd, &byte, 1);

        if (n > 0) {
            buffer[buf_len++] = byte;

            // Look for packet start
            if (buf_len >= 2) {
                uint16_t header = buffer[buf_len-2] | (buffer[buf_len-1] << 8);

                // Check for IMU packet
                if (header == 0xAA55 && buf_len >= sizeof(ImuPacket)) {
                    int start = buf_len - sizeof(ImuPacket);
                    ImuPacket pkt;
                    memcpy(&pkt, &buffer[start], sizeof(ImuPacket));

                    uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(ImuPacket) - 2);

                    if (calc_crc == pkt.crc16) {
                        printf("✓ IMU Packet #%d: ts=%u us, ax=%.2f, crc=OK\n",
                               ++imu_count, pkt.timestamp_us, pkt.ax);
                    } else {
                        printf("✗ IMU CRC ERROR: calc=0x%04X, pkt=0x%04X\n", calc_crc, pkt.crc16);
                        crc_errors++;
                    }

                    // Clear buffer
                    memmove(buffer, &buffer[start + sizeof(ImuPacket)],
                           buf_len - start - sizeof(ImuPacket));
                    buf_len -= start + sizeof(ImuPacket);
                }
                // Check for Trigger packet
                else if (header == 0xBB66 && buf_len >= sizeof(TriggerPacket)) {
                    int start = buf_len - sizeof(TriggerPacket);
                    TriggerPacket pkt;
                    memcpy(&pkt, &buffer[start], sizeof(TriggerPacket));

                    uint16_t calc_crc = calculate_crc16((uint8_t*)&pkt, sizeof(TriggerPacket) - 2);

                    if (calc_crc == pkt.crc16) {
                        printf("✓ TRIGGER Packet #%d: ts=%u us, frame_id=%d, crc=OK\n",
                               ++trigger_count, pkt.timestamp_us, pkt.frame_id);
                    } else {
                        printf("✗ TRIGGER CRC ERROR: calc=0x%04X, pkt=0x%04X\n", calc_crc, pkt.crc16);
                        crc_errors++;
                    }

                    // Clear buffer
                    memmove(buffer, &buffer[start + sizeof(TriggerPacket)],
                           buf_len - start - sizeof(TriggerPacket));
                    buf_len -= start + sizeof(TriggerPacket);
                }
            }

            // Prevent overflow
            if (buf_len > 400) {
                memmove(buffer, &buffer[200], buf_len - 200);
                buf_len = buf_len - 200;
            }
        }

        // Print stats every 100 IMU packets
        if (imu_count > 0 && imu_count % 100 == 0) {
            printf("\n--- Stats: IMU=%d, Trigger=%d, CRC_Errors=%d ---\n\n",
                   imu_count, trigger_count, crc_errors);
        }
    }

    close(fd);
    return 0;
}
