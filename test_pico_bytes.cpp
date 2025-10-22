/**
 * Raw byte inspector - shows actual bytes from Pico
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstdint>

void print_hex(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    if (len % 16 != 0) printf("\n");
}

int main()
{
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("Error opening serial\n");
        return 1;
    }

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag = 0;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);

    printf("Reading raw bytes from Pico...\n");
    printf("Looking for: 55 AA (IMU header) or 66 BB (Trigger header)\n\n");

    uint8_t buffer[128];
    int count = 0;

    while (count < 100) {  // Read 100 bytes
        uint8_t byte;
        int n = read(fd, &byte, 1);
        if (n > 0) {
            buffer[count++] = byte;

            // Check for headers
            if (count >= 2) {
                if (buffer[count-2] == 0x55 && buffer[count-1] == 0xAA) {
                    printf("\n=== Found IMU header at byte %d ===\n", count-2);
                    if (count >= 32) {
                        printf("Last 32 bytes (should be IMU packet):\n");
                        print_hex(&buffer[count-32], 32);
                    }
                } else if (buffer[count-2] == 0x66 && buffer[count-1] == 0xBB) {
                    printf("\n=== Found TRIGGER header at byte %d ===\n", count-2);
                    if (count >= 12) {
                        printf("Last 12 bytes (should be Trigger packet):\n");
                        print_hex(&buffer[count-12], 12);
                    }
                }
            }
        }
    }

    printf("\nAll 100 bytes:\n");
    print_hex(buffer, count);

    close(fd);
    return 0;
}
