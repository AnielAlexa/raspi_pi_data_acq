/**
 * Verify CRC calculation on actual packet data
 */
#include <cstdio>
#include <cstdint>

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

int main() {
    // First IMU packet from Pico output:
    uint8_t packet[] = {
        0x55, 0xAA,  // header
        0x09, 0x37, 0x6B, 0x2F,  // timestamp (4 bytes)
        0xF0, 0xBF, 0x8B, 0x3D,  // ax (float)
        0xB7, 0x2B, 0x29, 0xBD,  // ay (float)
        0xA0, 0x91, 0x1E, 0xC1,  // az (float)
        0x55, 0xE6, 0x62, 0x3B,  // gx (float)
        0x27, 0x72, 0xD1, 0xBB,  // gy (float)
        0x6F, 0xA1, 0x8B, 0xB9,  // gz (float)
        0x8D, 0x37  // CRC (last 2 bytes)
    };

    printf("Packet (32 bytes):\n");
    for (int i = 0; i < 32; i++) {
        printf("%02X ", packet[i]);
        if ((i+1) % 16 == 0) printf("\n");
    }
    printf("\n");

    // Calculate CRC on first 30 bytes (exclude last 2 bytes which are the CRC)
    uint16_t calc_crc = calculate_crc16(packet, 30);
    uint16_t packet_crc = packet[30] | (packet[31] << 8);  // Little-endian

    printf("Calculated CRC: 0x%04X\n", calc_crc);
    printf("Packet CRC:     0x%04X\n", packet_crc);

    if (calc_crc == packet_crc) {
        printf("✓ CRC MATCH!\n");
    } else {
        printf("✗ CRC MISMATCH!\n");
    }

    return 0;
}
