uint8_t crc(uint8_t* msg_buf, int8_t nbytes)
{
    uint8_t crc_reg = 0xff, poly, byte_count, bit_count;
    uint8_t* byte_point;
    uint8_t bit_point;

    for (byte_count = 0, byte_point = msg_buf; byte_count < nbytes; ++byte_count, ++byte_point)
    {
        for (bit_count = 0, bit_point = 0x80; bit_count < 8; ++bit_count, bit_point >>= 1)
        {
            if (bit_point & *byte_point) // case for new bit = 1
            {
                if (crc_reg & 0x80)
                    poly = 1; // define the polynomial
                else
                    poly = 0x1c;
                crc_reg = ((crc_reg << 1) | 1) ^ poly;
            }
            else // case for new bit = 0
            {
                poly = 0;
                if (crc_reg & 0x80)
                    poly = 0x1d;
                crc_reg = (crc_reg << 1) ^ poly;
            }
        }
    }
    return ~crc_reg; // Return CRC
}
