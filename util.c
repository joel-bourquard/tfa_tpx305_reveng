/** @file
    Various utility functions for use by device drivers.

    Copyright (C) 2015 Tommy Vestermark

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

//#include "util.h"
#include <stdint.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

uint8_t reverse8(uint8_t x)
{
    x = (x & 0xF0) >> 4 | (x & 0x0F) << 4;
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2;
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1;
    return x;
}

uint32_t reverse32(uint32_t x)
{
    uint32_t ret;
    uint8_t* xp = (uint8_t*)&x;
    ret = (uint32_t) reverse8(xp[0]) << 24 | reverse8(xp[1]) << 16 | reverse8(xp[2]) << 8 | reverse8(xp[3]);
    return ret;
}

void reflect_bytes(uint8_t message[], unsigned num_bytes)
{
    for (unsigned i = 0; i < num_bytes; ++i) {
        message[i] = reverse8(message[i]);
    }
}

uint8_t reflect4(uint8_t x)
{
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2;
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1;
    return x;
}

void reflect_nibbles(uint8_t message[], unsigned num_bytes)
{
    for (unsigned i = 0; i < num_bytes; ++i) {
        message[i] = reflect4(message[i]);
    }
}

unsigned extract_nibbles_4b1s(uint8_t *message, unsigned offset_bits, unsigned num_bits, uint8_t *dst)
{
    unsigned ret = 0;

    while (num_bits >= 5) {
        uint16_t bits = (message[offset_bits / 8] << 8) | message[(offset_bits / 8) + 1];
        bits >>= 11 - (offset_bits % 8); // align 5 bits to LSB
        if ((bits & 1) != 1)
            break; // stuff-bit error
        *dst++ = (bits >> 1) & 0xf;
        ret += 1;
        offset_bits += 5;
        num_bits -= 5;
    }

    return ret;
}

unsigned extract_bytes_uart(uint8_t *message, unsigned offset_bits, unsigned num_bits, uint8_t *dst)
{
    unsigned ret = 0;

    while (num_bits >= 10) {
        int startb = message[offset_bits / 8] >> (7 - (offset_bits % 8));
        offset_bits += 1;
        int datab = message[offset_bits / 8];
        if (offset_bits % 8) {
            datab = (message[offset_bits / 8] << 8) | message[offset_bits / 8 + 1];
            datab >>= 8 - (offset_bits % 8);
        }
        offset_bits += 8;
        int stopb = message[offset_bits / 8] >> (7 - (offset_bits % 8));
        offset_bits += 1;
        if ((startb & 1) != 0)
            break; // start-bit error
        if ((stopb & 1) != 1)
            break; // stop-bit error
        *dst++ = reverse8(datab & 0xff);
        ret += 1;
        num_bits -= 10;
    }

    return ret;
}

static unsigned symbol_match(uint8_t *message, unsigned offset_bits, unsigned num_bits, uint32_t symbol)
{
    unsigned symbol_len = symbol & 0x1f;

    // check required len
    if (num_bits < symbol_len) {
        return 0;
    }

    // match each bit otherwise abort
    for (unsigned pos = 0; pos < symbol_len; ++pos) {
        unsigned m_pos = offset_bits + pos;
        unsigned m_bit = message[m_pos / 8] >> (7 - (m_pos % 8));
        unsigned s_bit = symbol >> (31 - pos);
        if ((m_bit & 1) != (s_bit & 1)) {
            return 0;
        }
    }

    return symbol_len;
}

unsigned extract_bits_symbols(uint8_t *message, unsigned offset_bits, unsigned num_bits, uint32_t zero, uint32_t one, uint32_t sync, uint8_t *dst)
{
    unsigned zero_len = zero & 0x1f;
    unsigned one_len  = one & 0x1f;
    unsigned sync_len = sync & 0x1f;

    unsigned dst_len = 0;

    while (num_bits >= 1) {
        // TODO: match the longest symbol first
        if (symbol_match(message, offset_bits, num_bits, sync)) {
            offset_bits += sync_len;
            num_bits -= sync_len;
            // just skip
        }
        else if (symbol_match(message, offset_bits, num_bits, zero)) {
            offset_bits += zero_len;
            num_bits -= zero_len;
            // no need to set a zero
            dst_len += 1;
        }
        else if (symbol_match(message, offset_bits, num_bits, one)) {
            offset_bits += one_len;
            num_bits -= one_len;
            dst[dst_len / 8] |= 0x80 >> (dst_len % 8);
            dst_len += 1;
        }
        else {
            break;
        }
    }

    // fprintf(stderr, "extract_bits_symbols: %x %x %x : %u (%u)\n", zero, one, sync, dst_len, num_bits);
    return dst_len;
}

uint8_t crc4(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    unsigned remainder = init << 4; // LSBs are unused
    unsigned poly = polynomial << 4;
    unsigned bit;

    while (nBytes--) {
        remainder ^= *message++;
        for (bit = 0; bit < 8; bit++) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ poly;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder >> 4 & 0x0f; // discard the LSBs
}

uint8_t crc7(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    unsigned remainder = init << 1; // LSB is unused
    unsigned poly = polynomial << 1;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ poly;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder >> 1 & 0x7f; // discard the LSB
}

uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    uint8_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ polynomial;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

uint8_t crc8le(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    uint8_t remainder = reverse8(init);
    unsigned byte, bit;
    polynomial = reverse8(polynomial);

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 1) {
                remainder = (remainder >> 1) ^ polynomial;
            } else {
                remainder = (remainder >> 1);
            }
        }
    }
    return remainder;
}

uint16_t crc16lsb(uint8_t const message[], unsigned nBytes, uint16_t polynomial, uint16_t init)
{
    uint16_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 1) {
                remainder = (remainder >> 1) ^ polynomial;
            }
            else {
                remainder = (remainder >> 1);
            }
        }
    }
    return remainder;
}

uint16_t crc16(uint8_t const message[], unsigned nBytes, uint16_t polynomial, uint16_t init)
{
    uint16_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte] << 8;
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x8000) {
                remainder = (remainder << 1) ^ polynomial;
            }
            else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

uint8_t lfsr_digest8(uint8_t const message[], unsigned bytes, uint8_t gen, uint8_t key)
{
    uint8_t sum = 0;
    for (unsigned k = 0; k < bytes; ++k) {
        uint8_t data = message[k];
        for (int i = 7; i >= 0; --i) {
            // fprintf(stderr, "key is %02x\n", key);
            // XOR key into sum if data bit is set
            if ((data >> i) & 1)
                sum ^= key;

            // roll the key right (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 1)
                key = (key >> 1) ^ gen;
            else
                key = (key >> 1);
        }
    }
    return sum;
}

uint8_t lfsr_digest8_reflect(uint8_t const message[], int bytes, uint8_t gen, uint8_t key)
{
    uint8_t sum = 0;
    // Process message from last byte to first byte (reflected)
    for (int k = bytes - 1; k >= 0; --k) {
        uint8_t data = message[k];
        // Process individual bits of each byte (reflected)
        for (int i = 0; i < 8; ++i) {
            // fprintf(stderr, "key is %02x\n", key);
            // XOR key into sum if data bit is set
            if ((data >> i) & 1) {
                sum ^= key;
            }

            // roll the key left (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 0x80)
                key = (key << 1) ^ gen;
            else
                key = (key << 1);
        }
    }
    return sum;
}

uint16_t lfsr_digest16(uint8_t const message[], unsigned bytes, uint16_t gen, uint16_t key)
{
    uint16_t sum = 0;
    for (unsigned k = 0; k < bytes; ++k) {
        uint8_t data = message[k];
        for (int i = 7; i >= 0; --i) {
            // fprintf(stderr, "key at bit %d : %04x\n", i, key);
            // if data bit is set then xor with key
            if ((data >> i) & 1)
                sum ^= key;

            // roll the key right (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 1)
                key = (key >> 1) ^ gen;
            else
                key = (key >> 1);
        }
    }
    return sum;
}

/*
void lfsr_keys_fwd16(int rounds, uint16_t gen, uint16_t key)
{
    for (int i = 0; i <= rounds; ++i) {
        fprintf(stderr, "key at bit %d : %04x\n", i, key);

        // roll the key right (actually the lsb is dropped here)
        // and apply the gen (needs to include the dropped lsb as msb)
        if (key & 1)
            key = (key >> 1) ^ gen;
        else
            key = (key >> 1);
    }
}

void lfsr_keys_rwd16(int rounds, uint16_t gen, uint16_t key)
{
    for (int i = 0; i <= rounds; ++i) {
        fprintf(stderr, "key at bit -%d : %04x\n", i, key);

        // roll the key left (actually the msb is dropped here)
        // and apply the gen (needs to include the dropped msb as lsb)
        if (key & (1 << 15))
            key = (key << 1) ^ gen;
        else
            key = (key << 1);
    }
}
*/

// we could use popcount intrinsic, but don't actually need the performance
int parity8(uint8_t byte)
{
    byte ^= byte >> 4;
    byte &= 0xf;
    return (0x6996 >> byte) & 1;
}

int parity_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result ^= parity8(message[i]);
    }
    return result;
}

uint8_t xor_bytes(uint8_t const message[], unsigned num_bytes)
{
    uint8_t result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result ^= message[i];
    }
    return result;
}

int add_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result += message[i];
    }
    return result;
}

int add_nibbles(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result += (message[i] >> 4) + (message[i] & 0x0f);
    }
    return result;
}

// Unit testing
#ifdef _TEST
#define ASSERT_EQUALS(a, b) \
    do { \
        if ((a) == (b)) \
            ++passed; \
        else { \
            ++failed; \
            fprintf(stderr, "FAIL: %d <> %d\n", (a), (b)); \
        } \
    } while (0)

int main(void) {
    unsigned passed = 0;
    unsigned failed = 0;

    fprintf(stderr, "util:: test\n");

    uint8_t msg[] = {0x08, 0x0a, 0xe8, 0x80};

    fprintf(stderr, "util::crc8(): odd parity\n");
    ASSERT_EQUALS(crc8(msg, 3, 0x80, 0x00), 0x80);

    fprintf(stderr, "util::crc8(): even parity\n");
    ASSERT_EQUALS(crc8(msg, 4, 0x80, 0x00), 0x00);

    // sync-word 0b0 0xff 0b1 0b0 0x33 0b1 (i.e. 0x7fd99, note that 0x33 is 0xcc "on the wire")
    uint8_t uart[]   = {0x7f, 0xd9, 0x90};
    uint8_t bytes[6] = {0};

    // y0 xff y1 y0 xcc y1 y0 x80 y1 y0 x40 y1 y0 xc0 y1
    uint8_t uart123[] = {0x07, 0xfd, 0x99, 0x40, 0x48, 0x16, 0x04, 0x00};

    fprintf(stderr, "util::extract_bytes_uart():\n");
    ASSERT_EQUALS(extract_bytes_uart(uart, 0, 24, bytes), 2);
    ASSERT_EQUALS(bytes[0], 0xff);
    ASSERT_EQUALS(bytes[1], 0x33);

    ASSERT_EQUALS(extract_bytes_uart(uart123, 4, 60, bytes), 5);
    ASSERT_EQUALS(bytes[0], 0xff);
    ASSERT_EQUALS(bytes[1], 0x33);
    ASSERT_EQUALS(bytes[2], 0x01);
    ASSERT_EQUALS(bytes[3], 0x02);
    ASSERT_EQUALS(bytes[4], 0x03);

    fprintf(stderr, "util:: test (%u/%u) passed, (%u) failed.\n", passed, passed + failed, failed);

    return failed;
}
#endif /* _TEST */



uint8_t hex_to_byte(char c1, char c2) {    uint8_t byte = 0;    if (c1 >= '0' && c1 <= '9') {        byte += (c1 - '0') << 4;    } else if (c1 >= 'A' && c1 <= 'F') {        byte += (c1 - 'A' + 10) << 4;    } else if (c1 >= 'a' && c1 <= 'f') {        byte += (c1 - 'a' + 10) << 4;    }    if (c2 >= '0' && c2 <= '9') {        byte += (c2 - '0');    } else if (c2 >= 'A' && c2 <= 'F') {        byte += (c2 - 'A' + 10);    } else if (c2 >= 'a' && c2 <= 'f') {        byte += (c2 - 'a' + 10);    }    return byte;}

uint8_t get_nibble(uint8_t *data, int idxNibble) {
  uint8_t result = data[idxNibble/2];
  if ((idxNibble & 1) == 0)
    result >>= 4;
  return result & (uint8_t)0xF;
}

/*
  CRC reverse engineering tools:
    * CRC RevEng - https://reveng.sourceforge.io/
	* delsum - https://github.com/8051Enthusiast/delsum
  CRC reverse engineering example with delsum:
    #!/bin/bash
    for c in 0 1 2 3 4 5 6 7 8 9 a b c d e f; do
      echo "0$c" | xxd -r -p > byte-${c}.bin
    done
    delsum reverse --extended-search -m 'crc width=16 init=0' -c 0000,045a,08b4,0cee,1168,1532 byte-8.bin byte-9.bin byte-a.bin byte-b.bin byte-c.bin byte-d.bin

  Calculating the CRC from the 3 temperature nibbles:

	Nibble #1
	  Note: nibble '8' gives zero by convention
	  Note: based on the known nibble values (0x8 to 0xd) we found two matching CRC-16 polynomials. They are fully equivalent for all 16 nibble values (both known and unknown).
	  crc width=16 poly=0x55c5 init=0x0 xorout=0x22d0 refin=true refout=true out_endian=big
	  crc width=16 poly=0xb62b init=0x0 xorout=0x22d0 refin=true refout=true out_endian=big

    Nibble #2
	  A look-up table for now.

    Nibble #3
      Possible CRC parameters are:
      crc width=16 poly=0x2381 init=0x0 xorout=0x6f46 refin=false refout=false out_endian=big
      crc width=16 poly=0x1ba7 init=0x0 xorout=0xe131 refin=false refout=false wordsize=16 out_endian=big
*/


const uint16_t lut[] = {
  0xaa17, 0x0046, 0xee94, 0x44c5, 0x2311, 0x8940, 0x6792, 0xcdc3,
  0xa83a, 0x026b, 0xecb9, 0x46e8, 0x213c, 0x8b6d, 0x65bf, 0xcfee,
};

uint16_t calc_checksum(uint8_t *theBytes, unsigned nibbleOffset) {
    uint8_t tmp = reverse8(get_nibble(theBytes, nibbleOffset++));
    uint16_t result = reverse32(crc16(&tmp, 1, 0x55c5, 0)) >> 16;
	result ^= lut[get_nibble(theBytes, nibbleOffset++)];
	tmp = get_nibble(theBytes, nibbleOffset++);
	return result ^ crc16(&tmp, 1, 0x1ba7, 0);
}

int main()
{
  FILE *fd = fopen("samples.txt", "rb");
  int nbrNibbles = 19;
  int nbrBytes = nbrNibbles/2;
  int maxLines = 10000;
  int nbrLines = 0;
  int nbrCrcMatches = 0;
  uint8_t allLines[maxLines][nbrBytes];
  while (1) {
    char hexValue[nbrNibbles+1]; // 1 extra byte for newline
    int tmp = fread(hexValue, 1, nbrNibbles+1, fd);
    if (tmp != nbrNibbles+1)
      break;
	if (nbrLines == maxLines) {
	  printf("Too many lines of data!");
	  return (-1);
	}
    int i;
    int k = 0;
    // note: start at 1 because we have 19 nibbles
    for (i=nbrNibbles-2*nbrBytes; i<nbrNibbles; i+=2) {
      allLines[nbrLines][k] = hex_to_byte(hexValue[i], hexValue[i+1]);
      k++;
	}
	nbrLines++;
  }
  fclose(fd);
  printf("Nbr samples from input file: %d\n", nbrLines);

  int idxChecksum = nbrBytes-2;
  int idxLine;
  for (idxLine=0; idxLine<nbrLines; idxLine++) {
    uint8_t *theBytes = &allLines[idxLine][0];
    int i;
	printf("7");
    for (i=0; i<nbrBytes; i++) {
      printf("%02x", theBytes[i]);
    }
    uint16_t result = calc_checksum(theBytes, 9);
	printf(" crc = %02x%02x", theBytes[idxChecksum], theBytes[idxChecksum+1]);
	printf("  computed = %04x", result);
	if (theBytes[idxChecksum] == (result >> 8) && theBytes[idxChecksum+1] == (result & 0xff)) {
	  printf("  OK\n");
	  nbrCrcMatches++;
	}
	else {
	  printf("  --\n");
	}
  }
  printf("Samples: %u, CRC matches: %u\n", nbrLines, nbrCrcMatches);
}

/*
  Encoding:
    7aaaaaa = Preamble (0x7aaaaaa)
           5c = Constant or ID? (0x5c)
             2 = Type (2=temperature, 7=resync)
              8a0 = Temperature (12 bits). Temperature in ºC = (value/4)-532
                 ff = Separator (0xff). Differs if resync
                   ce69 = Checksum (16 bits)

  Example readings (not double-checked, just for illustration):
    No probe = 5c2700ffb791 (CRC matches)
    Resync   = 5c7052f9cee3 (not handled, at least separator differs)
	Temperatures in ºC:
    20 = 5c28a0ffce69
    21 = 5c28a4ffa0f5
    22 = 5c28a8ff1351
    23 = 5c28acff7dcd
    24 = 5c28b0ff6438
    25 = 5c28b4ff0aa4
    44 = 5c2900ff8c9d
    47 = 5c290cff3f39
    48 = 5c2910ff26cc
    49 = 5c2914ff4850
    51 = 5c291cff9568
  ~ 53 = 5c2924ffa682
  ~ 55 = 5c292cff7bba
    58 = 5c2938ffbf77
  ~ 60 = 5c293cffd1eb
    62 = 5c2948ffd8a3
    67 = 5c295cff1c6e
    69 = 5c2964ff2f84
    71 = 5c296cfff2bc
  ~ 73 = 5c296cfff2bc
    74 = 5c2978ff3671
    77 = 5c2984ffe02c
  ~ 81 = 5c2994ff4a7d
  ~ 84 = 5c29a0ffca33
  ~ 89 = 5c29b4ff0efe
  ~105 = 5c29ecfff091
  ~110 = 5c2a08ff5d4b
  ~111 = 5c2a0cff33d7
  ~118 = 5c2a28ff19c8
  ~133 = 5c2a64ff236a
  ~138 = 5c2a78ff3a9f
*/
