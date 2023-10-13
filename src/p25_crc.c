/*-------------------------------------------------------------------------------
 * p25_crc.c
 * P25 Misc CRC checks
 *
 * LWVMOBILE
 * 2022-09 DSD-FME Florida Man Edition
 *-----------------------------------------------------------------------------*/

#include "dsd.h"

//modified from the LEH ComputeCrcCCITT to accept variable len buffer bits
uint16_t ComputeCrcCCITT16b(const uint8_t buf[], unsigned int len)
{
  uint32_t i;
  uint16_t CRC = 0x0000;
  uint16_t Polynome = 0x1021;
  for(i = 0; i < len; i++)
  {
    if(((CRC >> 15) & 1) ^ (buf[i] & 1))
    {
      CRC = (CRC << 1) ^ Polynome;
    }
    else
    {
      CRC <<= 1;
    }
  }

  /* Invert the CRC */
  CRC ^= 0xFFFF;

  /* Return the CRC */
  return CRC;
} /* End ComputeCrcCCITT() */

//modified from crc12_ok to run a quickie on 16 instead
static uint16_t crc16_ok(const uint8_t bits[], unsigned int len) {
	uint16_t crc = 0;
  // fprintf (stderr, "\n  LEN = %d", len);
  for (int i = 0; i < 16; i++)
  {
    crc = crc << 1;
    crc = crc | bits[i+len];
  }
  uint16_t check = ComputeCrcCCITT16b(bits, len);

  if (crc == check)
  {
    //fprintf (stderr, " CRC = %04X %04X", crc, check);
    return (0);
  }
  else
  {
    //fprintf (stderr, " CRC = %04X %04X", crc, check);
    return (-1);
  }

}

//TSBK/LCCH CRC16 x-bit bridge to crc16b and crc16b_okay, wip, may need multi pdu format as well
int crc16_lb_bridge (int payload[190], int len)
{
  int err = -2;
  uint8_t buf[190] = {0};

  for (int i = 0; i < len+16; i++) //add +16 here so we load the entire frame but only run crc on the len portion
  {
    buf[i] = payload[i];
  }
  err = crc16_ok(buf, len);
  return (err);
}

//borrowing crc12 from OP25
static uint16_t crc12(const uint8_t bits[], unsigned int len) {
	uint16_t crc=0;
	static const unsigned int K = 12;
  //g12(x) = x12 + x11 + x7 + x4 + x2 + x + 1
	static const uint8_t poly[13] = {1,1,0,0,0,1,0,0,1,0,1,1,1}; // p25 p2 crc 12 poly
	uint8_t buf[256];
	if (len+K > sizeof(buf)) {
		//fprintf (stderr, "crc12: buffer length %u exceeds maximum %u\n", len+K, sizeof(buf));
		return 0;
	}
	memset (buf, 0, sizeof(buf));
	for (unsigned int i=0; i<len; i++){
		buf[i] = bits[i];
	}
	for (unsigned int i=0; i<len; i++)
		if (buf[i])
			for (unsigned int j=0; j<K+1; j++)
				buf[i+j] ^= poly[j];
	for (unsigned int i=0; i<K; i++){
		crc = (crc << 1) + buf[len + i];
	}
	return crc ^ 0xfff;
}

//borrowing crc12_ok from OP25
static uint16_t crc12_ok(const uint8_t bits[], unsigned int len) {
	uint16_t crc = 0;
  for (int i = 0; i < 12; i++)
  {
    crc = crc << 1;
    crc = crc | bits[i+len];
  }
  uint16_t check = crc12(bits,len);

  if (crc == check)
  {
    //fprintf (stderr, " CRC = %03X %03X", crc, check);
    return (0);
  }
  else
  {
    //fprintf (stderr, " CRC = %03X %03X", crc, check);
    return (-1);
  }

}

//xCCH CRC12 x-bit bridge to crc12b and crc12b_okay
int crc12_xb_bridge (int payload[190], int len)
{
  int err = -5;
  uint8_t buf[190] = {0};

  //need to load up the 12 crc bits as well, else it will fail on the check, but not on the crc calc
  for (int i = 0; i < len+12; i++)
  {
    buf[i] = payload[i];
  }
  err = crc12_ok(buf, len);
  return (err);
}

uint8_t lsd_parity[256] = {
0x00, 0x39, 0x72, 0x4B, 0xE4, 0xDD, 0x96, 0xAF, 0xF1, 0xC8, 0x83, 0xBA, 0x15, 0x2C, 0x67, 0x5E,
0xDB, 0xE2, 0xA9, 0x90, 0x3F, 0x06, 0x4D, 0x74, 0x2A, 0x13, 0x58, 0x61, 0xCE, 0xF7, 0xBC, 0x85,
0x8F, 0xB6, 0xFD, 0xC4, 0x6B, 0x52, 0x19, 0x20, 0x7E, 0x47, 0x0C, 0x35, 0x9A, 0xA3, 0xE8, 0xD1,
0x54, 0x6D, 0x26, 0x1F, 0xB0, 0x89, 0xC2, 0xFB, 0xA5, 0x9C, 0xD7, 0xEE, 0x41, 0x78, 0x33, 0x0A,
0x27, 0x1E, 0x55, 0x6C, 0xC3, 0xFA, 0xB1, 0x88, 0xD6, 0xEF, 0xA4, 0x9D, 0x32, 0x0B, 0x40, 0x79,
0xFC, 0xC5, 0x8E, 0xB7, 0x18, 0x21, 0x6A, 0x53, 0x0D, 0x34, 0x7F, 0x46, 0xE9, 0xD0, 0x9B, 0xA2,
0xA8, 0x91, 0xDA, 0xE3, 0x4C, 0x75, 0x3E, 0x07, 0x59, 0x60, 0x2B, 0x12, 0xBD, 0x84, 0xCF, 0xF6,
0x73, 0x4A, 0x01, 0x38, 0x97, 0xAE, 0xE5, 0xDC, 0x82, 0xBB, 0xF0, 0xC9, 0x66, 0x5F, 0x14, 0x2D,
0x4E, 0x77, 0x3C, 0x05, 0xAA, 0x93, 0xD8, 0xE1, 0xBF, 0x86, 0xCD, 0xF4, 0x5B, 0x62, 0x29, 0x10,
0x95, 0xAC, 0xE7, 0xDE, 0x71, 0x48, 0x03, 0x3A, 0x64, 0x5D, 0x16, 0x2F, 0x80, 0xB9, 0xF2, 0xCB,
0xC1, 0xF8, 0xB3, 0x8A, 0x25, 0x1C, 0x57, 0x6E, 0x30, 0x09, 0x42, 0x7B, 0xD4, 0xED, 0xA6, 0x9F,
0x1A, 0x23, 0x68, 0x51, 0xFE, 0xC7, 0x8C, 0xB5, 0xEB, 0xD2, 0x99, 0xA0, 0x0F, 0x36, 0x7D, 0x44,
0x69, 0x50, 0x1B, 0x22, 0x8D, 0xB4, 0xFF, 0xC6, 0x98, 0xA1, 0xEA, 0xD3, 0x7C, 0x45, 0x0E, 0x37,
0xB2, 0x8B, 0xC0, 0xF9, 0x56, 0x6F, 0x24, 0x1D, 0x43, 0x7A, 0x31, 0x08, 0xA7, 0x9E, 0xD5, 0xEC,
0xE6, 0xDF, 0x94, 0xAD, 0x02, 0x3B, 0x70, 0x49, 0x17, 0x2E, 0x65, 0x5C, 0xF3, 0xCA, 0x81, 0xB8,
0x3D, 0x04, 0x4F, 0x76, 0xD9, 0xE0, 0xAB, 0x92, 0xCC, 0xF5, 0xBE, 0x87, 0x28, 0x11, 0x5A, 0x63};

int p25p1_lsd_fec(uint8_t * input)
{
  uint8_t lsd = (uint8_t)ConvertBitIntoBytes(&input[0], 8);
  uint8_t par = (uint8_t)ConvertBitIntoBytes(&input[8], 8);
  uint8_t chk = lsd_parity[lsd];
  //NOTE: Simply checking the parity doesn't account for the parity bits being bad, but the 
  //low speed data bits being correct, but is better than nothing
  if (chk == par) return 1;
  else
  {
    //TODO: some FEC functions
    return 0;
  }
}