/*-------------------------------------------------------------------------------
 * dsd_alias.c
 * Talker Alias Handling for Various Protocols and Vendors
 *
 * LWVMOBILE
 * 2025-02 DSD-FME Florida Man Edition
 *-----------------------------------------------------------------------------*/

#include "dsd.h"

//Motorola P25 OTA Alias Decoding ripped/demystified from Ilya Smirnov's SDRTrunk Voodoo Code
uint8_t moto_alias_lut[256] = { 
  0xD2, 0xF6, 0xD4, 0x2B, 0x63, 0x49, 0x94, 0x5E, 0xA7, 0x5C, 0x70, 0x69, 0xF7, 0x08, 0xB1, 0x7D,
  0x38, 0xCF, 0xCC, 0xD8, 0x51, 0x8F, 0xD5, 0x93, 0x6A, 0xF3, 0xEF, 0x7E, 0xFB, 0x64, 0xF4, 0x35,
  0x27, 0x07, 0x31, 0x14, 0x87, 0x98, 0x76, 0x34, 0xCA, 0x92, 0x33, 0x1B, 0x4F, 0x8C, 0x09, 0x40,
  0x32, 0x36, 0x77, 0x12, 0xD3, 0xC3, 0x01, 0xAB, 0x72, 0x81, 0x95, 0xC9, 0xC0, 0xE9, 0x65, 0x52,
  0x24, 0x30, 0x1C, 0xDB, 0x88, 0xE8, 0x97, 0x9D, 0x58, 0x26, 0x04, 0x39, 0xAC, 0x2A, 0x9E, 0xAA,
  0x25, 0xD7, 0xCE, 0xEB, 0x96, 0xF5, 0x0E, 0x8D, 0xDC, 0xA9, 0x2F, 0xDD, 0x1F, 0xEA, 0x91, 0xB7,
  0xD6, 0x89, 0x8B, 0xD1, 0xB0, 0x99, 0x13, 0x7A, 0xE7, 0x9A, 0xB5, 0x86, 0xFF, 0x46, 0x85, 0xB2,
  0x73, 0xDA, 0xBF, 0xD0, 0x71, 0xCB, 0x4D, 0x80, 0x15, 0x67, 0x16, 0x1A, 0x20, 0x8E, 0x45, 0x3E,
  0xF2, 0x2E, 0x66, 0x90, 0x74, 0x8A, 0x6F, 0x78, 0xBB, 0x53, 0x03, 0x11, 0x68, 0xCD, 0x44, 0x17,
  0x28, 0x5F, 0x1E, 0x84, 0x75, 0x79, 0x6E, 0x9B, 0x2C, 0xBE, 0x62, 0x2D, 0xF1, 0x7C, 0xB8, 0x83,
  0xD9, 0x4E, 0x6D, 0x02, 0x61, 0x3D, 0xA8, 0x06, 0xB9, 0xF8, 0x9C, 0x37, 0x3A, 0x23, 0xC1, 0x50,
  0xED, 0x9F, 0xAF, 0x3B, 0xBD, 0x82, 0xBA, 0xA0, 0xDF, 0xC2, 0x47, 0x22, 0xF0, 0xEE, 0xA1, 0xFE,
  0xA2, 0x10, 0x5B, 0x48, 0x57, 0xA3, 0x05, 0x60, 0x7B, 0x0D, 0xF9, 0x6C, 0xB3, 0x56, 0x4C, 0xBC,
  0x29, 0xA4, 0x0F, 0xEC, 0xB6, 0xA5, 0xA6, 0x3C, 0x7F, 0x6B, 0xB4, 0x21, 0xAD, 0xAE, 0xC4, 0xC8,
  0xC5, 0x5D, 0xDE, 0xE0, 0x1D, 0x19, 0x4B, 0xC6, 0x0C, 0x3F, 0x5A, 0xC7, 0xE1, 0x59, 0x55, 0x54,
  0x4A, 0x43, 0x42, 0xE2, 0xE3, 0xFA, 0x00, 0xE4, 0xE5, 0x18, 0x41, 0x0B, 0x0A, 0xE6, 0xFC, 0xFD
};

void apx_embedded_alias_test_phase1 (dsd_opts * opts, dsd_state * state)
{

  uint64_t temp_a = 0x15900FE9060100B0; uint64_t temp_b = 0x5A; //72 bits, so break into segments
  uint8_t lcw[72]; memset (lcw, 0, sizeof(lcw));
  for (uint64_t i = 0; i < 64; i++)
    lcw[i] = (temp_a >> (63-i)) & 1;
  for (uint64_t i = 0; i < 8; i++)
    lcw[i+64] = (temp_b >> (7-i)) & 1;
  p25_lcw(opts, state, lcw, 0);

  temp_a = 0x179001BBEE001C70; temp_b = 0x13;
  memset (lcw, 0, sizeof(lcw));
  for (uint64_t i = 0; i < 64; i++)
    lcw[i] = (temp_a >> (63-i)) & 1;
  for (uint64_t i = 0; i < 8; i++)
    lcw[i+64] = (temp_b >> (7-i)) & 1;
  p25_lcw(opts, state, lcw, 0);

  temp_a = 0x179002B9CB7D5F2D; temp_b = 0x48;
  memset (lcw, 0, sizeof(lcw));
  for (uint64_t i = 0; i < 64; i++)
    lcw[i] = (temp_a >> (63-i)) & 1;
  for (uint64_t i = 0; i < 8; i++)
    lcw[i+64] = (temp_b >> (7-i)) & 1;
  p25_lcw(opts, state, lcw, 0);

  temp_a = 0x179003B23695F7ED; temp_b = 0x49;
  memset (lcw, 0, sizeof(lcw));
  for (uint64_t i = 0; i < 64; i++)
    lcw[i] = (temp_a >> (63-i)) & 1;
  for (uint64_t i = 0; i < 8; i++)
    lcw[i+64] = (temp_b >> (7-i)) & 1;
  p25_lcw(opts, state, lcw, 0);

  temp_a = 0x179004B9EA998F87; temp_b = 0x48;
  memset (lcw, 0, sizeof(lcw));
  for (uint64_t i = 0; i < 64; i++)
    lcw[i] = (temp_a >> (63-i)) & 1;
  for (uint64_t i = 0; i < 8; i++)
    lcw[i+64] = (temp_b >> (7-i)) & 1;
  p25_lcw(opts, state, lcw, 0);

  temp_a = 0x179005BE6DAB167F; temp_b = 0xAC;
  memset (lcw, 0, sizeof(lcw));
  for (uint64_t i = 0; i < 64; i++)
    lcw[i] = (temp_a >> (63-i)) & 1;
  for (uint64_t i = 0; i < 8; i++)
    lcw[i+64] = (temp_b >> (7-i)) & 1;
  p25_lcw(opts, state, lcw, 0);

  temp_a = 0x179006B15EC2C622; temp_b = 0x2E;
  memset (lcw, 0, sizeof(lcw));
  for (uint64_t i = 0; i < 64; i++)
    lcw[i] = (temp_a >> (63-i)) & 1;
  for (uint64_t i = 0; i < 8; i++)
    lcw[i+64] = (temp_b >> (7-i)) & 1;
  p25_lcw(opts, state, lcw, 0);
}

void apx_embedded_alias_header_phase1 (dsd_opts * opts, dsd_state * state, uint8_t slot, uint8_t * lc_bits)
{

  UNUSED(opts);
  uint8_t ta_len = (uint8_t)ConvertBitIntoBytes(&lc_bits[32], 8); //len in blocks of associated talker alias
  uint8_t sn = (uint8_t)ConvertBitIntoBytes(&lc_bits[56], 4);
  fprintf (stderr, " SN: %X;", sn);
  fprintf (stderr, " BN: 0/%d;", ta_len);

  //use dmr_pdu_sf for storage, store entire header (will be used to verify complete reception of full alias)
  memset (state->dmr_pdu_sf[slot], 0, sizeof (state->dmr_pdu_sf[slot])); //reset storage for header and blocks
  memcpy (state->dmr_pdu_sf[slot], lc_bits, 72*sizeof(uint8_t));

}

void apx_embedded_alias_blocks_phase1 (dsd_opts * opts, dsd_state * state, uint8_t slot, uint8_t * lc_bits)
{

  UNUSED(opts);
  uint8_t bn = (uint8_t)ConvertBitIntoBytes(&lc_bits[16], 8); //current block number
  uint8_t sn = (uint8_t)ConvertBitIntoBytes(&lc_bits[24], 4); //is a static value on all block sequences
  uint8_t ta_len = (uint8_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][32], 8); //len in blocks pulled from stored header
  uint16_t header = (uint16_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][0], 16); //header check, should be 0x1590

  if (ta_len == 0 || header != 0x1590) //checkdown, make sure we have an up to date header for this with a good len value
  {
    fprintf (stderr, " Missing Header");
    fprintf (stderr, " BN: %d/??;", bn);
    fprintf (stderr, " SN: %X;", sn);
    fprintf (stderr, " Partial: ");
    for (uint8_t i = 7; i < 18; i++)
      fprintf (stderr, "%0X", (uint8_t)ConvertBitIntoBytes(&lc_bits[0+(i*4)], 4));

    //clear out now stale storage
    memset (state->dmr_pdu_sf[slot], 0, sizeof (state->dmr_pdu_sf[slot]));
  }

  else //good len and header stored
  {

    //sanity check, bn cannot equal zero (this shouldn't happen, but bad decode could occur)
    if (bn == 0) bn = 1;

    fprintf (stderr, " SN: %X;", sn);
    fprintf (stderr, " BN: %d/%d;", bn, ta_len);

    //use dmr_pdu_sf for storage, store data relevant portion at ptr of (bn-1) * 44 + 72 offset for header
    memcpy(state->dmr_pdu_sf[slot]+(((bn-1)*44)+72), lc_bits+28, 44*sizeof(uint8_t));

    if (ta_len == bn) //this is the last block, proceed to decoding
    {

      //WIP: Need a way to calculate variable len bits determined by a bit counter or something
      // int16_t num_bits = bn * 44; //number of relevant data bits total from all blocks (not including header)
      int16_t num_bits = 56; //starting value is the static bit count on the fqsuid

      // //evaluate the storage and determine how many octets/bits are present at this point (fall back to this if needed)
      // for (int16_t i = 0; i < 31; i++) //end point? BEE00 doesn't trip this since it is 0xBE 0xE0 0x0X, but other things might, may want to start after the fqsuid
      // {
      //   uint8_t byte = (uint8_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][72+56+(i*8)], 8);
      //   if (byte == 0)
      //     break;
      //   else
      //     num_bits += 8;
      // }

      //evaluate the storage and determine how many octets/bits are present at this point (expanded to two octets each, CRC with a 00xx pattern failed this)
      for (int16_t i = 0; i < 31; i++) //end point? BEE00 doesn't trip this since it is 0xBE 0xE0 0x0X, but other things might, may want to start after the fqsuid
      {
        uint16_t bytes = (uint16_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][72+56+(i*16)], 16);
        if (bytes == 0)
          break;
        else
          num_bits += 16;
      }

      //pass to alias decoder
      apx_embedded_alias_decode (opts, state, slot, num_bits, state->dmr_pdu_sf[slot]);
      
      //clear out now stale storage
      memset (state->dmr_pdu_sf[slot], 0, sizeof(state->dmr_pdu_sf[slot]));

    }
  }
}

void apx_embedded_alias_header_phase2 (dsd_opts * opts, dsd_state * state, uint8_t slot, uint8_t * lc_bits)
{
  
  UNUSED(opts);
  uint8_t ta_len = (uint8_t)ConvertBitIntoBytes(&lc_bits[40], 8);
  uint8_t sn = (uint8_t)ConvertBitIntoBytes(&lc_bits[64], 4);
  uint8_t bn = (uint8_t)ConvertBitIntoBytes(&lc_bits[56], 8);
  fprintf (stderr, " SN: %X;", sn); //NOTE: vPDU header is also a partial block, and has a block num and SN value in it
  fprintf (stderr, " BN: %d/%d;", bn, ta_len);

  //bit array to rearrange input lc_bits from phase 2 mac to match the phase 1 header and block handling
  uint8_t bits[136]; memset(bits, 0, sizeof(bits));
  memcpy (bits, lc_bits, 2*8*sizeof(uint8_t));         //header 0x9190
  memcpy (bits+16, lc_bits+24, 4*8*sizeof(uint8_t)); //BN, SN, etc
  memcpy (bits+56, lc_bits+56, 10*8*sizeof(uint8_t)); //adding 8 bits of extra padding here

  int16_t alias_st = 136; //start of the encoded alias (the copied size of this header, basically)

  //debug, dump header arranged at this end
  // fprintf (stderr, " Header: ");
  // for (int16_t i = 0; i < alias_st/8; i++) //double check and adjust
  //   fprintf (stderr, "%02X", (uint8_t)ConvertBitIntoBytes(&bits[0+(i*8)], 8));

  //use dmr_pdu_sf for storage, store entire header (will be used to verify complete reception of full alias)
  memset (state->dmr_pdu_sf[slot], 0, sizeof (state->dmr_pdu_sf[slot])); //reset storage for header and blocks
  memcpy (state->dmr_pdu_sf[slot], bits, alias_st*sizeof(uint8_t)); //this header block has 128 bits of relevant data (through the fqsuid)

}

void apx_embedded_alias_blocks_phase2 (dsd_opts * opts, dsd_state * state, uint8_t slot, uint8_t * lc_bits)
{
  //WIP: Adjust values as needed for MAC vPDU Messages
  UNUSED(opts);
  int16_t rel_bits = 100;  //number of relevant bits in each block
  int16_t rel_st   = 36;   //start of relevant bits in this block
  int16_t alias_st = 136;  //start of the encoded alias
  uint8_t bn = (uint8_t)ConvertBitIntoBytes(&lc_bits[24], 8); //current block number
  uint8_t sn = (uint8_t)ConvertBitIntoBytes(&lc_bits[32], 4); //is a static value on all block sequences
  uint8_t ta_len = (uint8_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][32], 8); //len in blocks pulled from stored header
  uint16_t header = (uint16_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][0], 16); //header check, should be 0x9190

  if (ta_len == 0 || header != 0x9190)
  {
    fprintf (stderr, " Missing Header");
    fprintf (stderr, " BN: %d/??;", bn);
    fprintf (stderr, " SN: %X;", sn);
    fprintf (stderr, " Partial: ");
    for (uint8_t i = 9; i < 32; i++) //double check and adjust
      fprintf (stderr, "%0X", (uint8_t)ConvertBitIntoBytes(&lc_bits[0+(i*4)], 4));

    //clear out now stale storage
    memset (state->dmr_pdu_sf[slot], 0, sizeof(state->dmr_pdu_sf[slot]));
  }

  else //good len and header stored
  {

    //sanity check, bn cannot equal zero (this shouldn't happen, but bad decode could occur)
    if (bn == 0) bn = 1;

    fprintf (stderr, " SN: %X;", sn);
    fprintf (stderr, " BN: %d/%d;", bn, ta_len);

    //use dmr_pdu_sf for storage, store data relevant portion at ptr calculated below
    memcpy(state->dmr_pdu_sf[slot]+(alias_st+((bn-1)*rel_bits)), lc_bits+rel_st, rel_bits*sizeof(uint8_t)); //Fix this value when samples arrive

    //debug, dump accumulated data at this end
    // fprintf (stderr, " Accumulated: ");
    // for (int16_t i = 0; i < (alias_st+(rel_bits*bn))/8; i++) //double check and adjust
    //   fprintf (stderr, "%02X", (uint8_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][0+(i*8)], 8));

    if (ta_len == bn) //this is the last block, proceed to decoding
    {

      //WIP: Need a way to calculate variable len bits determined by a bit counter or something
      // int16_t num_bits = (bn * rel_bits) + 56; //number of relevant data bits total from all blocks (including header fqsuid stub)
      int16_t num_bits = 56; //starting value is the static bit count on the fqsuid

      // //evaluate the storage and determine how many octets/bits are present at this point (fall back to this if needed)
      // for (int16_t i = 0; i < 31; i++) //end point? BEE00 doesn't trip this since it is 0xBE 0xE0 0x0X, but other things might, may want to start after the fqsuid
      // {
      //   uint8_t byte = (uint8_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][72+56+(i*8)], 8);
      //   if (byte == 0)
      //     break;
      //   else
      //     num_bits += 8;
      // }

      //evaluate the storage and determine how many octets/bits are present at this point (expanded to two octets each, CRC with a 00xx pattern failed this)
      for (int16_t i = 0; i < 31; i++) //end point? BEE00 doesn't trip this since it is 0xBE 0xE0 0x0X, but other things might, may want to start after the fqsuid
      {
        uint16_t bytes = (uint16_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][72+56+(i*16)], 16);
        if (bytes == 0)
          break;
        else
          num_bits += 16;
      }

      //pass to alias decoder
      apx_embedded_alias_decode (opts, state, slot, num_bits, state->dmr_pdu_sf[slot]);
      
      //clear out now stale storage
      memset (state->dmr_pdu_sf[slot], 0, sizeof (state->dmr_pdu_sf[slot]));

    }
  }
}

void apx_embedded_alias_decode (dsd_opts * opts, dsd_state * state, uint8_t slot, int16_t num_bits, uint8_t * input)
{

  UNUSED(opts);
  UNUSED(state);
  UNUSED(slot);

  //debug, dump completed data set
  // fprintf (stderr, "\n Full: ");
  // for (int16_t i = 0; i < (72+num_bits)/4; i++)
  //   fprintf (stderr, "%X", (uint8_t)ConvertBitIntoBytes(&input[0+(i*4)], 4));

  //extract CRC
  uint16_t crc_ext = (uint16_t)ConvertBitIntoBytes(&input[(72+num_bits-16)], 16);

  //compute CRC
  uint16_t crc_cmp = ComputeCrcCCITT16d(&input[72], num_bits-16);

  //print comparison
  // fprintf (stderr, " CRC EXT: %04X CMP: %04X;", crc_ext, crc_cmp);
  if (crc_ext != crc_cmp)
    fprintf (stderr, " Alias CRC Error;");
  // else fprintf (stderr, " Alias Okay;");

  //start decoding the alias
  if (crc_ext == crc_cmp)
  {

    //extract fully qualified SUID
    uint32_t wacn = (uint32_t)ConvertBitIntoBytes(&input[72], 20);
    uint32_t sys  = (uint32_t)ConvertBitIntoBytes(&input[92], 12);
    uint32_t rid  = (uint32_t)ConvertBitIntoBytes(&input[104], 24);

    //print fully qualified SUID
    fprintf (stderr, "\n FQ-SUID: %05X.%03X.%06X (%d);", wacn, sys, rid, rid);

    //WIP: Working, needs more samples to verify various num_bits values
    uint16_t ptr = 128; //starting point of encoded data from test vectors
    uint8_t encoded[200]; memset(encoded, 0, sizeof(encoded));
    uint8_t decoded[200]; memset(decoded, 0, sizeof(decoded));
    uint16_t num_bytes = (num_bits / 8) - 9; //subtract 2 CRC and 7 FQSUID

    //sanity check
    if (num_bytes <= 0) num_bytes = 1;

    for (int16_t i = 0; i < num_bytes; i++)
    {
      encoded[i] = (uint8_t)ConvertBitIntoBytes(&input[ptr], 8);
      ptr += 8;
    }

    uint16_t accumulator = num_bytes;

    //Ilya's Voodoo Code
    for (uint16_t i = 0; i < num_bytes; i++)
    {
      // Multiplication step 1
      uint16_t accum_mult = accumulator * 293 + 0x72E9;

      // Lookup table step
      uint8_t lut = moto_alias_lut[encoded[i]];
      uint8_t mult1 = lut - (accum_mult >> 8);

      // Incrementing step
      uint8_t mult2 = 1;
      uint8_t shortstop = accum_mult | 0x1;
      uint8_t increment = shortstop << 1;
      
      while(mult2 != -1 && shortstop != 1)
      {
        shortstop += increment;
        mult2 += 2;
      }

      // Multiplication step 2
      decoded[i] = mult1 * mult2;

      // Update the accumulator
      accumulator += encoded[i] + 1;

    }

    // //debug, dump just the encoded alias portion
    // fprintf (stderr, "\n Encoded: ");
    // for (int16_t i = 0; i < num_bytes; i++)
    //   fprintf (stderr, "%02X", encoded[i]);

    // //debug, dump the decoded payload as hex octets
    // fprintf (stderr, "\n Decoded: ");
    // for (int16_t i = 0; i < num_bytes; i++)
    //   fprintf (stderr, "%02X", decoded[i]);

    //dump the decoded payload as long chars
    // fprintf (stderr, "\n");
    fprintf (stderr, " Alias: ");
    for (int16_t i = 0; i < num_bytes/2; i++)
      fprintf (stderr, "%lc", ((decoded[(i*2)+0])<<8) | ((decoded[(i*2)+1])<<0) );

    //For Ncurses Display (needs to be testing on real samples first to make sure this is setup right, %s doesn't seem to work well with utf16)
    // memcpy (state->dmr_embedded_gps[slot], decoded, num_bytes*sizeof(uint8_t));
    // state->dmr_embedded_gps[slot][199] = '\0'; //terminate string

    //For Ncurses Display (kind of a hack, but will only work if the UTF16 chars are just roman alphanumerical UTF8 compatible)
    for (int16_t i = 0; i < num_bytes/2; i++)
      state->dmr_embedded_gps[slot][i] = decoded[(i*2)+1];

    //TODO: Do this like the Harris Talker Alias so it 
    //can read/write to the files and not randomly reset 
    //and flicker each time its decoded
  }

}

//end Motorola P25 OTA Alias Decoding

//TODO: Migrate Other OTA Alias functions here