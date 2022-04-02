#include "dsd.h"
#include "nxdn_const.h"

//don't know why this doesn't work right in LEH, need to figure that out for him
//works just fien when ripped from his and into mine
//must be some random thing, I'll look at his when I have a chance
void processNXDNVoice (dsd_opts * opts, dsd_state * state)
{
  int i, j, dibit;
  char ambe_fr[4][24];
  unsigned char sacch_raw[72] = {0};
  unsigned char sacch_decoded[32] = {0}; /* 26 bit + 6 bit CRC */
  const int *w, *x, *y, *z;
  unsigned char *pr;
  uint8_t CrcIsGood = 0;
  uint8_t StructureField = 0;
  uint8_t RAN = 0;
  uint8_t PartOfFrame = 0;
  uint8_t KeyStream[1664] = {0};
  char ambe7bytesArray[7] = {0};
  int PartOfEncryptedSuperFrame = 0;

  if (opts->errorbars == 1)
  {
    fprintf(stderr, "VOICE - ");
  }

  /* Start pseudo-random NXDN sequence after
   * LITCH = 16 bit = 8 dibit
   * ==> Index 8 */
  pr = (unsigned char *)(&nxdnpr2[8]);
  for (i = 0; i < 30; i++)
  {
    dibit = getDibit (opts, state);

    /* Store and de-interleave directly SACCH bits */
    sacch_raw[(2*i)]   = ((*pr) & 1) ^ (1 & (dibit >> 1));  // bit 1
    sacch_raw[(2*i)+1] = (1 & dibit);               // bit 0

// TODO : Deinterleave is done in the "NXDN_SACCH_decode" function
//    /* Store and de-interleave directly SACCH bits */
//    sacch_raw[nsW[2*i]]   = *pr ^ (1 & (dibit >> 1));  // bit 1
//    sacch_raw[nsW[2*i+1]] = (1 & dibit);               // bit 0
    pr++;
#ifdef NXDN_DUMP
    fprintf(stderr, "%c", dibit + 48);
#endif
  }
#ifdef NXDN_DUMP
  fprintf(stderr, " ");
#endif

// TODO : Unpuncture is done in the "NXDN_SACCH_decode" function
//  /* For the sake of simplicity, handle the re-insertion of punctured bits here.
//   * This makes the actual Viterbi decoder code a LOT simpler. */
//  for (i = 30; i < 36; i++)
//  {
//    sacch_raw[nsW[2*i]]   = 0;
//    sacch_raw[nsW[2*i+1]] = 0;
//  }

  /* Decode the SACCH fields */
  CrcIsGood = NXDN_SACCH_raw_part_decode(sacch_raw, sacch_decoded);
  StructureField = (sacch_decoded[0]<<1 | sacch_decoded[1]<<0) & 0x03; /* Compute the Structure Field (remove 2 first bits of the SR Information in the SACCH) */
  RAN = (sacch_decoded[2]<<5 | sacch_decoded[3]<<4 | sacch_decoded[4]<<3 | sacch_decoded[5]<<2 | sacch_decoded[6]<<1 | sacch_decoded[7]<<0) & 0x3F; /* Compute the RAN (6 last bits of the SR Information in the SACCH) */

  /* Compute the Part of Frame according to the structure field */
  if     (StructureField == 3) PartOfFrame = 0;
  else if(StructureField == 2) PartOfFrame = 1;
  else if(StructureField == 1) PartOfFrame = 2;
  else if(StructureField == 0) PartOfFrame = 3;
  else PartOfFrame = 0; /* Normally we should never go here */

  /* Fill the SACCH structure */
  state->NxdnSacchRawPart[PartOfFrame].CrcIsGood = CrcIsGood;
  state->NxdnSacchRawPart[PartOfFrame].StructureField = StructureField;
  state->NxdnSacchRawPart[PartOfFrame].RAN = RAN;
  memcpy(state->NxdnSacchRawPart[PartOfFrame].Data, &sacch_decoded[8], 18); /* Copy the 18 bits of SACCH content */

  //fprintf(stderr, "RAN=%02d - Part %d/4 ", RAN, PartOfFrame + 1);
  //if(CrcIsGood) fprintf(stderr, "   (OK)   - ");
  if(CrcIsGood)
  {
    fprintf (stderr, "RAN=%02d - Part %d/4 ", RAN, PartOfFrame + 1);
    fprintf (stderr, "   (OK)   - ");
    state->nxdn_last_ran = RAN; //disable, try to grab this in voice instead
  }
  else fprintf(stderr, "(CRC ERR) - ");

  /* Generate the key stream */
  NxdnEncryptionStreamGeneration(opts, state, KeyStream);

  //fprintf(stderr, "\nKeyStream = ");
  //for(i = 0; i < 49; i++) fprintf(stderr, "%d", KeyStream[i]);
  //fprintf(stderr, "\n");


  /* Determine the current part of superframe
   * (in an AES or DES encrypted frame, two superframes
   * are used with the same IV, so to decipher the sound
   * correctly we must know the current superframe part) */
  if(state->NxdnElementsContent.PartOfNextEncryptedFrame == 1) PartOfEncryptedSuperFrame = 0;
  else if(state->NxdnElementsContent.PartOfNextEncryptedFrame == 2) PartOfEncryptedSuperFrame = 1;
  else PartOfEncryptedSuperFrame = 0;

  /* Decode the SACCH only when all 4 voice frame
   * SACCH spare parts have been received */
  if(PartOfFrame == 3)
  {
    /* Decode the entire SACCH content */
    NXDN_SACCH_Full_decode(opts, state);
  }


  if (opts->errorbars == 1)
  {
    fprintf(stderr, "e:");
  }

  /* Start pseudo-random NXDN sequence after
   * LITCH = 16 bit = 8 dibit +
   * SACCH = 60 bit = 30 dibit
   * = 76 bit = 38 dibit
   * ==> Index 38 */
  pr = (unsigned char *)(&nxdnpr2[38]);
  for (j = 0; j < 4; j++)
  {
    w = nW;
    x = nX;
    y = nY;
    z = nZ;
    for (i = 0; i < 36; i++)
    {
      dibit = getDibit (opts, state);
#ifdef NXDN_DUMP
      fprintf(stderr, "%c", dibit + 48);
#endif
      ambe_fr[*w][*x] = *pr ^ (1 & (dibit >> 1)); // bit 1
      pr++;
      ambe_fr[*y][*z] = (1 & dibit);              // bit 0

      w++;
      x++;
      y++;
      z++;
    }
    //processMbeFrame (opts, state, NULL, ambe_fr, NULL);
    processMbeFrameEncrypted(opts, state, NULL, ambe_fr, NULL, (char *)&KeyStream[(PartOfEncryptedSuperFrame * 4 * 4 * 49) + (PartOfFrame * 4 * 49) + (j * 49)], NULL);


#ifdef BUILD_DSD_WITH_FRAME_CONTENT_DISPLAY
    if(state->printNXDNAmbeVoiceSampleHex)
    {
      /* Display AMBE frame content */
      /* Convert the 49 bit AMBE frame into 7 bytes */
      Convert49BitSampleInto7Bytes(state->ambe_deciphered, ambe7bytesArray);
      fprintf(stderr, "\nVoice Frame %d/4 : ", j + 1);
      fprintf(stderr, "E1 = %d ; E2 = %d ; Content = ", state->errs, state->errs2);
      for(i = 0; i < 7; i++) fprintf(stderr, "0x%02X, ", ambe7bytesArray[i] & 0xFF);
      //fprintf(stderr, "\n");
    }
#endif /* BUILD_DSD_WITH_FRAME_CONTENT_DISPLAY */

#ifdef NXDN_DUMP
    fprintf(stderr, " ");
#endif
  } /* End for (j = 0; j < 4; j++) */

  if (opts->errorbars == 1)
  {
    fprintf(stderr, "\n");
  }

  /* Reset the SACCH CRCs when all 4 voice frame
   * SACCH spare parts have been received */
  if(PartOfFrame == 3)
  {
    /* Reset all CRCs of the SACCH */
    for(i = 0; i < 4; i++) state->NxdnSacchRawPart[i].CrcIsGood = 0;
  }
} /* End processNXDNVoice() */
