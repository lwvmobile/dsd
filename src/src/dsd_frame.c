/*
 * Copyright (C) 2010 DSD Author
 * GPG Key ID: 0x3F1D7FD0 (74EF 430D F7F2 0A48 FCE6  F630 FAA2 635D 3F1D 7FD0)
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#include "dsd.h"
#if !defined(NULL)
#define NULL 0
#endif

#include "p25p1_check_nid.h"

void
printFrameInfo (dsd_opts * opts, dsd_state * state)
{

  int level;

  level = (int) state->max / 164;
  if (opts->verbose > 0)
    {
      //fprintf (stderr,"inlvl: %2i%% ", level);
    }
  if (state->nac != 0)
    {
      fprintf (stderr, "%s", KCYN);
      fprintf (stderr,"nac: [%4X] ", state->nac);
      fprintf (stderr, "%s", KNRM);
    }

  if (opts->verbose > 1)
    {
      fprintf (stderr, "%s", KGRN);
      fprintf (stderr,"src: [%8i] ", state->lastsrc);
      fprintf (stderr, "%s", KNRM);
    }
  fprintf (stderr, "%s", KGRN);
  fprintf (stderr,"tg: [%5i] ", state->lasttg);
  fprintf (stderr, "%s", KNRM);
}

void
processFrame (dsd_opts * opts, dsd_state * state)
{

  int i, j, dibit;
  char duid[3];
  char nac[13];
  int level;

  char status_0;
  char bch_code[63];
  int index_bch_code;
  unsigned char parity;
  char v;
  int new_nac;
  char new_duid[3];
  int check_result;

  nac[12] = 0;
  duid[2] = 0;
  j = 0;

  if (state->rf_mod == 1)
    {
      state->maxref = (int)(state->max * 0.80F);
      state->minref = (int)(state->min * 0.80F);
    }
  else
    {
      state->maxref = state->max;
      state->minref = state->min;
    }

  if ((state->synctype == 8) || (state->synctype == 9)) //NXDN Voice
    {
      //state->rf_mod = 2; //wrong type of modulation HERE HERE
      state->nac = 0;
      state->lastsrc = 0;
      state->lasttg = 0;
      if (opts->errorbars == 1)
        {
          if (opts->verbose > 0)
            {
              level = (int) state->max / 164;
              //fprintf (stderr, "inlvl: %2i%% ", level);
            }
        }
      state->nac = 0;
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
        {
          openMbeOutFile (opts, state);
        }
      sprintf (state->fsubtype, " VOICE        ");
      processNXDNVoice (opts, state);
      //ProcessNXDNFrame (opts, state, 0);
      return;
    }
  else if ((state->synctype == 16) || (state->synctype == 17)) //NXDN Data
    {
      //state->rf_mod = 2;
      state->nac = 0;
      state->lastsrc = 0;
      state->lasttg = 0;
      if (opts->errorbars == 1)
        {
          if (opts->verbose > 0)
            {
              level = (int) state->max / 164;
              //fprintf (stderr, "inlvl: %2i%% ", level);
            }
        }
      state->nac = 0;
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
        {
          openMbeOutFile (opts, state);
        }
      sprintf (state->fsubtype, " DATA         ");
      processNXDNData (opts, state);
      //ProcessNXDNFrame (opts, state, 0);
      return;
    }
  else if ((state->synctype == 6) || (state->synctype == 7))
    {
      state->nac = 0;
      state->lastsrc = 0;
      state->lasttg = 0;
      if (opts->errorbars == 1)
        {
          if (opts->verbose > 0)
            {
              level = (int) state->max / 164;
              //fprintf (stderr,"inlvl: %2i%% ", level);
            }
        }
      state->nac = 0;
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
        {
          openMbeOutFile (opts, state);
        }
      sprintf (state->fsubtype, " VOICE        ");
      processDSTAR (opts, state);
      return;
    }
  else if ((state->synctype == 18) || (state->synctype == 19))
    {
      state->nac = 0;
      state->lastsrc = 0;
      state->lasttg = 0;
      if (opts->errorbars == 1)
        {
          if (opts->verbose > 0)
            {
              level = (int) state->max / 164;
              //fprintf (stderr,"inlvl: %2i%% ", level);
            }
        }
      state->nac = 0;
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
        {
          openMbeOutFile (opts, state);
        }
      sprintf (state->fsubtype, " DATA         ");
      processDSTAR_HD (opts, state);
      return;
    }
    //Start DMR Types
    else if ((state->synctype >= 10) && (state->synctype <= 13) || (state->synctype == 32) || (state->synctype == 33) || (state->synctype == 34) ) //32-34 DMR MS and RC
    {
      //disable so radio id doesn't blink in and out during ncurses and aggressive_framesync
      state->nac = 0;
      //state->lastsrc = 0;
      //state->lasttg = 0;
      //state->lastsrcR = 0;
      //state->lasttgR = 0;
      if (opts->errorbars == 1)
        {
          if (opts->verbose > 0)
            {
              level = (int) state->max / 164;
              //fprintf (stderr,"inlvl: %2i%% ", level);
            }
        }
      if ( (state->synctype == 11) || (state->synctype == 12) || (state->synctype == 32) ) //DMR Voice Modes
        {
          if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL) && opts->dmr_stereo == 0)
            {
              openMbeOutFile (opts, state);
            }
          sprintf (state->fsubtype, " VOICE        ");
          if (opts->dmr_stereo == 0 && state->synctype < 32) // -T option for DMR (TDMA) stereo
          {
            sprintf (state->slot1light, " slot1 ");
            sprintf (state->slot2light, " slot2 ");
            processDMRvoice (opts, state);
          }
          if (opts->dmr_stereo == 0 && state->synctype == 32)
          {
            fprintf (stderr, "%s", KRED);
            fprintf (stderr, "Please use the -T option to handle MS Mode with DMR Stereo TDMA Handling\n");
            fprintf (stderr, "%s", KNRM);
          }
          if (opts->dmr_stereo == 1)
          {
            state->dmr_stereo = 1; //set the state to 1 when handling pure voice frames
            if (state->synctype > 31 )
            {
              dmrMSBootstrap (opts, state); //bootstrap into MS Bootstrap (voice only)
            }
            else dmrBSBootstrap (opts, state); //bootstrap into BS Bootstrap
          }
        }
      else if ( (state->synctype == 33) || (state->synctype == 34) ) //MS Data and RC data
      {
        if (opts->dmr_stereo == 0)
        {
          closeMbeOutFile (opts, state);
          state->err_str[0] = 0;
          fprintf (stderr, "%s", KRED);
          fprintf (stderr, "Please use the -T option to handle MS Mode with DMR Stereo TDMA Handling\n");
          fprintf (stderr, "%s", KNRM);
        }
        if (opts->dmr_stereo == 1)
        {
          dmrMSData (opts, state);

        }
      }
      else
      {
        if (opts->dmr_stereo == 0)
        {
          closeMbeOutFile (opts, state);
          state->err_str[0] = 0;
          sprintf (state->slot1light, " slot1 ");
          sprintf (state->slot2light, " slot2 ");
          state->payload_miP = 0; //zero out when switching back to data sync
          processDMRdata (opts, state);
        }
        //switch dmr_stereo to 0 when handling BS data frame syncs with processDMRdata
        if (opts->dmr_stereo == 1)
        {
          state->dmr_stereo = 0; //set the state to zero for handling pure data frames
          sprintf (state->slot1light, " slot1 ");
          sprintf (state->slot2light, " slot2 ");
          state->payload_miP = 0; //zero out when switching back to data sync
          processDMRdata (opts, state);
        }
      }
      return;
    }
  else if ((state->synctype >= 2) && (state->synctype <= 5))
    {
      state->nac = 0;
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
        }
      if ((state->synctype == 3) || (state->synctype == 4))
        {
          if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
            {
              openMbeOutFile (opts, state);
            }
          sprintf (state->fsubtype, " VOICE        ");
          processX2TDMAvoice (opts, state);
        }
      else
        {
          closeMbeOutFile (opts, state);
          state->err_str[0] = 0;
          processX2TDMAdata (opts, state);
        }
      return;
    }
  else if ((state->synctype == 14) || (state->synctype == 15))
    {
      state->nac = 0;
      state->lastsrc = 0;
      state->lasttg = 0;
      if (opts->errorbars == 1)
        {
          if (opts->verbose > 0)
            {
              level = (int) state->max / 164;
              //fprintf (stderr,"inlvl: %2i%% ", level);
            }
        }
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
        {
          openMbeOutFile (opts, state);
        }
      sprintf (state->fsubtype, " VOICE        ");
      processProVoice (opts, state);
      return;
    }
    //ysf
    else if ((state->synctype == 30) || (state->synctype == 31))
    {
      //Do stuff
      //fprintf(stderr, "YSF Sync! \n");
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
      {
        //openMbeOutFile (opts, state);
      }
      //sprintf(state->fsubtype, " VOICE        ");
      processYSF(opts, state);
      return;
    }
    //P25 P2
    else if ((state->synctype == 35) || (state->synctype == 36))
    {
      //Do stuff
      //fprintf(stderr, "YSF Sync! \n");
      if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
      {
        //openMbeOutFile (opts, state);
      }
      //sprintf(state->fsubtype, " VOICE        ");
      processP2(opts, state);
      return;
    }
    //dPMR
    else if ((state->synctype == 20) || (state->synctype == 24))
    {
      /* dPMR Frame Sync 1 */
      fprintf(stderr, "dPMR Frame Sync 1 ");
    }
    else if ((state->synctype == 21) || (state->synctype == 25))
    {
      /* dPMR Frame Sync 2 */
        fprintf(stderr, "dPMR Frame Sync 2 ");

        state->nac = 0;
        state->lastsrc = 0;
        state->lasttg = 0;
        if (opts->errorbars == 1)
        {
          if (opts->verbose > 0)
          {
            level = (int) state->max / 164;
          }
        }
        state->nac = 0;

          if ((opts->mbe_out_dir[0] != 0) && (opts->mbe_out_f == NULL))
          {
            openMbeOutFile (opts, state);
          }
          sprintf(state->fsubtype, " VOICE        ");
          processdPMRvoice (opts, state);

        return;

    }
    else if ((state->synctype == 22) || (state->synctype == 26))
    {
      /* dPMR Frame Sync 3 */
      fprintf(stderr, "dPMR Frame Sync 3 ");
    }
    else if ((state->synctype == 23) || (state->synctype == 27))
    {
      /* dPMR Frame Sync 4 */
      fprintf(stderr, "dPMR Frame Sync 4 ");
    }
    //dPMR
  else //P25
    {
      // Read the NAC, 12 bits
      j = 0;
      index_bch_code = 0;
      for (i = 0; i < 6; i++)
        {
          dibit = getDibit (opts, state);

          v = 1 & (dibit >> 1); // bit 1
          nac[j] = v + '0';
          j++;
          bch_code[index_bch_code] = v;
          index_bch_code++;

          v = 1 & dibit;        // bit 0
          nac[j] = v + '0';
          j++;
          bch_code[index_bch_code] = v;
          index_bch_code++;
        }
      state->nac = strtol (nac, NULL, 2);

      // Read the DUID, 4 bits
      for (i = 0; i < 2; i++)
        {
          dibit = getDibit (opts, state);
          duid[i] = dibit + '0';

          bch_code[index_bch_code] = 1 & (dibit >> 1);  // bit 1
          index_bch_code++;
          bch_code[index_bch_code] = 1 & dibit;         // bit 0
          index_bch_code++;
        }

      // Read the BCH data for error correction of NAC and DUID
      for (i = 0; i < 3; i++)
        {
          dibit = getDibit (opts, state);

          bch_code[index_bch_code] = 1 & (dibit >> 1);  // bit 1
          index_bch_code++;
          bch_code[index_bch_code] = 1 & dibit;         // bit 0
          index_bch_code++;
        }
      // Intermission: read the status dibit
      status_0 = getDibit (opts, state) + '0';
      // ... continue reading the BCH error correction data
      for (i = 0; i < 20; i++)
        {
          dibit = getDibit (opts, state);

          bch_code[index_bch_code] = 1 & (dibit >> 1);  // bit 1
          index_bch_code++;
          bch_code[index_bch_code] = 1 & dibit;         // bit 0
          index_bch_code++;
        }

      // Read the parity bit
      dibit = getDibit (opts, state);
      bch_code[index_bch_code] = 1 & (dibit >> 1);      // bit 1
      parity = (1 & dibit);     // bit 0

      // Check if the NID is correct
      check_result = check_NID (bch_code, &new_nac, new_duid, parity);
      if (check_result) {
          if (new_nac != state->nac) {
              // NAC fixed by error correction
              state->nac = new_nac;
              state->debug_header_errors++;
          }
          if (strcmp(new_duid, duid) != 0) {
              // DUID fixed by error correction
              //fprintf (stderr,"Fixing DUID %s -> %s\n", duid, new_duid);
              duid[0] = new_duid[0];
              duid[1] = new_duid[1];
              state->debug_header_errors++;
          }
      } else {
          // Check of NID failed and unable to recover its value
          //fprintf (stderr,"NID error\n");
          duid[0] = 'E';
          duid[1] = 'E';
          state->debug_header_critical_errors++;
      }
    }

  if (strcmp (duid, "00") == 0)
    {
      // Header Data Unit
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr," HDU\n");
        }
      if (opts->mbe_out_dir[0] != 0)
        {
          closeMbeOutFile (opts, state);
          openMbeOutFile (opts, state);
        }
      mbe_initMbeParms (state->cur_mp, state->prev_mp, state->prev_mp_enhanced);
      state->lastp25type = 2;
      sprintf (state->fsubtype, " HDU          ");
      processHDU (opts, state);
    }
  else if (strcmp (duid, "11") == 0)
    {
      // Logical Link Data Unit 1
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr," LDU1  ");
        }
      if (opts->mbe_out_dir[0] != 0)
        {
          if (opts->mbe_out_f == NULL)
            {
              openMbeOutFile (opts, state);
            }
        }
      state->lastp25type = 1;
      sprintf (state->fsubtype, " LDU1         ");
      state->numtdulc = 0;
      if (state->payload_algid == 0x81)
      {
        fprintf (stderr, "\n");
        //LFSRP (state); //HERE HERE best placement?
      }

      processLDU1 (opts, state);
    }
  else if (strcmp (duid, "22") == 0)
    {
      // Logical Link Data Unit 2
      if (state->lastp25type != 1)
        {
          if (opts->errorbars == 1)
            {
              printFrameInfo (opts, state);
              fprintf (stderr,"\n Ignoring LDU2 not preceeded by LDU1\n");
            }
          state->lastp25type = 0;
          sprintf (state->fsubtype, "              ");
        }
      else
        {
          if (opts->errorbars == 1)
            {
              printFrameInfo (opts, state);
              fprintf (stderr," LDU2  ");
            }
          if (opts->mbe_out_dir[0] != 0)
            {
              if (opts->mbe_out_f == NULL)
                {
                  openMbeOutFile (opts, state);
                }
            }
          state->lastp25type = 2;
          sprintf (state->fsubtype, " LDU2         ");
          state->numtdulc = 0;
          processLDU2 (opts, state);
        }
    }
  else if (strcmp (duid, "33") == 0)
    {
      // Terminator with subsequent Link Control
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr," TDULC\n");
        }
      if (opts->mbe_out_dir[0] != 0)
        {
          closeMbeOutFile (opts, state);
        }
      mbe_initMbeParms (state->cur_mp, state->prev_mp, state->prev_mp_enhanced);
      state->lasttg = 0;
      state->lastsrc = 0;
      state->lastp25type = 0;
      state->err_str[0] = 0;
      sprintf (state->fsubtype, " TDULC        ");
      state->numtdulc++;
      if ((opts->resume > 0) && (state->numtdulc > opts->resume))
        {
          resumeScan (opts, state);
        }
      processTDULC (opts, state);
      state->err_str[0] = 0;
    }
  else if (strcmp (duid, "03") == 0)
    {
      // Terminator without subsequent Link Control
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr," TDU\n");
        }
      if (opts->mbe_out_dir[0] != 0)
        {
          closeMbeOutFile (opts, state);
        }
      mbe_initMbeParms (state->cur_mp, state->prev_mp, state->prev_mp_enhanced);
      state->lasttg = 0;
      state->lastsrc = 0;
      state->lastp25type = 0;
      state->err_str[0] = 0;
      sprintf (state->fsubtype, " TDU          ");

      processTDU (opts, state);
    }
  else if (strcmp (duid, "13") == 0)
    {
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr," TSDU\n");
        }
      if (opts->resume > 0)
        {
          resumeScan (opts, state);
        }
      state->lasttg = 0;
      state->lastsrc = 0;
      state->lastp25type = 3;
      sprintf (state->fsubtype, " TSDU         ");

      // Now processing NID

      skipDibit (opts, state, 328-25);
    }
  else if (strcmp (duid, "30") == 0)
    {
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr," PDU\n");
        }
      if (opts->resume > 0)
        {
          resumeScan (opts, state);
        }
      if (opts->mbe_out_dir[0] != 0)
        {
          if (opts->mbe_out_f == NULL)
            {
              openMbeOutFile (opts, state);
            }
        }
      state->lastp25type = 4;
      sprintf (state->fsubtype, " PDU          ");
    }
  // try to guess based on previous frame if unknown type
  else if (state->lastp25type == 1)
    {
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr,"(LDU2) ");
        }
      if (opts->mbe_out_dir[0] != 0)
        {
          if (opts->mbe_out_f == NULL)
            {
              openMbeOutFile (opts, state);
            }
        }
      //state->lastp25type = 0;
      // Guess that the state is LDU2
      state->lastp25type = 2;
      sprintf (state->fsubtype, "(LDU2)        ");
      state->numtdulc = 0;
      processLDU2 (opts, state);
    }
  else if (state->lastp25type == 2)
    {
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr,"(LDU1) ");
        }
      if (opts->mbe_out_dir[0] != 0)
        {
          if (opts->mbe_out_f == NULL)
            {
              openMbeOutFile (opts, state);
            }
        }
      //state->lastp25type = 0;
      // Guess that the state is LDU1
      state->lastp25type = 1;
      sprintf (state->fsubtype, "(LDU1)        ");
      state->numtdulc = 0;
      processLDU1 (opts, state);
    }
  else if (state->lastp25type == 3)
    {
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr," (TSDU)\n");
        }
      //state->lastp25type = 0;
      // Guess that the state is TSDU
      state->lastp25type = 3;
      sprintf (state->fsubtype, "(TSDU)        ");

      // Now processing NID

      skipDibit (opts, state, 328-25);
    }
  else if (state->lastp25type == 4)
    {
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          fprintf (stderr," (PDU)\n");
        }
      state->lastp25type = 0;
    }
  else
    {
      state->lastp25type = 0;
      sprintf (state->fsubtype, "              ");
      if (opts->errorbars == 1)
        {
          printFrameInfo (opts, state);
          //fprintf (stderr," duid:%s *Unknown DUID*\n", duid); //prints on dPMR frame 3
          fprintf (stderr, "\n"); //prints on dPMR frame 3
        }
    }
}