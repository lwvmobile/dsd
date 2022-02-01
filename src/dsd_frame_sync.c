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
#include <locale.h>
#include <ncurses.h>

//borrowed from LEH for 'improved NXDN sync detection'
int strncmperr(const char *s1, const char *s2, size_t size, int MaxErr)
{
  int Compare = -1;
  size_t i = 0;
  int err = 0;
  int BreakBeforeEnd = 0;

  if(s1 && s2)
  {
    for(i = 0; i < size; i++)
    {
      if(((s1[i] & 0xFF) != '\0') && ((s2[i] & 0xFF) != '\0'))
      {
        if((s1[i] & 0xFF) != (s2[i] & 0xFF)) err++;
      }
      else
      {
        BreakBeforeEnd = 1;
        break;
      }
    }

    if((err <= MaxErr) && (BreakBeforeEnd == 0))
    {
      Compare = 0;
    }
  } /* End if(s1 && s2) */

  return Compare;
} /* End strncmperr() */
//end LEH

time_t now;
char * getTime(void) //get pretty hh:mm:ss timestamp
{
  time_t t = time(NULL);

  char * curr;
  char * stamp = asctime(localtime( & t));

  curr = strtok(stamp, " ");
  curr = strtok(NULL, " ");
  curr = strtok(NULL, " ");
  curr = strtok(NULL, " ");

  return curr;
}

char * getDate(void) {
  char datename[32];
  char * curr2;
  struct tm * to;
  time_t t;
  t = time(NULL);
  to = localtime( & t);
  strftime(datename, sizeof(datename), "%Y-%m-%d", to);
  curr2 = strtok(datename, " ");
  return curr2;
}


void
printFrameSync (dsd_opts * opts, dsd_state * state, char *frametype, int offset, char *modulation)
{
  //char datestr[32];
  //struct tm timep;
  //erase();
  //attron(COLOR_PAIR(4));
  //for (short int i = 0; i < 7; i++) {
    //printw("%s \n", FM_banner2[i]);}
  //attroff(COLOR_PAIR(4));

  if (opts->verbose > 0)
    {
	  //strftime (datestr, 31, "%Y-%m-%d-%H%M%S", &timep);
      //printf ("Sync: %s ", frametype);
      //printf ("%s %s Sync: %s ", getDate(), getTime(), frametype);
      printf ("%s ", getTime());
      printf ("Sync: %s ", frametype);
      //printf("%s Sync: %s ", getTime(), frametype);
      //printw("%s Sync: %s ", getTime(), frametype);
    }
  if (opts->verbose > 2)
    {
      printf ("o: %4i ", offset);
      printw("o: %4i ", offset);
    }
  if (opts->verbose > 1)
    {
      printf ("mod: %s ", modulation);
      printw("mod: %s ", modulation);
    }
  if (opts->verbose > 2)
    {
      printf ("g: %f ", state->aout_gain);
      printw("g: %f ", state->aout_gain);
    }
 refresh();
}

int
getFrameSync (dsd_opts * opts, dsd_state * state)
{
  /* detects frame sync and returns frame type
   *  0 = +P25p1
   *  1 = -P25p1
   *  2 = +X2-TDMA (non inverted signal data frame)
   *  3 = -X2-TDMA (inverted signal voice frame)
   *  4 = +X2-TDMA (non inverted signal voice frame)
   *  5 = -X2-TDMA (inverted signal data frame)
   *  6 = +D-STAR
   *  7 = -D-STAR
   *  8 = +NXDN (non inverted voice frame)
   *  9 = -NXDN (inverted voice frame)
   * 10 = +DMR (non inverted signal data frame)
   * 11 = -DMR (inverted signal voice frame)
   * 12 = +DMR (non inverted signal voice frame)
   * 13 = -DMR (inverted signal data frame)
   * 14 = +ProVoice
   * 15 = -ProVoice
   * 16 = +NXDN (non inverted data frame)
   * 17 = -NXDN (inverted data frame)
   * 18 = +D-STAR_HD
   * 19 = -D-STAR_HD
   */


  int i, j, t, o, dibit, sync, symbol, synctest_pos, lastt;
  char synctest[25];
  char synctest18[19];
  char synctest32[33];
  char modulation[8];
  char *synctest_p;
  char synctest_buf[10240];
  int lmin, lmax, lidx;
  int lbuf[24], lbuf2[24];
  int lsum;
  char spectrum[64];

  for (i = 18; i < 24; i++)
    {
      lbuf[i] = 0;
      lbuf2[i] = 0;
    }

  // detect frame sync
  t = 0;
  synctest[24] = 0;
  synctest18[18] = 0;
  synctest32[32] = 0;
  synctest_pos = 0;
  synctest_p = synctest_buf + 10;
  sync = 0;
  lmin = 0;
  lmax = 0;
  lidx = 0;
  lastt = 0;
  state->numflips = 0;
  if ((opts->symboltiming == 1) && (state->carrier == 1))
    {
      printf ("\nSymbol Timing:\n");
      //printw("\nSymbol Timing:\n");
    }
  while (sync == 0)
    {
      t++;
      symbol = getSymbol (opts, state, 0);

      lbuf[lidx] = symbol;
      state->sbuf[state->sidx] = symbol;
      if (lidx == 23)
        {
          lidx = 0;
        }
      else
        {
          lidx++;
        }
      if (state->sidx == (opts->ssize - 1))
        {
          state->sidx = 0;
        }
      else
        {
          state->sidx++;
        }

      if (lastt == 23)
        {
          lastt = 0;
          if (state->numflips > opts->mod_threshold)
            {
              if (opts->mod_qpsk == 1)
                {
                  state->rf_mod = 1;
                }
            }
          else if (state->numflips > 18)
            {
              if (opts->mod_gfsk == 1)
                {
                  state->rf_mod = 2;
                }
            }
          else
            {
              if (opts->mod_c4fm == 1)
                {
                  state->rf_mod = 0;
                }
            }
          state->numflips = 0;
        }
      else
        {
          lastt++;
        }

      if (state->dibit_buf_p > state->dibit_buf + 900000)
        {
    	  state->dibit_buf_p = state->dibit_buf + 200;
        }

      //determine dibit state
      if (symbol > 0)
        {
          *state->dibit_buf_p = 1;
          state->dibit_buf_p++;
          dibit = 49;               // '1'
        }
      else
        {
          *state->dibit_buf_p = 3;
          state->dibit_buf_p++;
          dibit = 51;               // '3'
        }

      *synctest_p = dibit;
      if (t >= 18)
        {
          for (i = 0; i < 24; i++)
            {
              lbuf2[i] = lbuf[i];
            }
          qsort (lbuf2, 24, sizeof (int), comp);
          lmin = (lbuf2[2] + lbuf2[3] + lbuf2[4]) / 3;
          lmax = (lbuf2[21] + lbuf2[20] + lbuf2[19]) / 3;

          if (state->rf_mod == 1)
            {
              state->minbuf[state->midx] = lmin;
              state->maxbuf[state->midx] = lmax;
              if (state->midx == (opts->msize - 1))
                {
                  state->midx = 0;
                }
              else
                {
                  state->midx++;
                }
              lsum = 0;
              for (i = 0; i < opts->msize; i++)
                {
                  lsum += state->minbuf[i];
                }
              state->min = lsum / opts->msize;
              lsum = 0;
              for (i = 0; i < opts->msize; i++)
                {
                  lsum += state->maxbuf[i];
                }
              state->max = lsum / opts->msize;
              state->center = ((state->max) + (state->min)) / 2;
              state->maxref = (int)((state->max) * 0.80F);
              state->minref = (int)((state->min) * 0.80F);
            }
          else
            {
              state->maxref = state->max;
              state->minref = state->min;
            }

          if (state->rf_mod == 0)
            {
              sprintf (modulation, "C4FM");
            }
          else if (state->rf_mod == 1)
            {
              sprintf (modulation, "QPSK");
            }
          else if (state->rf_mod == 2)
            {
              sprintf (modulation, "GFSK");
            }

          if (opts->datascope == 1)
            {
              if (lidx == 0)
                {
                  for (i = 0; i < 64; i++)
                    {
                      spectrum[i] = 0;
                    }
                  for (i = 0; i < 24; i++)
                    {
                      o = (lbuf2[i] + 32768) / 1024;
                      spectrum[o]++;
                    }
                  if (state->symbolcnt > (4800 / opts->scoperate))
                    {
                      state->symbolcnt = 0;
                      printf ("\n");
                      printf ("Demod mode:     %s                Nac:                     %4X\n", modulation, state->nac);
                      printf ("Frame Type:    %s        Talkgroup:            %7i\n", state->ftype, state->lasttg);
                      printf ("Frame Subtype: %s       Source:          %12i\n", state->fsubtype, state->lastsrc);
                      printf ("TDMA activity:  %s %s     Voice errors: %s\n", state->slot0light, state->slot1light, state->err_str);
                      printf ("+----------------------------------------------------------------+\n");
                      for (i = 0; i < 10; i++)
                        {
                          printf ("|");
                          for (j = 0; j < 64; j++)
                            {
                              if (i == 0)
                                {
                                  if ((j == ((state->min) + 32768) / 1024) || (j == ((state->max) + 32768) / 1024))
                                    {
                                      printf ("#");
                                    }
                                  else if (j == (state->center + 32768) / 1024)
                                    {
                                      printf ("!");
                                    }
                                  else
                                    {
                                      if (j == 32)
                                        {
                                          printf ("|");
                                        }
                                      else
                                        {
                                          printf (" ");
                                        }
                                    }
                                }
                              else
                                {
                                  if (spectrum[j] > 9 - i)
                                    {
                                      printf ("*");
                                    }
                                  else
                                    {
                                      if (j == 32)
                                        {
                                          printf ("|");
                                        }
                                      else
                                        {
                                          printf (" ");
                                        }
                                    }
                                }
                            }
                          printf ("|\n");
                        }
                      printf ("+----------------------------------------------------------------+\n");
                    }
                }
            }

          strncpy (synctest, (synctest_p - 23), 24);
          if (opts->frame_p25p1 == 1)
            {
              if (strcmp (synctest, P25P1_SYNC) == 0)
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, " P25 Phase 1 ");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, " +P25p1    ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = 0;
                  return (0);
                }
              if (strcmp (synctest, INV_P25P1_SYNC) == 0)
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, " P25 Phase 1 ");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, " -P25p1    ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = 1;
                  return (1);
                }
            }
          if (opts->frame_x2tdma == 1)
            {
              if ((strcmp (synctest, X2TDMA_BS_DATA_SYNC) == 0) || (strcmp (synctest, X2TDMA_MS_DATA_SYNC) == 0))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + (lmax)) / 2;
                  state->min = ((state->min) + (lmin)) / 2;
                  if (opts->inverted_x2tdma == 0)
                    {
                      // data frame
                      sprintf (state->ftype, " X2-TDMA     ");
                      if (opts->errorbars == 1)
                        {
                          printFrameSync (opts, state, " +X2-TDMA  ", synctest_pos + 1, modulation);
                        }
                      state->lastsynctype = 2;
                      return (2);
                    }
                  else
                    {
                      // inverted voice frame
                      sprintf (state->ftype, " X2-TDMA     ");
                      if (opts->errorbars == 1)
                        {
                          printFrameSync (opts, state, " -X2-TDMA  ", synctest_pos + 1, modulation);
                        }
                      if (state->lastsynctype != 3)
                        {
                          state->firstframe = 1;
                        }
                      state->lastsynctype = 3;
                      return (3);
                    }
                }
              if ((strcmp (synctest, X2TDMA_BS_VOICE_SYNC) == 0) || (strcmp (synctest, X2TDMA_MS_VOICE_SYNC) == 0))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  if (opts->inverted_x2tdma == 0)
                    {
                      // voice frame
                      sprintf (state->ftype, " X2-TDMA     ");
                      if (opts->errorbars == 1)
                        {
                          printFrameSync (opts, state, " +X2-TDMA  ", synctest_pos + 1, modulation);
                        }
                      if (state->lastsynctype != 4)
                        {
                          state->firstframe = 1;
                        }
                      state->lastsynctype = 4;
                      return (4);
                    }
                  else
                    {
                      // inverted data frame
                      sprintf (state->ftype, " X2-TDMA     ");
                      if (opts->errorbars == 1)
                        {
                          printFrameSync (opts, state, " -X2-TDMA  ", synctest_pos + 1, modulation);
                        }
                      state->lastsynctype = 5;
                      return (5);
                    }
                }
            }
          if (opts->frame_dmr == 1)
            {
              if ((strcmp (synctest, DMR_MS_DATA_SYNC) == 0) || (strcmp (synctest, DMR_BS_DATA_SYNC) == 0))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + (lmax)) / 2;
                  state->min = ((state->min) + (lmin)) / 2;
                  if (opts->inverted_dmr == 0)
                    {
                      // data frame
                      sprintf (state->ftype, " DMR         ");
                      if (opts->errorbars == 1)
                        {
                          printFrameSync (opts, state, " +DMR      ", synctest_pos + 1, modulation);
                        }
                      state->lastsynctype = 10;
                      return (10);
                    }
                  else
                    {
                      // inverted voice frame
                      sprintf (state->ftype, " DMR         ");
                      if (opts->errorbars == 1)
                        {
                          printFrameSync (opts, state, " -DMR      ", synctest_pos + 1, modulation);
                        }
                      if (state->lastsynctype != 11)
                        {
                          state->firstframe = 1;
                        }
                      state->lastsynctype = 11;
                      return (11);
                    }
                }
              if ((strcmp (synctest, DMR_MS_VOICE_SYNC) == 0) || (strcmp (synctest, DMR_BS_VOICE_SYNC) == 0))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  if (opts->inverted_dmr == 0)
                    {
                      // voice frame
                      sprintf (state->ftype, " DMR         ");
                      if (opts->errorbars == 1)
                        {
                          printFrameSync (opts, state, " +DMR      ", synctest_pos + 1, modulation);
                        }
                      if (state->lastsynctype != 12)
                        {
                          state->firstframe = 1;
                        }
                      state->lastsynctype = 12;
                      return (12);
                    }
                  else
                    {
                      // inverted data frame
                      sprintf (state->ftype, " DMR         ");
                      if (opts->errorbars == 1)
                        {
                          printFrameSync (opts, state, " -DMR      ", synctest_pos + 1, modulation);
                        }
                      state->lastsynctype = 13;
                      return (13);
                    }
                }
            }
          if (opts->frame_provoice == 1)
            {
              strncpy (synctest32, (synctest_p - 31), 32);
              if ((strcmp (synctest32, PROVOICE_SYNC) == 0) || (strcmp (synctest32, PROVOICE_EA_SYNC) == 0))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, " ProVoice    ");
                  if (opts->errorbars == 1)
                  //if (opts->errorbars == 1 && (time(NULL) - now) > 2 )
                    {
                      printFrameSync (opts, state, " -ProVoice ", synctest_pos + 1, modulation);
                      now = time(NULL);
                    }
                  state->lastsynctype = 14;
                  return (14);
                }
              else if ((strcmp (synctest32, INV_PROVOICE_SYNC) == 0) || (strcmp (synctest32, INV_PROVOICE_EA_SYNC) == 0))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, " ProVoice    ");
                  if (opts->errorbars == 1)
                  //if (opts->errorbars == 1 && (time(NULL) - now) > 2 )
                    {
                      printFrameSync (opts, state, " -ProVoice ", synctest_pos + 1, modulation);
                      now = time(NULL);
                    }
                  state->lastsynctype = 15;
                  return (15);
                }

            }

          if ((opts->frame_nxdn96 == 1) || (opts->frame_nxdn48 == 1))
            {
              strncpy (synctest18, (synctest_p - 17), 18); //seems like other sync tests do this as well
              //if ((strcmp (synctest18, NXDN_BS_VOICE_SYNC) == 0) || (strcmp (synctest18, NXDN_MS_VOICE_SYNC) == 0)) //or is this the double up test?
              //borrowing from LEH here WOW! This works so much better, its not even a joke
              if ((strncmperr (synctest18, NXDN_BS_VOICE_SYNC, 18, 1) == 0) || (strncmperr (synctest18, NXDN_MS_VOICE_SYNC, 18, 1) == 0))
                {
                  //if ((state->lastsynctype == 8) || (state->lastsynctype == 16)) //is this checking for multiple matches first, could be causing 'skips' in audio decode HERE HERE
                  if ( (opts->frame_nxdn96 == 1) ||(opts->frame_nxdn48 == 1)) //kind of hacky, but too lazy to remove brackets and re-indent
                    {
                      state->carrier = 1;
                      state->offset = synctest_pos;
                      state->max = ((state->max) + lmax) / 2;
                      state->min = ((state->min) + lmin) / 2;
                      if (state->samplesPerSymbol == 20)
                        {
                          sprintf (state->ftype, " NXDN48      ");
                          if (opts->errorbars == 1)
                            {
                              printFrameSync (opts, state, " +NXDN48   ", synctest_pos + 1, modulation);
                            }
                        }
                      else
                        {
                          sprintf (state->ftype, " NXDN96      ");
                          if (opts->errorbars == 1)
                            {
                              printFrameSync (opts, state, " +NXDN96   ", synctest_pos + 1, modulation);
                            }
                        }
                      state->lastsynctype = 8;
                      return (8);
                    }
                  else
                    {
                      state->lastsynctype = 8;
                    }
                }
              //else if ((strcmp (synctest18, INV_NXDN_BS_VOICE_SYNC) == 0) || (strcmp (synctest18, INV_NXDN_MS_VOICE_SYNC) == 0))
              else if ((strncmperr (synctest18, NXDN_BS_DATA_SYNC, 18, 1) == 0) || (strncmperr (synctest18, NXDN_MS_DATA_SYNC, 18, 1) == 0))
                {
                  //if ((state->lastsynctype == 9) || (state->lastsynctype == 17))
                  if ( (opts->frame_nxdn96 == 1) ||(opts->frame_nxdn48 == 1)) //again, skip the double up
                    {
                      state->carrier = 1;
                      state->offset = synctest_pos;
                      state->max = ((state->max) + lmax) / 2;
                      state->min = ((state->min) + lmin) / 2;
                      if (state->samplesPerSymbol == 20)
                        {
                          sprintf (state->ftype, " NXDN48      ");
                          if (opts->errorbars == 1)
                            {
                              printFrameSync (opts, state, " -NXDN48   ", synctest_pos + 1, modulation);
                            }
                        }
                      else
                        {
                          sprintf (state->ftype, " NXDN96      ");
                          if (opts->errorbars == 1)
                            {
                              printFrameSync (opts, state, " -NXDN96   ", synctest_pos + 1, modulation);
                            }
                        }
                      state->lastsynctype = 9;
                      return (9);
                    }
                  else
                    {
                      state->lastsynctype = 9;
                    }
                }
              //else if ((strcmp (synctest18, NXDN_BS_DATA_SYNC) == 0) || (strcmp (synctest18, NXDN_MS_DATA_SYNC) == 0))
              else if ((strncmperr (synctest18, NXDN_BS_DATA_SYNC, 18, 1) == 0) || (strncmperr (synctest18, NXDN_MS_DATA_SYNC, 18, 1) == 0))
                {
                  //if ((state->lastsynctype == 8) || (state->lastsynctype == 16))
                  if ( (opts->frame_nxdn96 == 1) ||(opts->frame_nxdn48 == 1))
                    {
                      state->carrier = 1;
                      state->offset = synctest_pos;
                      state->max = ((state->max) + lmax) / 2;
                      state->min = ((state->min) + lmin) / 2;
                      if (state->samplesPerSymbol == 20)
                        {
                          sprintf (state->ftype, " NXDN48      ");
                          if (opts->errorbars == 1)
                            {
                              printFrameSync (opts, state, " +NXDN48   ", synctest_pos + 1, modulation);
                            }
                        }
                      else
                        {
                          sprintf (state->ftype, " NXDN96      ");
                          if (opts->errorbars == 1)
                            {
                              printFrameSync (opts, state, " +NXDN96   ", synctest_pos + 1, modulation);
                            }
                        }
                      state->lastsynctype = 16;
                      return (16);
                    }
                  else
                    {
                      state->lastsynctype = 16;
                    }
                }
              //else if ((strcmp (synctest18, INV_NXDN_BS_DATA_SYNC) == 0) || (strcmp (synctest18, INV_NXDN_MS_DATA_SYNC) == 0))
              else if ((strncmperr (synctest18, INV_NXDN_BS_DATA_SYNC, 18, 1) == 0) || (strncmperr (synctest18, INV_NXDN_MS_DATA_SYNC, 18, 1) == 0))
                {
                  //if ((state->lastsynctype == 9) || (state->lastsynctype == 17))
                  if ( (opts->frame_nxdn96 == 1) ||(opts->frame_nxdn48 == 1))
                    {
                      state->carrier = 1;
                      state->offset = synctest_pos;
                      state->max = ((state->max) + lmax) / 2;
                      state->min = ((state->min) + lmin) / 2;
                      sprintf (state->ftype, " NXDN        ");
                      if (state->samplesPerSymbol == 20)
                        {
                          sprintf (state->ftype, " NXDN48      ");
                          if (opts->errorbars == 1)
                            {
                              printFrameSync (opts, state, " -NXDN48   ", synctest_pos + 1, modulation);
                            }
                        }
                      else
                        {
                          sprintf (state->ftype, " NXDN96      ");
                          if (opts->errorbars == 1)
                            {
                              printFrameSync (opts, state, " -NXDN96   ", synctest_pos + 1, modulation);
                            }
                        }
                      state->lastsynctype = 17;
                      return (17);
                    }
                  else
                    {
                      state->lastsynctype = 17;
                    }
                }
            }
          if (opts->frame_dstar == 1)
            {
              if (strcmp (synctest, DSTAR_SYNC) == 0)
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, " D-STAR      ");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, " +D-STAR   ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = 6;
                  return (6);
                }
              if (strcmp (synctest, INV_DSTAR_SYNC) == 0)
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, " D-STAR      ");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, " -D-STAR   ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = 7;
                  return (7);
                }
              if (strcmp (synctest, DSTAR_HD) == 0)
                 {
                   state->carrier = 1;
                   state->offset = synctest_pos;
                   state->max = ((state->max) + lmax) / 2;
                   state->min = ((state->min) + lmin) / 2;
                   sprintf (state->ftype, " D-STAR_HD   ");
                   if (opts->errorbars == 1)
                     {
                       printFrameSync (opts, state, " +D-STAR_HD   ", synctest_pos + 1, modulation);
                     }
                   state->lastsynctype = 18;
                   return (18);
                 }
              if (strcmp (synctest, INV_DSTAR_HD) == 0)
                {
                   state->carrier = 1;
                   state->offset = synctest_pos;
                   state->max = ((state->max) + lmax) / 2;
                   state->min = ((state->min) + lmin) / 2;
                   sprintf (state->ftype, " D-STAR_HD   ");
                   if (opts->errorbars == 1)
                     {
                       printFrameSync (opts, state, " -D-STAR_HD   ", synctest_pos + 1, modulation);
                     }
                   state->lastsynctype = 19;
                   return (19);
                 }

            }

          if ((t == 24) && (state->lastsynctype != -1))
            {
              if ((state->lastsynctype == 0) && ((state->lastp25type == 1) || (state->lastp25type == 2)))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + (lmax)) / 2;
                  state->min = ((state->min) + (lmin)) / 2;
                  sprintf (state->ftype, "(P25 Phase 1)");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, "(+P25p1)   ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = -1;
                  return (0);
                }
              else if ((state->lastsynctype == 1) && ((state->lastp25type == 1) || (state->lastp25type == 2)))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, "(P25 Phase 1)");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, "(-P25p1)   ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = -1;
                  return (1);
                }
              else if ((state->lastsynctype == 3) && ((strcmp (synctest, X2TDMA_BS_VOICE_SYNC) != 0) || (strcmp (synctest, X2TDMA_MS_VOICE_SYNC) != 0)))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, "(X2-TDMA)    ");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, "(-X2-TDMA) ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = -1;
                  return (3);
                }
              else if ((state->lastsynctype == 4) && ((strcmp (synctest, X2TDMA_BS_DATA_SYNC) != 0) || (strcmp (synctest, X2TDMA_MS_DATA_SYNC) != 0)))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, "(X2-TDMA)    ");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, "(+X2-TDMA) ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = -1;
                  return (4);
                }
              else if ((state->lastsynctype == 11) && ((strcmp (synctest, DMR_BS_VOICE_SYNC) != 0) || (strcmp (synctest, DMR_MS_VOICE_SYNC) != 0)))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, "(DMR)        ");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, "(-DMR)     ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = -1;
                  return (11);
                }
              else if ((state->lastsynctype == 12) && ((strcmp (synctest, DMR_BS_DATA_SYNC) != 0) || (strcmp (synctest, DMR_MS_DATA_SYNC) != 0)))
                {
                  state->carrier = 1;
                  state->offset = synctest_pos;
                  state->max = ((state->max) + lmax) / 2;
                  state->min = ((state->min) + lmin) / 2;
                  sprintf (state->ftype, "(DMR)        ");
                  if (opts->errorbars == 1)
                    {
                      printFrameSync (opts, state, "(+DMR)     ", synctest_pos + 1, modulation);
                    }
                  state->lastsynctype = -1;
                  return (12);
                }
            }
        }

      if (exitflag == 1)
        {
          cleanupAndExit (opts, state);
        }

      if (synctest_pos < 10200)
        {
          synctest_pos++;
          synctest_p++;
        }
      else
        {
          // buffer reset
          synctest_pos = 0;
          synctest_p = synctest_buf;
          noCarrier (opts, state);
        }

      if (state->lastsynctype != 1)
        {
          if (synctest_pos >= 1800)
            {
              if ((opts->errorbars == 1) && (opts->verbose > 1) && (state->carrier == 1))
                {
                  printf ("Sync: no sync\n");
                  printf("Press CTRL + C to close.\n"); //Kindly remind user to double tap CTRL + C
                }
              noCarrier (opts, state);
              return (-1);
            }
        }
    }

  return (-1);
}
