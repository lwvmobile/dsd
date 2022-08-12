
#include "dsd.h"
#include "p25p1_heuristics.h"

void
resetState (dsd_state * state)
{

  int i, j;

  // state->dibit_buf = malloc (sizeof (int) * 1000000);
  // state->dibit_buf_p = state->dibit_buf + 200;
  // memset (state->dibit_buf, 0, sizeof (int) * 200);
  state->repeat = 0; //not sure yet
  //state->audio_out_buf = malloc (sizeof (short) * 1000000);
  //memset (state->audio_out_buf, 0, 100 * sizeof (short));
  //state->audio_out_buf_p = state->audio_out_buf + 100;
  //state->audio_out_float_buf = malloc (sizeof (float) * 1000000);
  //memset (state->audio_out_float_buf, 0, 100 * sizeof (float));
  //state->audio_out_float_buf_p = state->audio_out_float_buf + 100;
  //state->audio_out_idx = 0;
  //state->audio_out_idx2 = 0;
  //state->audio_out_temp_buf_p = state->audio_out_temp_buf;
  //state->wav_out_bytes = 0;
  state->center = 0;
  state->jitter = -1;
  state->synctype = -1;
  state->min = -15000;
  state->max = 15000;
  state->lmid = 0;
  state->umid = 0;
  state->minref = -12000;
  state->maxref = 12000;
  state->lastsample = 0;
  for (i = 0; i < 128; i++)
    {
      state->sbuf[i] = 0;
    }
  state->sidx = 0;
  for (i = 0; i < 1024; i++)
    {
      state->maxbuf[i] = 15000;
    }
  for (i = 0; i < 1024; i++)
    {
      state->minbuf[i] = -15000;
    }
  state->midx = 0;
  state->err_str[0] = 0;
  sprintf (state->fsubtype, "              ");
  sprintf (state->ftype, "             ");
  state->symbolcnt = 0;
  state->rf_mod = 0;
  state->numflips = 0;
  state->lastsynctype = -1;
  state->lastp25type = 0;
  state->offset = 0;
  state->carrier = 0;
  for (i = 0; i < 25; i++)
    {
      for (j = 0; j < 16; j++)
        {
          state->tg[i][j] = 48;
        }
    }
  state->tgcount = 0;
  state->lasttg = 0;
  state->lastsrc = 0;
  state->nac = 0;
  state->errs = 0;
  state->errs2 = 0;
  state->mbe_file_type = -1;
  state->optind = 0;
  state->numtdulc = 0;
  state->firstframe = 0;
  sprintf (state->slot0light, " slot0 ");
  sprintf (state->slot1light, " slot1 ");
  state->aout_gain = 25;
  memset (state->aout_max_buf, 0, sizeof (float) * 200);
  state->aout_max_buf_p = state->aout_max_buf;
  state->aout_max_buf_idx = 0;
  state->samplesPerSymbol = 10;
  state->symbolCenter = 4;
  sprintf (state->algid, "________");
  sprintf (state->keyid, "________________");
  state->currentslot = 0;
  state->cur_mp = malloc (sizeof (mbe_parms));
  state->prev_mp = malloc (sizeof (mbe_parms));
  state->prev_mp_enhanced = malloc (sizeof (mbe_parms));
  mbe_initMbeParms (state->cur_mp, state->prev_mp, state->prev_mp_enhanced);
  state->p25kid = 0;

  state->debug_audio_errors = 0;
  state->debug_header_errors = 0;
  state->debug_header_critical_errors = 0;

  state->nxdn_last_ran = 0;
  // state->payload_algid = 0;
  // state->payload_algidR = 0;
  state->dmr_encL = 0;
  state->dmr_encR = 0;

  //each time you run this, it increses memory use by 4MB, massive memory leak
  //need to revisit this sometime and look into only resetting only the necesary items to let P25 switch between signals (C4FM or Wide) without needing a restart
  initialize_p25_heuristics(&state->p25_heuristics); //see if we want to re-init this or not, currently seems to cause memory leak when running over and over, mitigated with a reset flag
}
