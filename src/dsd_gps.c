/*-------------------------------------------------------------------------------
 * dsd_gps.c
 * GPS Handling Functions for Various Protocols
 *
 * LWVMOBILE
 * 2023-12 DSD-FME Florida Man Edition
 *-----------------------------------------------------------------------------*/

#include "dsd.h"

void lip_protocol_decoder (dsd_opts * opts, dsd_state * state, uint8_t * input)
{
  //NOTE: This is defined in ETSI TS 102 361-4 V1.12.1 (2023-07) p208
  //6.6.11.3.2 USBD Polling Service Poll Response PDU for LIP (That's a mouthful)

  int slot = state->currentslot;

  //NOTE: This format is pretty much the same as DMR EMB GPS, but has a few extra elements,
  //so I got lazy and just lifted most of the code from there, also assuming same lat/lon calcs
  //since those have been tested to work in DMR EMB GPS and units are same in LIP

  fprintf (stderr, "Location Information Protocol; ");

  // uint8_t service_type = (uint8_t)ConvertBitIntoBytes(&input[0], 4); //checked before arrival here
  uint8_t time_elapsed = (uint8_t)ConvertBitIntoBytes(&input[6], 2);
  uint8_t lon_sign = input[8];
  uint32_t lon = (uint32_t)ConvertBitIntoBytes(&input[9], 24); //8, 25
  uint8_t lat_sign = input[33];
  uint32_t lat = (uint32_t)ConvertBitIntoBytes(&input[34], 23); //33, 24
  uint8_t pos_err = (uint8_t)ConvertBitIntoBytes(&input[57], 2);
  uint8_t hor_vel = (uint8_t)ConvertBitIntoBytes(&input[59], 7);
  uint8_t dir_tra = (uint8_t)ConvertBitIntoBytes(&input[66], 4);
  uint8_t reason = (uint8_t)ConvertBitIntoBytes(&input[70], 3);
  uint8_t add_hash = (uint8_t)ConvertBitIntoBytes(&input[73], 8); //MS Source Address Hash

  //NOTE: May need to use double instead of float to avoid rounding errors
  double latitude, longitude = 0.0f;
  double lat_unit = (double)180/ pow (2.0, 24); //180 divided by 2^24 -- 6.3.30
  double lon_unit = (double)360/ pow (2.0, 25); //360 divided by 2^25 -- 6.3.50
  double lon_sf = 1.0f; //float value we can multiple longitude with
  double lat_sf = 1.0f; //float value we can multiple latitude with

  char latstr[3];
  char lonstr[3];
  sprintf (latstr, "%s", "N");
  sprintf (lonstr, "%s", "E");

  char deg_glyph[4];
  sprintf (deg_glyph, "%s", "°");

  //lat and lon calculations
  if (lat_sign)
  {
    lat = 0x800001 - lat;
    sprintf (latstr, "%s", "S");
    lat_sf = -1.0f;
  } 
  latitude = ((double)lat * lat_unit);

  if (lon_sign)
  {
    lon = 0x1000001 - lon;
    sprintf (lonstr, "%s", "W");
    lon_sf = -1.0f;
  } 
  longitude = ((double)lon * lon_unit);

  //6.3.63 Position Error
  //6.3.17 Horizontal velocity
  /*
    Horizontal velocity shall be encoded for speeds 0 km/h to 28 km/h in 1 km/h steps and 
    from 28 km/h onwards using equation: v = C × (1 + x)^(K-A) + B where:
  */
  float v = 0.0f;
  float C = 16.0f;
  float x = 0.038f;
  float A = 13.0f;
  float K = (float)hor_vel;
  float B = 0.0f;

  if (hor_vel > 28)
    v = C * (pow(1+x, K-A)) + B; //I think this pow function is set up correctly
  else v = (float)hor_vel;

  float dir = (((float)dir_tra + 11.25f)/22.5f); //page 68, Table 6.45

  //truncated and rounded forms
  int vt = (int)v;
  int dt = (int)dir;

  //sanity check
  if (abs (latitude) < 90 && abs(longitude) < 180)
  {
    fprintf (stderr, "Src(Hash); %03d;  Lat: %.5lf%s%s Lon: %.5lf%s%s (%.5lf, %.5lf); Spd: %d km/h; Dir: %d%s",add_hash, latitude, deg_glyph, latstr, longitude, deg_glyph, lonstr, lat_sf * latitude, lon_sf * longitude, vt, dt, deg_glyph);

    //6.3.63 Position Error
    uint16_t position_error = 2 * pow(10, pos_err); //2 * 10^pos_err 
    if (pos_err == 0x7 ) fprintf (stderr, "\n  Position Error: Unknown or Invalid;");
    else fprintf (stderr, "\n  Position Error: Less than %dm;", position_error);

    //Reason For Sending 6.6.11.3.3 Table 6.80
    if (reason) fprintf (stderr, " Reserved: %d;", reason);
    else fprintf (stderr, " Request Response; ");

    //6.3.78 Time elapsed
    if (time_elapsed == 0) fprintf (stderr, " TE: < 5s;");
    if (time_elapsed == 1) fprintf (stderr, " TE: < 5m;");
    if (time_elapsed == 2) fprintf (stderr, " TE: < 30m;");
    if (time_elapsed == 3) fprintf (stderr, " TE: NA or UNK;"); //not applicable or unknown

    //save to array for ncurses
    if (pos_err != 0x7)
    {
      sprintf (state->dmr_embedded_gps[slot], "%03d; LIP: %.5lf%s%s %.5lf%s%s; Err: %dm; Spd: %d km/h; Dir: %d%s", add_hash, latitude, deg_glyph, latstr, longitude, deg_glyph, lonstr, position_error, vt, dt, deg_glyph);
    }
    else sprintf (state->dmr_embedded_gps[slot], "%03d; LIP: %.5lf%s%s %.5lf%s%s Unknown Pos Err; Spd: %d km/h; Dir %d%s", add_hash, latitude, deg_glyph, latstr, longitude, deg_glyph, lonstr, vt, dt, deg_glyph);

    //save to LRRP report for mapping/logging
    FILE * pFile; //file pointer
    if (opts->lrrp_file_output == 1)
    {

      char * datestr = getDate();
      char * timestr = getTime();

      //open file by name that is supplied in the ncurses terminal, or cli
      pFile = fopen (opts->lrrp_out_file, "a");
      fprintf (pFile, "%s\t", datestr );
      fprintf (pFile, "%s\t", timestr );
      fprintf (pFile, "%08d\t", add_hash);
      fprintf (pFile, "%.5lf\t", latitude);
      fprintf (pFile, "%.5lf\t", longitude);
      fprintf (pFile, "%d\t", vt); //speed in km/h
      fprintf (pFile, "%d\t", dt); //direction of travel
      fprintf (pFile, "\n");
      fclose (pFile);

      if (timestr != NULL)
      {
        free (timestr);
        timestr = NULL;
      }
      if (datestr != NULL)
      {
        free (datestr);
        datestr = NULL;
      }

    }

  }
  else fprintf (stderr, " Position Calculation Error;");

}

void nmea_iec_61162_1 (dsd_opts * opts, dsd_state * state, uint8_t * input, uint32_t src, int type)
{
  int slot = state->currentslot;

  //NOTE: The Only difference between Short (type == 1) and Long Format (type == 2) 
  //is the utc_ss3 on short vs utc_ss6 and inclusion of COG value on long

  //NOTE: MFID Specific Formats are not handled here, unknown, could be added if worked out
  //they could share most of the same elements and use the large spare bits in block 2 for extra
  // uint8_t mfid = (uint8_t)ConvertBitIntoBytes(&input[88], 8); //on type 3, last octet of first block carries MFID

  // uint8_t nmea_c = input[0];  //encrypted -- checked before we get here
  uint8_t nmea_ns = input[1]; //north/south (lat sign)
  uint8_t nmea_ew = input[2]; //east/west (lon sign)
  uint8_t nmea_q = input[3]; //Quality Indicator (no fix or fix valid)
  uint8_t nmea_speed = (uint8_t)ConvertBitIntoBytes(&input[4], 7); //speed in knots (127 = greater than 126 knots)

  //Latitude Bits
  uint8_t nmea_ndeg = (uint8_t)ConvertBitIntoBytes(&input[11], 7); //Latitude Degrees
  uint8_t nmea_nmin = (uint8_t)ConvertBitIntoBytes(&input[18], 6); //Latitude Minutes
  uint16_t nmea_nminf = (uint16_t)ConvertBitIntoBytes(&input[24], 14); //Latitude Fractions of Minutes

  //Longitude Bits
  uint8_t nmea_edeg = (uint8_t)ConvertBitIntoBytes(&input[38], 8); //Longitude Degrees
  uint8_t nmea_emin = (uint8_t)ConvertBitIntoBytes(&input[46], 6); //Longitude Minutes
  uint16_t nmea_eminf = (uint16_t)ConvertBitIntoBytes(&input[52], 14); //Longitude Fractions of Minutes

  //UTC Time and COG
  uint8_t nmea_utc_hh  = (uint8_t)ConvertBitIntoBytes(&input[66], 5);
  uint8_t nmea_utc_mm  = (uint8_t)ConvertBitIntoBytes(&input[71], 6);
  //seconds and the addition of COG is the difference between short and long formats
  uint8_t nmea_utc_ss3 = (uint8_t)ConvertBitIntoBytes(&input[77], 3); //seconds in 10s
  uint8_t nmea_utc_ss6 = (uint8_t)ConvertBitIntoBytes(&input[77], 6); //seconds in 1s
  uint16_t nmea_cog = (uint16_t)ConvertBitIntoBytes(&input[103], 9); //course over ground in degrees

  //lat and lon conversion
  char deg_glyph[4];
  sprintf (deg_glyph, "%s", "°");
  float latitude = 0.0f;
  float longitude = 0.0f;
  // float m_unit = 1.0f / 60.0f;     //unit to convert min into decimal value - (1/60)*60 minutes = 1 degree
  // float mm_unit = 1.0f / 10000.0f;  //unit to convert minf into decimal value - (0000 - 9999) 0.0001×9999 = .9999 minutes, so its sub 1 minute decimal

  //testing w/ Harris NMEA like values (ran tests over there with this code, this seems to work on those values)
  float m_unit = 1.0f / 60.0f;     //unit to convert min into decimal value - (1/60)*60 minutes = 1 degree
  float mm_unit = 1.0f / 600000.0f;  //unit to convert minf into decimal value - (0000 - 9999) 0.0001×9999 = .9999 minutes, so its sub 1 minute decimal

  //speed conversion
  float fmps, fmph, fkph = 0.0f; //conversion of knots to mps, kph, and mph values
  fmps = (float)nmea_speed * 0.514444; UNUSED(fmps);
  fmph = (float)nmea_speed * 1.15078f; UNUSED(fmph);
  fkph = (float)nmea_speed * 1.852f;

  //calculate decimal representation of latidude and longitude (need some samples to test)
  latitude  = ( (float)nmea_ndeg + ((float)nmea_nmin*m_unit) + ((float)nmea_nminf*mm_unit) );
  longitude = ( (float)nmea_edeg + ((float)nmea_emin*m_unit) + ((float)nmea_eminf*mm_unit) );

  if (!nmea_ns) latitude  *= -1.0f; //0 is South, 1 is North
  if (!nmea_ew) longitude *= -1.0f; //0 is West, 1 is East

  fprintf (stderr, " GPS: %f%s, %f%s;", latitude, deg_glyph, longitude, deg_glyph);

  //Speed in Knots
  if (nmea_speed > 126)
    fprintf (stderr, " SPD > 126 knots or %f kph;", fkph);
  else
    fprintf (stderr, " SPD: %d knots; %f kph;", nmea_speed, fkph);

  //Print UTC Time and COG according to Format Type
  if (type == 1)
    fprintf (stderr, " FIX: %d; %02d:%02d:%02d UTC; Short Format;", nmea_q, nmea_utc_hh, nmea_utc_mm, nmea_utc_ss3);
  if (type == 2)
    fprintf (stderr, " FIX: %d; %02d:%02d:%02d UTC; COG: %d%s; Long Format;", nmea_q, nmea_utc_hh, nmea_utc_mm, nmea_utc_ss6, nmea_cog, deg_glyph);

  //save to ncurses string
  sprintf (state->dmr_embedded_gps[slot], "GPS: (%f%s, %f%s)", latitude, deg_glyph, longitude, deg_glyph);

  //save to LRRP report for mapping/logging
  FILE * pFile; //file pointer
  if (opts->lrrp_file_output == 1)
  {

    char * datestr = getDate();
    char * timestr = getTime();

    int s = (int)fkph; //rounded interger format for the log report
    int a = 0;
    if (type == 2)
      a = nmea_cog; //long format only
    //open file by name that is supplied in the ncurses terminal, or cli
    pFile = fopen (opts->lrrp_out_file, "a");
    fprintf (pFile, "%s\t", datestr );
    fprintf (pFile, "%s\t", timestr ); //could switch to UTC time if desired, but would require local user offset
    fprintf (pFile, "%08d\t", src);
    fprintf (pFile, "%.6lf\t", latitude);
    fprintf (pFile, "%.6lf\t", longitude);
    fprintf (pFile, "%d\t ", s);
    fprintf (pFile, "%d\t ", a);
    fprintf (pFile, "\n");
    fclose (pFile);

    if (timestr != NULL)
    {
      free (timestr);
      timestr = NULL;
    }
    if (datestr != NULL)
    {
      free (datestr);
      datestr = NULL;
    }

  }

}

//restructured the Harris GPS to flow more like the DMR UDT NMEA Format when possible
void nmea_harris (dsd_opts * opts, dsd_state * state, uint8_t * input, uint32_t src, int slot)
{

  //GPS working, Time Working (old method), COG and Speed are logical on due west sample
  //TODO: Fiddle/Test various ns and ew bits for proper signs in case its located elsewhere
  //UPDATE: Found two contiguous bits that always work on all my accumulated samples
  //the bits also place it with the 'lone' flagged bit observed prior

  uint8_t nmea_ns = input[88]; //north/south (lat sign) //72 previously used bit
  uint8_t nmea_ew = input[89]; //east/west (lon sign)  //88 previously used bit
  uint8_t nmea_q = input[147]; //Quality Indicator (last fix or current fix)
  // uint8_t nmea_speed = (uint8_t)ConvertBitIntoBytes(&input[129], 7); //129,7
  uint8_t nmea_speed = (uint8_t)ConvertBitIntoBytes(&input[129], 6); //this is one bit shorter than called for, but I believe it works

  //Latitude Bits
  uint8_t nmea_ndeg = (uint8_t)ConvertBitIntoBytes(&input[65], 7); //Latitude Degrees
  uint8_t nmea_nmin = (uint8_t)ConvertBitIntoBytes(&input[58], 6); //Latitude Minutes
  uint16_t nmea_nminf = (uint16_t)ConvertBitIntoBytes(&input[42], 14); //Latitude Fractions of Minutes

  //Longitude Bits
  uint8_t nmea_edeg = (uint8_t)ConvertBitIntoBytes(&input[96], 8); //Longitude Degrees
  uint8_t nmea_emin = (uint8_t)ConvertBitIntoBytes(&input[90], 6); //Longitude Minutes
  uint16_t nmea_eminf = (uint16_t)ConvertBitIntoBytes(&input[74], 14); //Longitude Fractions of Minutes

  //NOTE: Removed UTC hh.mm.ss6 format found in DMR UDT, could not make it work
  //with bits already established for time and it works extremely well as is

  //Course Over Ground in Degrees (0-360)
  uint16_t nmea_cog = (uint16_t)ConvertBitIntoBytes(&input[135], 9) % 360; //course over ground in degrees //working on due west sample without %360

  //timestamp (16-bit value w/ appended 17th bit to MSB)
  uint32_t rtime = (uint32_t)ConvertBitIntoBytes(&input[104], 16);
  rtime |= input[144] << 16; //this bit is flagged on after 0xFFFF rollover in the late afternoon
  uint32_t thour = rtime / 3600;
  uint32_t tmin  = (rtime % 3600) / 60;
  uint32_t tsec  = (rtime % 3600) % 60;

  //lat and lon conversion
  char deg_glyph[4];
  sprintf (deg_glyph, "%s", "°");
  float latitude = 0.0f;
  float longitude = 0.0f;
  float m_unit = 1.0f / 60.0f;
  float mm_unit = 1.0f / 600000.0f;

  //speed conversion
  float fmps, fmph, fkph = 0.0f; //conversion of knots to mps, kph, and mph values
  fmps = (float)nmea_speed * 0.514444; UNUSED(fmps);
  fmph = (float)nmea_speed * 1.15078f; UNUSED(fmph);
  fkph = (float)nmea_speed * 1.852f;   UNUSED(fkph);

  //calculate decimal representation of latidude and longitude (need some samples to test)
  latitude  = ( (float)nmea_ndeg + ((float)nmea_nmin*m_unit) + ((float)nmea_nminf*mm_unit) );
  longitude = ( (float)nmea_edeg + ((float)nmea_emin*m_unit) + ((float)nmea_eminf*mm_unit) );

  //This is opposite (seemingly) compared to the version specified by DMR UDT NMEA, could still be different bits involved (to be tested)
  // if (nmea_ns) latitude  *= -1.0f; //1 is South, 0 is North
  // if (nmea_ew) longitude *= -1.0f; //1 is West, 0 is East

  //DMR UDT NMEA original version (working now with new bit selection 88 and 89 like how DMR UDT NMEA format specifies)
  if (!nmea_ns) latitude  *= -1.0f; //0 is South, 1 is North
  if (!nmea_ew) longitude *= -1.0f; //0 is West, 1 is East

  fprintf (stderr, "\n");

  fprintf (stderr, " VCH: %d - SRC: %08d;", slot, src);
  fprintf (stderr, " GPS: %f%s, %f%s;", latitude, deg_glyph, longitude, deg_glyph);

  //Speed in Knots (assuming in knots, and not in mps, or kph)
  if (nmea_speed > 126)
    fprintf (stderr, " SPD > 126 knots or %f MPH;", fmph); //using MPH here, its Harris, they're in the U.S.
  else
    fprintf (stderr, " SPD: %d knots; %f MPH;", nmea_speed, fmph);

  //Course Over Ground (COG)
  fprintf (stderr, " COG: %03d%s;", nmea_cog, deg_glyph); //%360

  //debug speed
  // fprintf (stderr, " RSPD: %02X;", nmea_speed);

  //debug cog
  // fprintf (stderr, " RCOG: %03X;", nmea_cog >> 0);

  //UTC Time
  fprintf (stderr, " T: %02d:%02d:%02d UTC;", thour, tmin, tsec);

  //Fix Indicator
  if (nmea_q) fprintf (stderr, " Current Fix;");
  else fprintf (stderr, " Last Fix;");

  //save to ncurses string
  sprintf (state->dmr_embedded_gps[slot], "GPS: (%f%s, %f%s)", latitude, deg_glyph, longitude, deg_glyph);

  //save to LRRP report for mapping/logging
  FILE * pFile; //file pointer
  if (opts->lrrp_file_output == 1)
  {

    char * datestr = getDate();
    char * timestr = getTime();

    //rounded interger formats for the log report
    int s = (int)fkph;
    int a = nmea_cog;

    //open file by name that is supplied in the ncurses terminal, or cli
    pFile = fopen (opts->lrrp_out_file, "a");
    fprintf (pFile, "%s\t", datestr );
    fprintf (pFile, "%s\t", timestr ); //could switch to UTC time on PDU if desired
    fprintf (pFile, "%08d\t", src);
    fprintf (pFile, "%.6lf\t", latitude);
    fprintf (pFile, "%.6lf\t", longitude);
    fprintf (pFile, "%d\t ", s);
    fprintf (pFile, "%d\t ", a);
    fprintf (pFile, "\n");
    fclose (pFile);

    if (timestr != NULL)
    {
      free (timestr);
      timestr = NULL;
    }
    if (datestr != NULL)
    {
      free (datestr);
      datestr = NULL;
    }

  }

  //NOTE: There seems to be a few more octets left undecoded in this PDU
  //not including the CRC obviously, unsure of their meaning, could relate
  //to other GPS functions like precision, etc, unknown.

  //NOTE2: All these values are just best effort based on observation and 
  //making map points match using logic and distance over time for speed and course
  //without any documentation, any of these values may still be wrong

}

//fallback version (if desired/required)
void harris_gps(dsd_opts * opts, dsd_state * state, int slot, uint8_t * input)
{

  uint8_t lat_sign, lat_deg, lat_min = 0;
  uint8_t lon_sign, lon_deg, lon_min = 0;
  uint16_t lat_mmin = 0;
  uint16_t lon_mmin = 0;

  //potentially in this PDU, but unverifiable without documentation or reasonable 
  //mathematical proof vs map points and direction of travel / distance over time (assuming the radio is facing that direction)
  uint16_t rspeed = (uint16_t)ConvertBitIntoBytes(&input[136], 8); //MSB //136
  rspeed = (rspeed << 8) + (uint16_t)ConvertBitIntoBytes(&input[128], 8); //LSB //128

  //this works okay for example of driver driving due west at the speed limit of the road (fluke?)
  uint16_t rangle = (uint16_t)ConvertBitIntoBytes(&input[120], 8); //120,8
  float fspeed = (float)rspeed;
  float fangle = (float)rangle;

  fspeed /= 255.0f; //unit value of 1 bit
  fspeed *= 1.56f; //this is based on an observation of a driver moving approx 210 meters in about 8 seconds and this makes it just under the speed limit on the road there
  // fangle *= 360.0f/255.0f; //unit value of 1 bit
  fangle *= 2.0f; //may exceed 360 degrees as is (use %)

  int s, a = 0;
  s = (int)fspeed * 3.6;
  a = (int)fangle % 360;

  //fix and quality indicators?
  uint8_t fix = input[147]; UNUSED(fix);
  uint8_t quality = (uint16_t)ConvertBitIntoBytes(&input[148], 3); UNUSED(quality);

  //timestamp (16-bit value w/ appended 17th bit to MSB)
  uint32_t rtime = (uint32_t)ConvertBitIntoBytes(&input[104], 16); //seconds since midnight since last GPS fix (whatever the radio has as internal time)
  rtime |= input[144] << 16; //test appending this as bit 17 (observed this bit set after the afternoon 0xFFFF rollover)
  uint32_t thour = rtime / 3600;
  uint32_t tmin  = (rtime % 3600) / 60;
  uint32_t tsec  = (rtime % 3600) % 60;

  float lat_dec = 0.0f;
  float lon_dec = 0.0f;

  char deg_glyph[4];
  sprintf (deg_glyph, "%s", "°");

  //This appears to be similar to the NMEA GPGGA format (DDmm.mm) but 
  //octets are ordered in least significant to most significant value
  lat_mmin = (uint16_t)ConvertBitIntoBytes(&input[40], 16); //? bits required, but grabbing two octets
  lat_min  = (uint8_t)ConvertBitIntoBytes(&input[58], 6);  //6 bits required to get 60
  lat_deg  = (uint8_t)ConvertBitIntoBytes(&input[65], 7); //7 bits required to get 90
  lat_sign = input[72]; //64

  lon_mmin = (uint16_t)ConvertBitIntoBytes(&input[72], 16); //? bits required, but grabbing two octets
  lon_min  = (uint8_t)ConvertBitIntoBytes(&input[90], 6);  //6 bits required to get 60
  lon_deg  = (uint8_t)ConvertBitIntoBytes(&input[96], 8); //8 bits required to get 180 
  lon_sign = input[88]; //88, unsure of a correct location, but on the sample with 0 minutes, this lonely bit was flagged on

  int src = 0;
  if (slot == 0) src = state->lastsrc;
  if (slot == 1) src = state->lastsrcR;

  //calculate decimal representation (was a pain to figure out the sub minute values)
  lat_dec = ( (float)lat_deg + ((float)lat_min/60.0f) + ((float)lat_mmin/600000.0f) );
  lon_dec = ( (float)lon_deg + ((float)lon_min/60.0f) + ((float)lon_mmin/600000.0f) );

  if (lat_sign)
    lat_dec *= -1.0f;

  if (lon_sign) 
    lon_dec *= -1.0f;

  //line break
  fprintf (stderr, "\n");
  fprintf (stderr, " GPS: %f%s, %f%s;", lat_dec, deg_glyph, lon_dec, deg_glyph);

  //gps fix time
  // fprintf (stderr, " RT: %05X;", rtime); //debug
  fprintf (stderr, " LTS: %02d:%02d:%02d UTC;", thour, tmin, tsec);  //last time synced to GPS

  //speed and direction
  // fprintf (stderr, " RSPD: %04X;", rspeed); //debug
  // fprintf (stderr, " RANG: %02X;", rangle); //debug
  // fprintf (stderr, " MPS: %06.02f;", fspeed);
  // fprintf (stderr, " KPH: %06.02f;", fspeed * 3.6f);
  // fprintf (stderr, " MPH: %06.02f;", fspeed * 2.2369f);
  fprintf (stderr, " DIR: %03d%s;", a, deg_glyph);

  // fprintf (stderr, " FIX: %d", fix);
  // fprintf (stderr, " QLT: %02d", quality);

  // fprintf (stderr, " SRC: %08d;", src);

  //save to array for ncurses
  sprintf (state->dmr_embedded_gps[slot], "GPS: (%f%s, %f%s)", lat_dec, deg_glyph, lon_dec, deg_glyph);

  //save to LRRP report for mapping/logging
  FILE * pFile; //file pointer
  if (opts->lrrp_file_output == 1)
  {

    char * datestr = getDate();
    char * timestr = getTime();

    //open file by name that is supplied in the ncurses terminal, or cli
    pFile = fopen (opts->lrrp_out_file, "a");
    fprintf (pFile, "%s\t", datestr );
    fprintf (pFile, "%s\t", timestr );
    fprintf (pFile, "%08d\t", src);
    fprintf (pFile, "%.6lf\t", lat_dec);
    fprintf (pFile, "%.6lf\t", lon_dec);
    fprintf (pFile, "%d\t ", s);
    fprintf (pFile, "%d\t ", a);
    fprintf (pFile, "\n");
    fclose (pFile);

    if (timestr != NULL)
    {
      free (timestr);
      timestr = NULL;
    }
    if (datestr != NULL)
    {
      free (datestr);
      datestr = NULL;
    }

  }

  //NOTE: Thanks to DSheirer (SDRTrunk) for helping me work out a few of the things in here
  //not entirely convinced on some of these calcs (speed, angle, and TS) but sure these bits
  //are actual speed/direction/timestamps judging from what the coordinates show on a map
}

//externalize embedded GPS - Confirmed working now on NE, NW, SE, and SW coordinates
void dmr_embedded_gps (dsd_opts * opts, dsd_state * state, uint8_t lc_bits[])
{
  UNUSED(opts);

  fprintf (stderr, "%s", KYEL);
  fprintf (stderr, " Embedded GPS:");
  uint8_t slot = state->currentslot;
  uint8_t pf = lc_bits[0];
  uint8_t res_a = lc_bits[1];
  uint8_t res_b = (uint8_t)ConvertBitIntoBytes(&lc_bits[16], 4);
  uint8_t pos_err = (uint8_t)ConvertBitIntoBytes(&lc_bits[20], 3);
  UNUSED2(res_a, res_b);

  char deg_glyph[4];
  sprintf (deg_glyph, "%s", "°");

  uint32_t lon_sign = lc_bits[23]; 
  uint32_t lon = (uint32_t)ConvertBitIntoBytes(&lc_bits[24], 24); 
  uint32_t lat_sign = lc_bits[48]; 
  uint32_t lat = (uint32_t)ConvertBitIntoBytes(&lc_bits[49], 23); 
  double lon_sf = 1.0f; //float value we can multiple longitude with
  double lat_sf = 1.0f; //float value we can multiple latitude with

  double lat_unit = (double)180/ pow (2.0, 24); //180 divided by 2^24
  double lon_unit = (double)360/ pow (2.0, 25); //360 divided by 2^25

  char latstr[3];
  char lonstr[3];
  sprintf (latstr, "%s", "N");
  sprintf (lonstr, "%s", "E");

  //run calculations and print
  //7.2.16 and 7.2.17 (two's compliment)

  double latitude = 0;  
  double longitude = 0; 

  if (pf) fprintf (stderr, " Protected");
  else
  {
    if (lat_sign)
    {
      lat = 0x800001 - lat;
      sprintf (latstr, "%s", "S");
      lat_sf = -1.0f;
    } 
    latitude = ((double)lat * lat_unit);

    if (lon_sign)
    {
      lon = 0x1000001 - lon;
      sprintf (lonstr, "%s", "W");
      lon_sf = -1.0f;
    } 
    longitude = ((double)lon * lon_unit);

    //sanity check
    if (abs (latitude) < 90 && abs(longitude) < 180)
    {
      fprintf (stderr, " Lat: %.5lf%s%s Lon: %.5lf%s%s (%.5lf, %.5lf)", latitude, deg_glyph, latstr, longitude, deg_glyph, lonstr, lat_sf * latitude, lon_sf * longitude);

      //7.2.15 Position Error
      uint16_t position_error = 2 * pow(10, pos_err); //2 * 10^pos_err 
      if (pos_err == 0x7 ) fprintf (stderr, "\n  Position Error: Unknown or Invalid");
      else fprintf (stderr, "\n  Position Error: Less than %dm", position_error);

      //save to array for ncurses
      if (pos_err != 0x7)
      {
        sprintf (state->dmr_embedded_gps[slot], "GPS: %.5lf%s%s %.5lf%s%s Err: %dm", latitude, deg_glyph, latstr, longitude, deg_glyph, lonstr, position_error);
      }
      else sprintf (state->dmr_embedded_gps[slot], "GPS: %.5lf%s%s %.5lf%s%s Unknown Pos Err", latitude, deg_glyph, latstr, longitude, deg_glyph, lonstr);

      //save to LRRP report for mapping/logging
      FILE * pFile; //file pointer
      if (opts->lrrp_file_output == 1)
      {

        char * datestr = getDate();
        char * timestr = getTime();

        int src = 0;
        if (slot == 0) src = state->lasttg;
        if (slot == 1) src = state->lasttgR;

        //open file by name that is supplied in the ncurses terminal, or cli
        pFile = fopen (opts->lrrp_out_file, "a");
        fprintf (pFile, "%s\t", datestr );
        fprintf (pFile, "%s\t", timestr );
        fprintf (pFile, "%08d\t", src);
        fprintf (pFile, "%.5lf\t", latitude);
        fprintf (pFile, "%.5lf\t", longitude);
        fprintf (pFile, "0\t " ); //zero for velocity
        fprintf (pFile, "0\t " ); //zero for azimuth
        fprintf (pFile, "\n");
        fclose (pFile);

      }
    }
  }

  fprintf (stderr, "%s", KNRM);
}

//This Function has been redone and variables fixed to mirror SDRTrunk 
//my earlier assumption that this was the same format as DMR Embedded GPS was incorrect
void apx_embedded_gps (dsd_opts * opts, dsd_state * state, uint8_t lc_bits[])
{

  fprintf (stderr, "%s", KYEL);
  fprintf (stderr, " GPS:");
  uint8_t slot = state->currentslot;
  uint8_t pf = lc_bits[0];
  uint8_t res_a = lc_bits[1];
  uint8_t res_b = (uint8_t)ConvertBitIntoBytes(&lc_bits[16], 7);
  uint8_t expired = lc_bits[23]; //this bit seems to indicate that the GPS coordinates are out of date or fresh

  char deg_glyph[4];
  sprintf (deg_glyph, "%s", "°");

  uint32_t lon_sign = lc_bits[48]; 
  uint32_t lon = (uint32_t)ConvertBitIntoBytes(&lc_bits[49], 23); 
  uint32_t lat_sign = lc_bits[24];
  uint32_t lat = (uint32_t)ConvertBitIntoBytes(&lc_bits[25], 23); 
  //todo: acquire more samples for additional validation
  double lat_unit = 90.0f / 0x7FFFFF;
  double lon_unit = 180.0f / 0x7FFFFF;
  double lon_sf = 1.0f; //float value we can multiple longitude with
  double lat_sf = 1.0f; //float value we can multiple latitude with

  char latstr[3];
  char lonstr[3];
  char valid[9];
  sprintf (latstr, "%s", "N");
  sprintf (lonstr, "%s", "E");
  sprintf (valid, "%s", "Current");

  double latitude = 0;  
  double longitude = 0; 

  if (pf) fprintf (stderr, " Protected");
  else
  {
    if (lat_sign)
    {
      sprintf (latstr, "%s", "S");
      lat_sf = -1.0f;
    }
    latitude = ((double)lat * lat_unit);

    if (lon_sign)
    {
      sprintf (lonstr, "%s", "W");
      lon_sf = -1.0f;
    }
    longitude = ((double)lon * lon_unit);

    //sanity check
    if (abs (latitude) < 90 && abs(longitude) < 180)
    {
      fprintf (stderr, " Lat: %.5lf%s%s Lon: %.5lf%s%s (%.5lf, %.5lf) ", latitude, deg_glyph, latstr, longitude, deg_glyph, lonstr, latitude * lat_sf, longitude * lon_sf);

      if (expired)
      {
        fprintf (stderr, "Expired; ");
        sprintf (valid, "%s", "Expired");
      }
      else if (!expired)
        fprintf (stderr, "Current; ");

      if (res_a)
        fprintf (stderr, "RES_A: %d; ", res_a);

      if (res_b)
        fprintf (stderr, "RES_B: %02X; ", res_b);

      //save to array for ncurses
      sprintf (state->dmr_embedded_gps[slot], "GPS: %lf%s%s %lf%s%s %s", latitude, deg_glyph, latstr, longitude, deg_glyph, lonstr, valid);

      //save to LRRP report for mapping/logging
      FILE * pFile; //file pointer
      if (opts->lrrp_file_output == 1)
      {
        int src = 0;
        if (slot == 0) src = state->lastsrc;
        if (slot == 1) src = state->lastsrcR;

        char * datestr = getDate();
        char * timestr = getTime();

        //open file by name that is supplied in the ncurses terminal, or cli
        pFile = fopen (opts->lrrp_out_file, "a");
        fprintf (pFile, "%s\t", datestr );
        fprintf (pFile, "%s\t", timestr );
        fprintf (pFile, "%08d\t", src);
        fprintf (pFile, "%.5lf\t", latitude);
        fprintf (pFile, "%.5lf\t", longitude);
        fprintf (pFile, "0\t " ); //zero for velocity
        fprintf (pFile, "0\t " ); //zero for azimuth
        fprintf (pFile, "\n");
        fclose (pFile);

        if (timestr != NULL)
        {
          free (timestr);
          timestr = NULL;
        }
        if (datestr != NULL)
        {
          free (datestr);
          datestr = NULL;
        }

      }

    }
  }

  fprintf (stderr, "%s", KNRM);
}

//relocate all below later on
uint8_t alias_lut[256] = { 
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

void apx_embedded_alias_test (dsd_opts * opts, dsd_state * state)
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

void apx_embedded_alias_header (dsd_opts * opts, dsd_state * state, uint8_t slot, uint8_t lc_bits[])
{
  UNUSED(opts);
  uint8_t ta_len = (uint8_t)ConvertBitIntoBytes(&lc_bits[32], 8); //len in blocks of associated talker alias
  fprintf (stderr, " Block Len: %d;", ta_len);

  //use dmr_pdu_sf for storage, store entire header (will be used to verify complete reception of full alias)
  memset (state->dmr_pdu_sf[slot], 0, sizeof (state->dmr_pdu_sf[slot])); //reset storage for header and blocks
  memcpy(state->dmr_pdu_sf[slot], lc_bits, 72*sizeof(uint8_t));

}

void apx_embedded_alias_blocks (dsd_opts * opts, dsd_state * state, uint8_t slot, uint8_t lc_bits[])
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

      //debug, let's look at byte, bit counts and see if we can find this in the completed dump
      // fprintf (stderr, " Bits: %X; Bytes: %X; Mod: %d; ", bn*44, (bn*44)/8, (bn*44)%8); //the portion with 0100 is 8 off from 0108 (hex) bits

      //debug, dump completed data set
      // fprintf (stderr, "\n Full: ");
      // for (uint8_t i = 0; i < ((bn*11)+18); i++) //double check on other bn values
      //   fprintf (stderr, "%X", (uint8_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][0+(i*4)], 4));
      // fprintf (stderr, "\n");

      //extract fully qualified SUID
      uint32_t wacn = (uint32_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][72], 20);
      uint32_t sys  = (uint32_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][92], 12);
      uint32_t rid  = (uint32_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][104], 24);

      //print fully qualified SUID
      fprintf (stderr, "\n WACN: %05X; SYS: %03X; RID: %d;", wacn, sys, rid);

      //extract CRC
      uint16_t crc_ext = (uint16_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][(72+(bn*44)-16)], 16);

      //compute CRC
      uint16_t crc_cmp = ComputeCrcCCITT16d(&state->dmr_pdu_sf[slot][72], (bn*44)-16);

      //print comparison
      // fprintf (stderr, " CRC EXT: %04X CMP: %04X;", crc_ext, crc_cmp);
      if (crc_ext != crc_cmp)
        fprintf (stderr, " CRC Error;");
      // else fprintf (stderr, " Okay;");

      //start decoding the alias
      if (crc_ext == crc_cmp)
      {
        //WIP: Working, needs more samples to verify various len values
        uint16_t ptr = 128; //starting point of encoded data from test vectors
        uint8_t encoded[200]; memset(encoded, 0, sizeof(encoded));
        uint8_t decoded[200]; memset(decoded, 0, sizeof(decoded));
        uint16_t num_bytes = ((44*bn) / 8) - 9; //substract 2 CRC and 7 FQSUID 

        for (int16_t i = 0; i < num_bytes; i++)
        {
          encoded[i] = (uint8_t)ConvertBitIntoBytes(&state->dmr_pdu_sf[slot][ptr], 8);
          ptr += 8;
        }

        uint16_t accumulator = num_bytes;

        //Ilya's Voodoo Code
        for (int16_t i = 0; i < num_bytes; i++)
        {
          // Multiplication step 1
          uint16_t accum_mult = accumulator * 293 + 0x72E9;

          // Lookup table step
          uint8_t lut = alias_lut[encoded[i]];
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
        // for (int16_t i = 0; i < bytes; i++)
        //   fprintf (stderr, "%02X", encoded[i]);

        // //debug, dump the decoded payload as hex octets
        // fprintf (stderr, "\n Decoded: ");
        // for (int16_t i = 0; i < bytes; i++)
        //   fprintf (stderr, "%02X", decoded[i]);

        //dump the decoded payload as long chars
        // fprintf (stderr, "\n");
        fprintf (stderr, " Alias: ");
        for (int16_t i = 0; i < num_bytes/2; i++)
          fprintf (stderr, "%lc", ((decoded[(i*2)+0])<<8) | ((decoded[(i*2)+1])<<0) );

        //For Ncurses Display (needs to be testing on real samples first to make sure this is setup right)
        // memcpy(state->dmr_embedded_gps[slot], decoded, bytes*sizeof(uint8_t));
        // state->dmr_embedded_gps[slot][199] = '\0'; //terminate string

      }

      //clear out now stale storage
      memset (state->dmr_pdu_sf[slot], 0, sizeof (state->dmr_pdu_sf[slot]));

    }
  }
}