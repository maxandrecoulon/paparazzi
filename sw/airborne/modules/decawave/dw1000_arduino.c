/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/decawave/dw1000_arduino.c"
 * @author Gautier Hattenberger
 * Driver to get ranging data from Decawave DW1000 modules connected to Arduino
 */


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/abi.h"
#include "modules/decawave/dw1000_arduino.h"
#include "modules/decawave/trilateration.h"
#include "subsystems/gps.h"
#include "state.h"
#include "generated/flight_plan.h"



/** default sonar */
#ifndef AGL_DIST_SONAR_ID
#define AGL_DIST_SONAR_ID ABI_BROADCAST
#endif

abi_event sonar_ev;
float sonar_value;
float psi;

/* Definition des etats de notre machine a etats */

#define WS 0
#define RB 1
#define CK 2
#define NB 6
#define NB_ELEM 10


/* Definition des structures Ancres (contenant les distances des ancres au tag) et Machine a etats */

typedef struct Anchor Anchor;
struct Anchor {
    float distances1[NB_ELEM][2]; //tableau de buffers tournants contenant sur chaque ligne une distance, un temps d'emission
    float distances2[NB_ELEM][2];
    float distances3[NB_ELEM][2];
    float filtered_distances[3];
    int index[3]; //tableau des trois index contenant les indices courants des buffers tournants
    uint16_t ids[3];
    bool new_dist;
};

static Anchor anchor;

typedef struct Machine_state Machine_state;
struct Machine_state {
    uint8_t current_state;
    uint8_t i;
    uint8_t buffer[NB];
};

struct GpsState gps_dw1000;

struct LtpDef_i ltp_def;


/* Fonctions de conversion entre float, uint16 et tableaux d'uint8 */
static float float_from_buf(uint8_t* b) {
  union {
    float f;
    uint8_t t[4];
  } r;
  memcpy (r.t, b, sizeof(float));
  return r.f;
}

static uint16_t uint16_from_buf(uint8_t* b) {
  union {
    uint16_t f;
    uint8_t t[2];
  } r;
  memcpy (r.t, b, sizeof(uint16_t));
  return r.f;
}

static uint8_t* float_to_buf(float f, uint8_t* b) {
  memcpy (b, (uint8_t*)(&f), sizeof(float));
  return b;
}

/* Fonction de remplissage du tableau des distances ancres-tag */
static void fill_anchor(uint8_t *buffer, Anchor* _anchor) {

    if (uint16_from_buf(buffer) == _anchor->ids[0]) {
        _anchor->index[0] = (_anchor->index[0] + 1) % NB_ELEM;
        _anchor->distances1[_anchor->index[0]][0] = float_from_buf(buffer+2);
        _anchor->distances1[_anchor->index[0]][1] = get_sys_time_float();
    }

    else if (uint16_from_buf(buffer) == _anchor->ids[1]) {
        _anchor->index[1] = (_anchor->index[1] + 1) % NB_ELEM;
        _anchor->distances2[_anchor->index[0]][0] = float_from_buf(buffer+2);
        _anchor->distances2[_anchor->index[0]][1] = get_sys_time_float();
    }

    else if (uint16_from_buf(buffer) == _anchor->ids[2]) {
        _anchor->index[2] = (_anchor->index[2] + 1) % NB_ELEM;
        _anchor->distances3[_anchor->index[0]][0] = float_from_buf(buffer+2);
        _anchor->distances3[_anchor->index[0]][1] = get_sys_time_float();
    }

    else printf("Error ID\n");
}

/* Fonction de lecture et de mise en place de la machine a etats */
static void dw1000_arduino_parse(uint8_t c, Machine_state *machine_state, Anchor* _anchor) {


  switch (machine_state->current_state) {

      uint8_t checksum;

      case WS:
      /* First state : Waiting Synchro */

        if (c == 0xfe) {
            machine_state->i = 0;
            machine_state->current_state = RB;
        }
        break;


      case RB:
      /* Second state : Read Bytes */

        machine_state->buffer[machine_state->i++] = c;

        if (machine_state->i == NB) {
            machine_state->current_state = CK;
        }
        break;

      case CK:
      /* Third state : Checksum */

        checksum = 0x00;

        for (int j=0; j<NB; j++) {
            checksum += machine_state->buffer[j];
        }

        if (checksum == c) {
            fill_anchor(machine_state->buffer, _anchor);
            _anchor->new_dist = true;
        }

        else printf("Error checksum\n");

        machine_state->current_state = WS;
        break;
  }
}


static void filter_distances(Anchor *_anchor) {
  float filtered_distance1 = 0;
  float filtered_distance2 = 0;
  float filtered_distance3 = 0;

  for (int i=0; i<NB_ELEM; i++) {
    filtered_distance1 += _anchor->distances1[i][0];
    filtered_distance2 += _anchor->distances2[i][0];
    filtered_distance3 += _anchor->distances3[i][0];
  }
  
  _anchor->filtered_distances[0] = filtered_distance1/NB_ELEM;
  _anchor->filtered_distances[1] = filtered_distance2/NB_ELEM;
  _anchor->filtered_distances[2] = filtered_distance3/NB_ELEM;
}

static void sonar_cb(uint8_t __attribute__((unused)) sender_id, float distance) {
  sonar_value = distance;
}

/* Fonction d'envoi de message de communication interne */
static void parse_gps_dw1000_small(float x, float y, float z) {

  struct EnuCoor_i enu_pos;
  enu_pos.x = (int32_t) (x * 100);
  enu_pos.y = (int32_t) (y * 100);
  enu_pos.z = (int32_t) (z * 100);

  // Rotation
  // on doit faire tourner notre repere local de l'angle psi initialise dans le plan de vol
  enu_pos.x = enu_pos.x * cos(psi) - enu_pos.y * sin(psi);
  enu_pos.y = enu_pos.x * sin(psi) + enu_pos.y * cos(psi);

  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&gps_dw1000.ecef_pos, &ltp_def, &enu_pos);
  SetBit(gps_dw1000.valid_fields, GPS_VALID_POS_ECEF_BIT);

  lla_of_ecef_i(&gps_dw1000.lla_pos, &gps_dw1000.ecef_pos);
  SetBit(gps_dw1000.valid_fields, GPS_VALID_POS_LLA_BIT);

 
  gps_dw1000.hmsl = ltp_def.hmsl + enu_pos.z * 10; //séparation modèle géoide et ellipsoide
  SetBit(gps_dw1000.valid_fields, GPS_VALID_HMSL_BIT);


  gps_dw1000.num_sv = 7;
  gps_dw1000.tow = get_sys_time_msec();
  gps_dw1000.fix = GPS_FIX_3D; // set 3D fix to true

  // set gps msg time
  gps_dw1000.last_msg_ticks = sys_time.nb_sec_rem;
  gps_dw1000.last_msg_time = sys_time.nb_sec;

  gps_dw1000.last_3dfix_ticks = sys_time.nb_sec_rem;
  gps_dw1000.last_3dfix_time = sys_time.nb_sec;

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgGPS(GPS_DW1000_ID, now_ts, &gps_dw1000);
}

void dw1000_arduino_init() {

  //anchor structure init
  for(int i=0; i<NB_ELEM; i++) {
  	anchor.distances1[i][0] = 0;
  	anchor.distances1[i][1] = 0;
  	anchor.distances2[i][0] = 0;
  	anchor.distances2[i][1] = 0;
  	anchor.distances3[i][0] = 0;
  	anchor.distances3[i][1] = 0;
  }

  anchor.filtered_distances[0] = 0;
  anchor.filtered_distances[1] = 0;
  anchor.filtered_distances[2] = 0;

  anchor.index[0] = 0;
  anchor.index[1] = 0;
  anchor.index[2] = 0;

  anchor.ids[0] = 1;
  anchor.ids[1] = 2;
  anchor.ids[2] = 3;

  anchor.new_dist = false;

  trilateration_init();
  //sonar init
  sonar_value = 0.;
  AbiBindMsgAGL(AGL_DIST_SONAR_ID, &sonar_ev, sonar_cb);

  //heading init
  psi = stateGetNedToBodyEulers_f()->psi;

  //gps structure init
  gps_dw1000.fix = GPS_FIX_NONE;
  gps_dw1000.pdop = 0;
  gps_dw1000.sacc = 0;
  gps_dw1000.pacc = 0; //erreur de position en cm !!!!!!!!!!!!!!!!!!!
  gps_dw1000.cacc = 0;

  gps_dw1000.comp_id = GPS_DW1000_ID;

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  ltp_def_from_lla_i(&ltp_def, &llh_nav0);
}

void dw1000_arduino_report() {
  // TODO Can be used to send debug information over telemetry
}

void dw1000_arduino_event() {

  struct EnuCoor_f pos;
  Machine_state machine_state = {WS, 0, {}};
  uint8_t ch;

  // Look for data on serial link and send to parser
  while (uart_char_available(&"evvze")) {
    ch = uart_getch(&"zz");
    dw1000_arduino_parse(ch, &machine_state, &anchor);
    if (anchor.new_dist) {
      filter_distances(&anchor);
      pos = trilateration(anchor.filtered_distances, sonar_value); // vecteur de position (x, y)
      parse_gps_dw1000_small(pos.x, pos.y, pos.z);
      anchor.new_dist = false;
    }
  }
}