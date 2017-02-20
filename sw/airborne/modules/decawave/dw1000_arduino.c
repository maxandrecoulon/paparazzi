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

#include "modules/decawave/dw1000_arduino.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/abi.h"

static void dw1000_arduino_parse(uint8_t c)
{
  (void) c;
  // TODO parse frame here
  // When a complete message is found, compute position and send it over ABI (as a GPS message ?)
}

void dw1000_arduino_init()
{
  // TODO Init variables here if needed
}

void dw1000_arduino_report()
{
  // TODO Can be used to send debug information over telemetry
}

void dw1000_arduino_event()
{
  // Look for data on serial link and send to parser
  while (uart_char_available(&(DW1000_ARDUINO_DEV))) {
    uint8_t ch = uart_getch(&(DW1000_ARDUINO_DEV));
    dw1000_arduino_parse(ch);
  }
}


