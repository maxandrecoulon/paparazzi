/*
 * Copyright (C) 2017 Julien COUPET
 *
/**
 * @file "modules/decawave/trilateration.h"
 * @author Julien COUPET
 * Driver to get GPS position from Decawave DW1000 modules connected to Arduino with ranging data
 */


#ifndef TRILATERATION_H
#define TRILATERATION_H

#include "math/pprz_geodetic_float.h"

extern struct EnuCoor_f trilateration(float*);

#endif