//
//  main.c
//  Trilateration
//
//  Created by Benjamin BERNARD on 10/03/2017.
//  Copyright © 2017 Benjamin BERNARD. All rights reserved.
//

#include <stdio.h>
#include <math.h>
#include "math/pprz_simple_matrix.h"
#include "trilateration.h"


// Calcule l'angle au niveau de la balise n°1 à partir de la formule d'Al-Kashi
static float alKashi_angle1(double _1_2, double _1_3, double _2_3) {
    return (float)acosf((_1_3*_1_3 + _1_2*_1_2 - _2_3*_2_3) / (_1_3*_1_2));
}

static float projection(float r, float z) {
    return sqrtf(r*r - z*z);
}


/* Distance entre les ancres */
float DISTANCE_ANCHOR_1_2; //a recuperer une seule fois au debut de la mission
float DISTANCE_ANCHOR_1_3;
float DISTANCE_ANCHOR_2_3;

float angle1 = 0.f;


/* Positions des ancres */
float POSITION1[2];
float POSITION2[2];
float POSITION3[2];


static void MAT_INV22(float _invS[2][2],float _S[2][2]) {
    float det = _S[0][0]*_S[1][1] - _S[1][0]*_S[0][1];
    if(det == 0)
    {
        printf("value determinant error\n");
    }
    else
    {
        _invS[0][0] = (1/det)*_S[1][1];
        _invS[0][1] = -(1/det)*_S[0][1];
        _invS[1][0] = -(1/det)*_S[1][0];
        _invS[1][1] = (1/det)*_S[0][0];
    }
}

// C = A*B   A:(i,k) B:(k,j) C:(i,j)
static void MAT_MUL(int _i,int _k,int _j,float C[2][2],float A[2][2],float B[2][2]) {
    int l,c,m;
    for (l=0; l<_i; l++)
            for (c=0; c<_j; c++)
            {
                C[l][c] = 0.;
                    for (m=0; m<_k; m++)
                        C[l][c] += A[l][m]*B[m][c];
            }
}

void trilateration_init(void) {
    angle1 = alKashi_angle1((double)DISTANCE_ANCHOR_1_2, (double)DISTANCE_ANCHOR_1_3, (double)DISTANCE_ANCHOR_2_3);

    POSITION1[2] = {0, 0};
    POSITION2[2] = {DISTANCE_ANCHOR_1_2, 0};
    POSITION3[2] = {DISTANCE_ANCHOR_1_3 * cos(angle1), DISTANCE_ANCHOR_1_3 * sin(angle1)};
}


struct EnuCoor_f trilateration(float* dist, float z) {

    float r1 = dist[0];
    float r2 = dist[1];
    float r3 = dist[2];


    float A[2][2] = {{POSITION1[0] - POSITION2[0], POSITION1[1] - POSITION2[1]},
        {POSITION1[0] - POSITION3[0], POSITION1[1] - POSITION3[1]}};  //initialisation de la matrice A
    
    float A_t[2][2] = {{POSITION1[0] - POSITION2[0], POSITION1[0] - POSITION3[0]},
        {POSITION1[1] - POSITION2[1], POSITION1[1] - POSITION3[1]}};  //initialisation de la matrice A
    
    float b[2][1] = {{0.5*(pow(POSITION1[0],2)+pow(POSITION1[1],2) - pow(POSITION2[0],2)+pow(POSITION2[1],2) + pow(projection(r2,z),2) - 
                    pow(projection(r1,z),2))},{0.5*(pow(POSITION1[0],2)+pow(POSITION1[1],2) - pow(POSITION3[0],2)+pow(POSITION3[1],2) + 
                    pow(projection(r3,z),2) - pow(projection(r1,z),2))}};
    

    float A_tA[2][2];
    float _invA_tA[2][2];
    float C[2][2];
    float X[2][1];
    
    
    MAT_MUL(2, 2, 2, A_tA, A_t, A);
    MAT_INV33(_invA_tA, A_tA);
    MAT_MUL(2, 2, 2, C, _invA_tA, A_t);
    X[0][0] = C[0][0]*b[0][0]+C[0][1]*b[1][0];
    X[1][0] = C[1][0]*b[0][0]+C[1][1]*b[1][0];

    struct EnuCoor_f enu;
    enu.x = X[0][0];
    enu.y = X[1][0];
    enu.z =  z;

    return enu;
}


