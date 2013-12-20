/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** Author: Neil Dantam
 */

#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <ach.h>
#include <amino.hpp>
#include "piranha.h"

using namespace amino;

static int is_init = 0;
static DualQuat S0[2];
static Quat r_ft_rel;

static void kin_init(void) {

    // Arm
    {
        RotMat R0l(0, 0, -1,
                   1, 0,  0,
                   0,-1,  0);
        assert( aa_tf_isrotmat( R0l.data ) );
        double x = LWA4_L_0 + PIR_L_SHOULDER_WIDTH/2 - LWA4_L_P;

        S0[PIR_LEFT] = DualQuat( Quat(R0l),
                                 Vec3(0, x,  0) );
        RotMat R0r(0, 0, 1,
                   -1, 0,  0,
                   0, -1,  0);
        assert( aa_tf_isrotmat( R0r.data ) );
        S0[PIR_RIGHT] = DualQuat( Quat(R0r),
                                  Vec3(0, -x,  0) );


        /* DualQuat Srel( Quat(AxisAngle( 1,0,0, M_PI )), */
        /*                       Vec3(0,0,0) ); */

        /* S0[PIR_RIGHT] = Srel * S0[PIR_LEFT]; */
        /* AxisAngle A0(S0[PIR_LEFT].real); */
        /* printf("arm: "); */
        /* aa_dump_vec( stdout, A0.data, 4 ); */
    }

    // F/T rotation
    {
        RotMat R0( 0, 0, 1,
                   1, 0, 0,
                   0, 1, 0 );
        assert(aa_tf_isrotmat(R0.data));

        AxisAngle R_rel( 0,0,1, LWA4_FT_ANGLE );

        r_ft_rel = Quat(R0) * Quat(R_rel);
    }


    double xl[3], xr[3];
    aa_tf_duqu_trans( S0[PIR_LEFT].data, xl );
    aa_tf_duqu_trans( S0[PIR_RIGHT].data, xr );
    printf("S0 left:  " ); aa_dump_vec( stdout, xl, 3 );
    printf("S0 right: " ); aa_dump_vec( stdout, xr, 3 );

    is_init = 1;
}






static const double lwa4_axis[][3] = {
    {-1,0,0},   /* 01 */
    {0,-1,0},  /* 12 */
    {-1,0,0},  /* 23 */
    {0,-1,0},  /* 34 */
    {-1,0,0},  /* 45 */
    {0,1,0}, /* 56 */
    {1,0,0}  /* 67 */
};

static const double lwa4_trans[][3] = {
    {0,0,0},   /* 01 */
    {0,0,0},  /* 12 */
    {0,0,0},  /* 23 */
    {LWA4_L_1,0,0},  /* 34 */
    {LWA4_L_2,0,0},  /* 45 */
    {0,0,0}, /* 56 */
    {0,0,0}  /* 67 */
};

void lwa4_duqu( const double *q, double *S_rel ) {
    /* aa_tf_xxyz2duqu( -q[0],        0, 0, 0, S_rel + 0*8 ); */
    /* aa_tf_yxyz2duqu( -q[1],        0, 0, 0, S_rel + 1*8); */
    /* aa_tf_xxyz2duqu( -q[2],        0, 0, 0, S_rel + 2*8); */
    /* aa_tf_yxyz2duqu( -q[3], LWA4_L_1, 0, 0, S_rel + 3*8); */
    /* aa_tf_xxyz2duqu( -q[4], LWA4_L_2, 0, 0, S_rel + 4*8); */
    /* aa_tf_yxyz2duqu(  q[5],        0, 0, 0, S_rel + 5*8); */
    /* //aa_tf_xxyz2duqu(  q[6], LWA4_L_e, 0, 0, S_rel + 6*8); */
    /* aa_tf_xxyz2duqu(  q[6],        0, 0, 0, S_rel + 6*8); */

    /* Fill in S_rel with the relative dual quaternions */
    for( size_t i = 0; i < 7; i ++ ) {
        new (S_rel+i*8) DualQuat( AxisAngle(lwa4_axis[i], q[i]),
                                  Vec3(lwa4_trans[i]) );
    }


}

void lwa4_kin_duqu( const double *q, const double S0[8], const double See[8], double S[8], double *J ) {

    double S_rel[8*7];
    lwa4_duqu(q, S_rel );

    double S_abs[8*7];
    if( J ) {
        rfx_kin_duqu_revchain( 7, S0, S_rel, See, lwa4_axis[0], S_abs, S, J, 6 );
    } else {

        rfx_kin_duqu_chain( 7, S0, S_rel, S_abs );
        aa_tf_duqu_mul( S_abs+8*6, See, S );
    }
}


int pir_kin_arm( struct pir_state *X ) {

    if( !is_init) kin_init();

    /* double qq[2][7] = { {-M_PI_2,M_PI_2,0,0,0,0,0}, */
    /*                     {M_PI_2,-M_PI_2,0,0,0,0,0} }; */

    /* for( size_t i = 0; i < 2; i ++ ) { */
    /*     int j, k; */
    /*     PIR_SIDE_INDICES(i, j, k); */
    /*     lwa4_kin_duqu( &X->q[j], S0[i].data, aa_tf_duqu_ident, */
    /*                    X->S_wp[i], X->J_wp[i]  ); */
    /* } */
    /* double xl[3], xr[3]; */
    /* aa_tf_duqu_trans( X->S_wp[PIR_LEFT], xl ); */
    /* aa_tf_duqu_trans( X->S_wp[PIR_RIGHT], xr ); */
    /* printf("left:  " ); aa_dump_vec( stdout, xl, 3 ); */
    /* printf("right: " ); aa_dump_vec( stdout, xr, 3 ); */


    for( size_t i = 0; i < 2; i ++ ) {
        int j, k;
        PIR_SIDE_INDICES(i, j, k);
        /*-- Arm --*/
        //printf("%d: ", i ); aa_dump_vec( stdout, &X->q[j], 7 );
        lwa4_kin_duqu( &X->q[j], S0[i].data, aa_tf_duqu_ident,
                       X->S_wp[i], X->J_wp[i]  );
        /*-- Hand --*/
        double y1, y2;
        rfx_kin_2d2_fk( SDH_L1, SDH_L2,
                        X->q[k + PIR_SDH_L0],
                        X->q[k + PIR_SDH_L1],
                        NULL,
                        &y1 );
        rfx_kin_2d2_fk( SDH_L1, SDH_L2,
                        X->q[k + PIR_SDH_R0],
                        X->q[k + PIR_SDH_R1],
                        NULL,
                        &y2 );
        // add F/T, SDH, Fingers to S_ee
        double s0[8], s1[8];
        double x = LWA4_L_e + LWA4_FT_L + SDH_LB + (y1+y2)/2;
        aa_tf_xxyz2duqu( -60 * M_PI/180, x, 0, 0, s0 );
        aa_tf_xxyz2duqu( 0, 0, 0, -SDH_FC, s1 );
        aa_tf_duqu_mul( s0, s1, X->S_eer[i] );

        /* double x = LWA4_L_e + LWA4_FT_L + SDH_LB + (y1+y2)/2; */
        /* aa_tf_xxyz2duqu( -60 * M_PI/180, */
        /*                  x, 0, 0, */
        /*                  X->S_eer[i] ); */

    }


    return 0;
}

int pir_kin_ft( struct pir_state *X, double F_raw[2][6], double r_ft[2][4] ) {

    if( !is_init) kin_init();

    for( size_t i = 0; i < 2; i ++ ) {
        int j, k;
        PIR_SIDE_INDICES(i, j, k);
        /*-- F/T --*/
        aa_tf_qmul( X->S_wp[i], r_ft_rel.data, r_ft[i] );
        // rotate
        aa_tf_qrot( r_ft[i], F_raw[i],   X->F[i]);
        aa_tf_qrot( r_ft[i], F_raw[i]+3, X->F[i]+3 );
        // subtract end-effector mass
        X->F[i][2] = X->F[i][2] - PIR_FT_WEIGHT - SDH_WEIGHT;
    }
}
