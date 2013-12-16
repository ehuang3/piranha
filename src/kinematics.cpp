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



static int is_init = 0;
static amino::DualQuat S0[2];
static amino::Quat r_ft_rel;

static void kin_init(void) {

    // Arm
    {
        amino::RotMat R0(0,0,-1,
                         1,0,0,
                         0,-1,0);
        assert( aa_tf_isrotmat( R0.data ) );

        S0[PIR_LEFT] = amino::DualQuat( amino::Quat(R0),
                                        amino::Vec3(0, LWA4_L_0 + PIR_L_SHOULDER_WIDTH/2 - LWA4_L_P,  0) );

        amino::DualQuat Srel( amino::Quat(amino::AxisAngle( 0,0,1, M_PI )),
                              amino::Vec3(0,0,0) );

        S0[PIR_RIGHT] = Srel * S0[PIR_LEFT];
        amino::AxisAngle A0(S0[PIR_LEFT].real);
        printf("arm: ");
        aa_dump_vec( stdout, A0.data, 4 );
    }

    // F/T rotation
    {
        amino::RotMat R0( 0, 0, 1,
                          1, 0, 0,
                          0, 1, 0 );
        assert(aa_tf_isrotmat(R0.data));

        amino::AxisAngle R_rel( 0,0,1, LWA4_FT_ANGLE );

        r_ft_rel = amino::Quat(R0) * amino::Quat(R_rel);
    }

    is_init = 1;
}

int pir_kin_arm( struct pir_state *X ) {

    if( !is_init) kin_init();

    for( size_t i = 0; i < 2; i ++ ) {
        int j, k;
        PIR_SIDE_INDICES(i, j, k);
        /*-- Arm --*/
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
        double x = LWA4_L_e + LWA4_FT_L + SDH_LB + (y1+y2)/2;
        aa_tf_xxyz2duqu( -60 * M_PI/180,
                         x, 0, -SDH_FC,
                         X->S_eer[i] );

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
