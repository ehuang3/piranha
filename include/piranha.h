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

#ifndef PIRANHA_H
#define PIRANHA_H

#include <amino.h>
#include <reflex.h>

#define PIR_MAX_MSG_AXES 7

enum pir_axis {
    PIR_AXIS_T  =   0,
    PIR_AXIS_L0 =   1,
    PIR_AXIS_L1 =   2,
    PIR_AXIS_L2 =   3,
    PIR_AXIS_L3 =   4,
    PIR_AXIS_L4 =   5,
    PIR_AXIS_L5 =   6,
    PIR_AXIS_L6 =   7,
    PIR_AXIS_R0 =   8,
    PIR_AXIS_R1 =   9,
    PIR_AXIS_R2 =  10,
    PIR_AXIS_R3 =  11,
    PIR_AXIS_R4 =  12,
    PIR_AXIS_R5 =  13,
    PIR_AXIS_R6 =  14,
    PIR_AXIS_CNT = 15
};

// TODO: all transforms, jacobians, forces, in global frame

struct pir_state {
    double q[PIR_AXIS_CNT];
    double dq[PIR_AXIS_CNT];

    double F_L[6];
    double F_R[6];

    double Tee[12];
    double T0[12];
    double T_L[12];
    double J_L[7*6];
    double T_R[12];
    double J_R[7*6];
};

void lwa4_kin_( const double *q, const double *T0, const double *Tee, double *T, double *J );
void lwa4_tf_( const double *q, double *TT );
void lwa4_tf_abs_( const double *q, const double *T0, double *TT );


void lwa4_kin2_( const double *q, const double *T0, const double *Tee, double *T, double *J );
#endif //PIRANHA_H
