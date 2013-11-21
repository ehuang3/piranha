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

#include <argp.h>
#include <syslog.h>
#include <sns.h>
#include <signal.h>
#include <unistd.h>
#include <inttypes.h>
#include <gamepad.h>
#include <amino.h>
#include <reflex.h>
#include "piranha.h"


void sdh_pos( pirctrl_cx_t *cx, pir_side_t side, double x[7])  {
    struct sns_msg_motor_ref *msg = sns_msg_motor_ref_local_alloc( 7 );
    sns_msg_header_fill ( &msg->header );
    msg->mode = SNS_MOTOR_MODE_POS;
    AA_MEM_CPY( msg->u, x, 7 );
    aa_dump_vec( stdout, x, 7 );
    // TODO: check result

    sns_msg_set_time( &msg->header, &cx->now, 5 * 1e9 ); // 5 sec duration

    switch(side) {
    case PIR_LEFT:
        ach_put( &cx->chan_sdhref_left, msg, sns_msg_motor_ref_size(msg) );
        break;
    case PIR_RIGHT:
        ach_put( &cx->chan_sdhref_right, msg, sns_msg_motor_ref_size(msg) );
        break;
    }
}

int sdh_zero( pirctrl_cx_t *cx, pir_side_t side, struct pir_msg *m )  {
    (void) *m;
    double x[7] = {0};
    sdh_pos( cx, side, x );
    return 0;
}

int sdh_set( pirctrl_cx_t *cx, pir_side_t side, struct pir_msg *m )  {
    (void) *m;
    if ( m->n == 7 ) {
        sdh_pos( cx, side, &m->x[0].f );
    }
    return 0;
}

int sdh_set_left( pirctrl_cx_t *cx, struct pir_msg *m )  {
    return sdh_set( cx, PIR_LEFT, m );
}

int sdh_set_right( pirctrl_cx_t *cx, struct pir_msg *m )  {
    return sdh_set( cx, PIR_RIGHT, m );
}

void cmd_ring_q(const double *theta, double *q) {
    q[0] = aa_ang_norm_pi( M_PI_2 - theta[0] );
    q[1] = aa_ang_norm_pi( M_PI_2 - q[0] - theta[1] );
}

int sdh_pinch2( pirctrl_cx_t *cx, pir_side_t side, double r, double y ) {

    double l[2] = {SDH_L1, SDH_L2};
    double x[2] = {SDH_B/2 - r, y};
    double theta_a[2], theta_b[2], qa[2], qb[2];
    int k = aa_kin_planar2_ik_theta2( l, x, theta_a, theta_b );

    cmd_ring_q( theta_a, qa );
    cmd_ring_q( theta_b, qb );

    double *q = (qa[1] > 0) ? qa : ( (qb[1] > 0) ? qb : NULL );

    if( k && !q ) {
        printf("no ik\n");
        return -1;
    }

    double X[7] = {0};
    X[PIR_SDH_T0] = -M_PI_2;
    X[PIR_SDH_T1] = -M_PI_2;
    X[PIR_SDH_AXIAL] = M_PI_2;

    X[PIR_SDH_L0] = q[0];
    X[PIR_SDH_R0] = q[0];
    X[PIR_SDH_L1] = q[1];
    X[PIR_SDH_R1] = q[1];

    sdh_pos( cx, side, X );

    return 0;
}

int sdh_pinch( pirctrl_cx_t *cx, pir_side_t side, struct pir_msg *m )  {
    (void) *m;
    if ( m->n == 2 ) {
        double r = m->x[0].f;
        double y = m->x[1].f;
        sdh_pinch2( cx, side, r, y );
    }
    return 0;
}

int sdh_pinch_left( pirctrl_cx_t *cx, struct pir_msg *m )  {
    return sdh_pinch(cx, PIR_LEFT, m);
}

int sdh_pinch_right( pirctrl_cx_t *cx, struct pir_msg *m )  {
    return sdh_pinch(cx, PIR_RIGHT, m);
}
