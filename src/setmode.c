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

void pir_zero_refs(pirctrl_cx_t *cx) {
    for( int side = 0; side < 2; side++ ) {
        AA_MEM_CPY( cx->G[side].ref.S, cx->state.S_wp[side], 8 );
        AA_MEM_SET( cx->G[side].ref.q,  0, cx->G[side].n_q );
        AA_MEM_SET( cx->G[side].ref.dq, 0, cx->G[side].n_q );
    }
}

int set_mode_nop(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    (void)msg_ctrl;
    (void)cx;
    return 0;
}


int set_mode_k_s2min(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if( msg_ctrl->n == 1 ) {
        printf("k_s2min: %f\n", msg_ctrl->x[0].f );
        cx->Kx.s2min = msg_ctrl->x[0].f;
    }
    return 0;
}

int set_mode_k_pt(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if ( msg_ctrl->n == 1 ) {
        printf("k_pt: %f\n", msg_ctrl->x[0].f );
        AA_MEM_SET( cx->Kx.p, msg_ctrl->x[0].f, 3 );
    }
    return 0;
}
int set_mode_k_pr(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if ( msg_ctrl->n == 1 ) {
        printf("k_pr: %f\n", msg_ctrl->x[0].f );
        AA_MEM_SET( cx->Kx.p+3, msg_ctrl->x[0].f, 3 );
    }
    return 0;
}

int set_mode_k_q(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if(  msg_ctrl->n == 1 ) {
        printf("k_q: %f\n", msg_ctrl->x[0].f );
        AA_MEM_SET( cx->Kx.q, msg_ctrl->x[0].f, 7 );
    }
    return 0;
}
int set_mode_k_f(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if(  msg_ctrl->n == 1 ) {
        printf("k_f: %f\n", msg_ctrl->x[0].f );
        AA_MEM_SET( cx->Kx.f, msg_ctrl->x[0].f, 3 );
    }
    return 0;
}

int set_mode_cpy(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    printf("ctrl: %s\n", msg_ctrl->mode );
    memcpy( &cx->msg_ctrl, msg_ctrl, sizeof(cx->msg_ctrl) );
    return 0;
}

int set_mode_ws_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    pir_zero_refs(cx);
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}


int set_mode_ws_left_finger(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    pir_zero_refs(cx);
    aa_tf_duqu_smul( cx->state.S_wp[PIR_LEFT], cx->state.S_eer[PIR_LEFT], cx->G[PIR_LEFT].ref.S );
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}

int set_mode_ws_right_finger(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    pir_zero_refs(cx);
    aa_tf_duqu_smul( cx->state.S_wp[PIR_RIGHT], cx->state.S_eer[PIR_RIGHT], cx->G[PIR_RIGHT].ref.S );
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}

int set_mode_ws_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    pir_zero_refs(cx);
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}

int set_mode_sin(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    cx->sint=0;
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}

int set_mode_trajx_side(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl, double S0[8] ) {
    if( msg_ctrl->n < 9 ) return -1;
    pir_zero_refs(cx);

    // free old stuff
    aa_mem_region_release( &cx->modereg );

    // aloc
    struct rfx_trajx_point_list *plist = rfx_trajx_point_list_alloc( &cx->modereg );

    // initial point
    double t = 0;
    rfx_trajx_point_list_addb_duqu( plist, t, 1, S0 );

    // final point
    for( size_t i = 0; i + 9 <= msg_ctrl->n; i += 9 ) {
        double dt = msg_ctrl->x[i].f;
        t += dt;
        double *S = &msg_ctrl->x[i+1].f;
        rfx_trajx_point_list_addb_duqu( plist, t, 1, S );
    }

    // generate
    cx->trajx_segs = rfx_trajx_splend_generate( plist, &cx->modereg );

    memcpy( &cx->t0, &cx->now, sizeof(cx->t0) );

    return 0;
}


int set_mode_trajx_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    double S0[8];
    aa_tf_duqu_mul( cx->state.S_wp[PIR_LEFT], cx->state.S_eer[PIR_LEFT], S0 );
    return set_mode_trajx_side( cx, msg_ctrl, S0 );
}

int set_mode_trajx_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    double S0[8];
    aa_tf_duqu_mul( cx->state.S_wp[PIR_RIGHT], cx->state.S_eer[PIR_RIGHT], S0 );
    return set_mode_trajx_side( cx, msg_ctrl, S0 );
}

int set_mode_trajx_w_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    return set_mode_trajx_side( cx, msg_ctrl, cx->state.S_wp[PIR_LEFT] );
}

int set_mode_trajx_w_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    return set_mode_trajx_side( cx, msg_ctrl, cx->state.S_wp[PIR_RIGHT]);
}

static int collect_trajq(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl, double *q0, size_t n ) {
    if( msg_ctrl->n < n+1 ) return -1;
    pir_zero_refs(cx);

    // free old stuff
    aa_mem_region_release( &cx->modereg );

    struct rfx_trajq_points *points = rfx_trajq_points_alloc( &cx->modereg, n );

    double t = 0;
    rfx_trajq_points_add( points, t, q0 );
    for( size_t i = 0; i + (n+1) <= msg_ctrl->n; i += (n+1) ) {
        t +=  msg_ctrl->x[i].f;
        rfx_trajq_points_add( points, t, &msg_ctrl->x[i+1].f );
        printf("t: ");
        aa_dump_vec(stdout, &msg_ctrl->x[i+1].f, n );
    }

    cx->trajq_segs = rfx_trajq_gen_pblend_tm1( &cx->modereg, points, 1.0 );

    memcpy( &cx->t0, &cx->now, sizeof(cx->t0) );
    return 0;
}

int set_mode_trajq_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    return collect_trajq(cx, msg_ctrl, cx->state.q + PIR_AXIS_L0, 7 );
}

int set_mode_trajq_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    return collect_trajq(cx, msg_ctrl, cx->state.q + PIR_AXIS_R0, 7 );
}

int set_mode_trajq_lr(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    _Static_assert(  PIR_AXIS_L0 + 7 == PIR_AXIS_R0, "Invalid axis ordering" );
    return collect_trajq(cx, msg_ctrl, cx->state.q + PIR_AXIS_L0, 14 );
}

int set_mode_trajq_torso(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if( msg_ctrl->n != 2 ) return -1;
    pir_zero_refs(cx);

    // free old stuff
    aa_mem_region_release( &cx->modereg );

    // aloc
    rfx_trajq_trapvel_t *T = AA_MEM_REGION_NEW( &cx->modereg, rfx_trajq_trapvel_t );
    rfx_trajq_trapvel_init( T, &cx->modereg, 1 );

    for( size_t i = 0; i < 1; i ++ ) {
        T->dq_max[i] = 10.0;
        T->ddq_max[i] = 10.0;
    }

    // initial point
    rfx_trajq_add( &T->traj, 0, cx->state.q + PIR_AXIS_T );
    rfx_trajq_add( &T->traj, msg_ctrl->x[0].f, &msg_ctrl->x[1].f );

    rfx_trajq_generate( &T->traj );

    //printf( "torso: %f, %f\n", msg_ctrl->x[0].f, msg_ctrl->x[1].f );

    memcpy( &cx->t0, &cx->now, sizeof(cx->t0) );

    //rfx_trajq_plot( &T->traj, .001 );

    cx->trajq = &T->traj;

    return 0;
}


int set_mode_servo_cam(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl )
{
    pir_zero_refs(cx);
    if( msg_ctrl->n*sizeof(double) != sizeof( struct servo_cam_cx) ) return -1;

    // free old stuff
    aa_mem_region_release( &cx->modereg );

    // copy target pose
    cx->mode_cx = aa_mem_region_dup( &cx->modereg,
                                     &msg_ctrl->x[0].f, sizeof(struct servo_cam_cx) );

    return 0;
}

int set_mode_biservo_rel(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    pir_zero_refs(cx);
    if( msg_ctrl->n*sizeof(double) != sizeof( struct biservo_rel_cx) ) return -1;

    // free old stuff
    aa_mem_region_release( &cx->modereg );

    // copy target pose
    cx->mode_cx = aa_mem_region_dup( &cx->modereg,
                                     &msg_ctrl->x[0].f, sizeof(struct biservo_rel_cx) );

    return 0;
}
