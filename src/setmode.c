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

static void zero_refs(pirctrl_cx_t *cx) {
    AA_MEM_CPY( cx->G_L.ref.S, cx->state.S_wp_L, 8 );
    AA_MEM_SET( cx->G_L.ref.q, 0, cx->G_L.n_q );
    AA_MEM_SET( cx->G_L.ref.dq, 0, cx->G_L.n_q );

    AA_MEM_CPY( cx->G_R.ref.S, cx->state.S_wp_R, 8 );
    AA_MEM_SET( cx->G_R.ref.q, 0, cx->G_R.n_q );
    AA_MEM_SET( cx->G_R.ref.dq, 0, cx->G_R.n_q );
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
        aa_fset( cx->Kx.p, msg_ctrl->x[0].f, 3 );
    }
    return 0;
}
int set_mode_k_pr(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if ( msg_ctrl->n == 1 ) {
        printf("k_pr: %f\n", msg_ctrl->x[0].f );
        aa_fset( cx->Kx.p+3, msg_ctrl->x[0].f, 3 );
    }
    return 0;
}

int set_mode_k_q(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if(  msg_ctrl->n == 1 ) {
        printf("k_q: %f\n", msg_ctrl->x[0].f );
        aa_fset( cx->Kx.q, msg_ctrl->x[0].f, 7 );
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
    zero_refs(cx);
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}


int set_mode_ws_left_finger(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    zero_refs(cx);
    aa_tf_duqu_smul( cx->state.S_wp_L, cx->state.S_eer_L, cx->G_L.ref.S );
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}

int set_mode_ws_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    zero_refs(cx);
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}

int set_mode_sin(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    cx->sint=0;
    set_mode_cpy(cx,msg_ctrl);
    return 0;
}

int set_mode_trajx(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if( msg_ctrl->n < 9 ) return -1;
    zero_refs(cx);
    /* double *S1 = &msg_ctrl->x[0].f; */
    /* aa_tf_duqu_cmul( cx->state.S_L, S1, S_rel ); */
    /* aa_tf_duqu_trans( S_rel, xr ); */
    /* printf("xrel: "); aa_dump_vec( stdout, xr, 3 ); */
    /* printf("qrel: "); aa_dump_vec( stdout, S_rel, 4 ); */
    /* printf("S0: "); aa_dump_vec( stdout, cx->state.S_L, 8 ); */
    /* printf("S1: "); aa_dump_vec( stdout, S1, 8 ); */


    // free old stuff
    aa_mem_region_release( &cx->modereg );

    // aloc
    rfx_trajx_parablend_t *T = AA_MEM_REGION_NEW( &cx->modereg, rfx_trajx_parablend_t );
    rfx_trajx_splend_init( T, &cx->modereg, 1 );

    rfx_trajx_t *pT = (rfx_trajx_t*)T;

    // initial point
    {
        double S0[8];
        aa_tf_duqu_mul( cx->state.S_wp_L, cx->state.S_eer_L, S0 );
        rfx_trajx_add_duqu( pT, 0, S0 );
    }

    // final point
    for( size_t i = 0; i + 9 <= msg_ctrl->n; i += 9 ) {
        double t = msg_ctrl->x[i].f;
        double *S = &msg_ctrl->x[i+1].f;
        rfx_trajx_add_duqu( pT, t, S );
    }

    // generate
    rfx_trajx_generate( pT );

    // debugging plot
    {
        //struct rfx_trajx_plot_opts xopts = {0};
        //xopts.to_file = 1;
        //rfx_trajx_plot( pT, .001, &xopts );
    }
    memcpy( &cx->t0, &cx->now, sizeof(cx->t0) );

    cx->trajx = pT;
    return 0;
}


int set_mode_trajq(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if( msg_ctrl->n != 8 ) return -1;
    zero_refs(cx);

    // free old stuff
    aa_mem_region_release( &cx->modereg );

    // aloc
    rfx_trajq_trapvel_t *T = AA_MEM_REGION_NEW( &cx->modereg, rfx_trajq_trapvel_t );
    rfx_trajq_trapvel_init( T, &cx->modereg, 7 );

    for( size_t i = 0; i < 7; i ++ ) {
        T->dq_max[i] = 10.0;
        T->ddq_max[i] = 10.0;
    }

    // initial point
    rfx_trajq_add( &T->traj, 0, cx->state.q + PIR_AXIS_L0 );
    rfx_trajq_add( &T->traj, msg_ctrl->x[0].f, &msg_ctrl->x[1].f );

    rfx_trajq_generate( &T->traj );

    memcpy( &cx->t0, &cx->now, sizeof(cx->t0) );

    //rfx_trajq_plot( &T->traj, .001 );

    cx->trajq = &T->traj;

    return 0;
}
