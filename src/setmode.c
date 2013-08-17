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

void set_mode_nop(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    (void)msg_ctrl;
    (void)cx;
}


void set_mode_k_s2min(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if( msg_ctrl->n == 1 ) {
        printf("k_s2min: %f\n", msg_ctrl->x[0].f );
        cx->K.s2min = msg_ctrl->x[0].f;
    }
}

void set_mode_k_pt(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if ( msg_ctrl->n == 1 ) {
        printf("k_pt: %f\n", msg_ctrl->x[0].f );
        aa_fset( cx->K.p, msg_ctrl->x[0].f, 3 );
    }
}
void set_mode_k_pr(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if ( msg_ctrl->n == 1 ) {
        printf("k_pr: %f\n", msg_ctrl->x[0].f );
        aa_fset( cx->K.p+3, msg_ctrl->x[0].f, 3 );
    }
}

void set_mode_k_q(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if(  msg_ctrl->n == 1 ) {
        printf("k_q: %f\n", msg_ctrl->x[0].f );
        aa_fset( cx->K.q, msg_ctrl->x[0].f, 7 );
    }
}

void set_mode_cpy(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    printf("ctrl: %s\n", msg_ctrl->mode );
    memcpy( &cx->msg_ctrl, msg_ctrl, sizeof(cx->msg_ctrl) );
}

void set_mode_ws_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    aa_tf_duqu2qv( cx->state.S_L, cx->G_L.r_r, cx->G_L.x_r );
    set_mode_cpy(cx,msg_ctrl);
}

void set_mode_ws_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    aa_tf_duqu2qv( cx->state.S_R, cx->G_R.r_r, cx->G_R.x_r );
    set_mode_cpy(cx,msg_ctrl);
}

void set_mode_sin(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    cx->sint=0;
    set_mode_cpy(cx,msg_ctrl);
}

void set_mode_trajx(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl ) {
    if( msg_ctrl->n != 8 ) return;
    double S_rel[8], xr[3];
    double *S1 = &msg_ctrl->x[0].f;
    aa_tf_duqu_cmul( cx->state.S_L, S1, S_rel );
    aa_tf_duqu_trans( S_rel, xr );
    printf("xrel: "); aa_dump_vec( stdout, xr, 3 );
    printf("qrel: "); aa_dump_vec( stdout, S_rel, 4 );
    printf("S0: "); aa_dump_vec( stdout, cx->state.S_L, 8 );
    printf("S1: "); aa_dump_vec( stdout, S1, 8 );


    // free old stuff
    aa_mem_region_release( &cx->modereg );

    // aloc
    rfx_trajx_parablend_t *T = AA_MEM_REGION_NEW( &cx->modereg, rfx_trajx_parablend_t );
    rfx_trajx_splend_init( T, &cx->modereg, 1 );

    rfx_trajx_t *pT = (rfx_trajx_t*)T;

    // initial point
    double r0[4], x0[3];
    aa_tf_duqu2qv( cx->state.S_L, r0, x0 );
    rfx_trajx_add( pT, 0, x0, r0 );

    // final point
    double r1[4], x1[3];
    aa_tf_duqu2qv( S1, r1, x1 );
    rfx_trajx_add( pT, 5, x1, r1 );

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
}
