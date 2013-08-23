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


pirctrl_cx_t cx;



// 20 deg/s
#define MAXVEL_FACTOR 20 * M_PI/180


#define VALID_NS (1000000000 / 5)

static void set_mode(void);
static void update(void);
static void control(void);

// all the different control modes
typedef void (*ctrl_fun_t)(void);
static void ctrl_joint_left_shoulder(void);
static void ctrl_joint_left_wrist(void);
static void ctrl_joint_right_shoulder(void);
static void ctrl_joint_right_wrist(void);
static void ctrl_ws_left(void);
static void ctrl_ws_right(void);
static void ctrl_zero(void);
static void ctrl_sin(void);
static void ctrl_step(void);
static void ctrl_trajx(void);
static void ctrl_trajq(void);

static void control_n( uint32_t n, size_t i, ach_channel_t *chan );


struct pir_mode_desc mode_desc[] = {
    {"left-shoulder",
     set_mode_cpy,
     ctrl_joint_left_shoulder},
    {"left-wrist",
     set_mode_cpy,
     ctrl_joint_left_wrist},
    {"right-shoulder",
     set_mode_cpy,
     ctrl_joint_right_shoulder},
    {"right-wrist",
     set_mode_cpy,
     ctrl_joint_right_wrist},
    {"ws-left",
     set_mode_ws_left,
     ctrl_ws_left},
    {"ws-right",
     set_mode_ws_right,
     ctrl_ws_right},
    {"zero",
     set_mode_cpy,
     ctrl_zero},
    {"sin",
     set_mode_sin,
     ctrl_sin},
    {"step",
     set_mode_cpy,
     ctrl_step},
    {"trajx",
     set_mode_trajx,
     ctrl_trajx},
    {"trajq",
     set_mode_trajq,
     ctrl_trajq},
    {"sdh-zero",
     sdh_zero,
     NULL},
    {"k-pt",
     set_mode_k_pt,
     NULL},
    {"k-pr",
     set_mode_k_pr,
     NULL},
    {"k-f",
     set_mode_k_f,
     NULL},
    {NULL, NULL, NULL} };


static const double tf_ident[] = {1,0,0, 0,1,0, 0,0,1, 0,0,0};

int main( int argc, char **argv ) {
    memset(&cx, 0, sizeof(cx));
    cx.dt = .002;

    /*-- args --*/
    for( int c; -1 != (c = getopt(argc, argv, "V?hH" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES;
        default:
            SNS_DIE( "Invalid argument: %s\n", optarg );
        }
    }

    sns_start();

    // open channel
    sns_chan_open( &cx.chan_js,          "joystick",    NULL );
    sns_chan_open( &cx.chan_ctrl,        "pir-ctrl",    NULL );
    sns_chan_open( &cx.chan_ref_torso,   "ref-torso",   NULL );
    sns_chan_open( &cx.chan_ref_left,    "ref-left",    NULL );
    sns_chan_open( &cx.chan_ref_right,   "ref-right",   NULL );
    sns_chan_open( &cx.chan_sdhref_left,    "sdhref-left",    NULL );
    sns_chan_open( &cx.chan_sdhref_right,   "sdhref-right",   NULL );
    sns_chan_open( &cx.chan_state_pir,   "pir-state",   NULL );
    {
        ach_channel_t *chans[] = {&cx.chan_state_pir, &cx.chan_js, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    // alloc messages
    cx.msg_ref = sns_msg_motor_ref_alloc( PIR_MAX_MSG_AXES );
    cx.msg_ref->mode = SNS_MOTOR_MODE_VEL;

    // memory
    aa_mem_region_init( &cx.modereg, 64 * 1024 );

    // setup reflex controller
    for( size_t i = 0; i < PIR_AXIS_CNT; i ++ ) {
        cx.q_min[i] = -2*M_PI;
        cx.q_max[i] = M_PI;
    }
    // left
    cx.G_L.n_q = 7;
    cx.G_L.J =  cx.state.J_L;
    cx.G_L.act.q =  &cx.state.q[PIR_AXIS_L0];
    cx.G_L.act.dq = &cx.state.dq[PIR_AXIS_L0];
    cx.G_L.act.S = cx.state.S_L;
    cx.G_L.act.F = cx.state.F_L;
    cx.G_L.ref.q =  &cx.ref.q[PIR_AXIS_L0];
    cx.G_L.ref.dq = &cx.ref.dq[PIR_AXIS_L0];
    cx.G_L.q_min = &cx.q_min[PIR_AXIS_L0];
    cx.G_L.q_max = &cx.q_max[PIR_AXIS_L0];
    cx.G_L.ref.S = AA_NEW0_AR( double, 8 );
    cx.G_L.ref.F = AA_NEW0_AR( double, 6 );
    cx.G_L.ref.dx = AA_NEW0_AR( double, 6 );
    cx.G_L.act.dx = AA_NEW0_AR( double, 6 );
    for( size_t i = 0; i < 3; i ++ ) {
        cx.G_L.x_min[i] = -10;
        cx.G_L.x_max[i] = 10;
    }
    cx.G_L.F_max = 20;
    // right
    cx.G_R.n_q = 7;
    cx.G_R.J =  cx.state.J_R;
    cx.G_R.act.q =  &cx.state.q[PIR_AXIS_R0];
    cx.G_R.act.dq = &cx.state.dq[PIR_AXIS_R0];
    cx.G_R.act.S = cx.state.S_R;
    cx.G_R.act.F = cx.state.F_R;
    cx.G_R.ref.q =  &cx.ref.q[PIR_AXIS_R0];
    cx.G_R.ref.dq = &cx.ref.dq[PIR_AXIS_R0];
    cx.G_R.q_min = &cx.q_min[PIR_AXIS_R0];
    cx.G_R.q_max = &cx.q_max[PIR_AXIS_R0];
    cx.G_R.ref.S = AA_NEW0_AR( double, 8 );
    cx.G_R.ref.F = AA_NEW0_AR( double, 6 );
    cx.G_R.ref.dx = AA_NEW0_AR( double, 6 );
    cx.G_R.act.dx = AA_NEW0_AR( double, 6 );
    for( size_t i = 0; i < 3; i ++ ) {
        cx.G_R.x_min[i] = -10;
        cx.G_R.x_max[i] = 10;
    }
    cx.G_R.F_max = 20;

    rfx_ctrl_ws_lin_k_init( &cx.Kx, 7 );
    aa_fset( cx.Kx.q, 0.1, 7 );
    cx.Kx.q[1] *= 5; // lower limits
    cx.Kx.q[3] *= 5; // lower limits
    cx.Kx.q[5] *= 5; // lower limits
    cx.Kx.q[6] *= 5; // this module is most sensitive to limits
    aa_fset( cx.Kx.f, .003, 6 );
    //aa_fset( cx.Kx.f, -.000, 6 );
    aa_fset( cx.Kx.p, 1.0, 3 );
    aa_fset( cx.Kx.p+3, 1.0, 3 );
    /* aa_fset( cx.K.p, 0.0, 3 ); */
    /* aa_fset( cx.K.p+3, 0.0, 3 ); */
    cx.Kx.dls = .005;
    cx.Kx.s2min = .01;
    printf("dls s2min: %f\n", cx.Kx.s2min);
    printf("dls k: %f\n", cx.Kx.dls);

    // joint
    cx.Kq.n_q = 7;
    cx.Kq.p = AA_NEW_AR( double, 7 );
    AA_MEM_SET( cx.Kq.p, 0, 7 );
    if( clock_gettime( ACH_DEFAULT_CLOCK, &cx.now ) )
        SNS_LOG( LOG_ERR, "clock_gettime failed: '%s'\n", strerror(errno) );

    /* -- RUN -- */
    while (!sns_cx.shutdown) {

        // get state
        update();

        //if( clock_gettime( ACH_DEFAULT_CLOCK, &cx.now ) )
            //SNS_LOG( LOG_ERR, "clock_gettime failed: '%s'\n", strerror(errno) );

        // control
        control();

        cx.now = sns_time_add_ns(cx.now, (int64_t)(cx.dt*1e9) );
        clock_nanosleep( ACH_DEFAULT_CLOCK, TIMER_ABSTIME,
                         &cx.now, NULL );

        aa_mem_region_local_release();

    }

    sns_end();
    return 0;
}

static void update(void) {

    // state
    {
        size_t frame_size;
        ach_status_t r = ach_get( &cx.chan_state_pir, &cx.state, sizeof(cx.state), &frame_size,
                                  NULL, ACH_O_LAST );
        switch(r) {
        case ACH_OK:
        case ACH_MISSED_FRAME:
        case ACH_CANCELED:
        case ACH_TIMEOUT:
        case ACH_STALE_FRAMES:
            break;
        default:
            SNS_LOG(LOG_ERR, "Failed to get frame: %s\n", ach_result_to_string(r) );
        }
    }


    // joystick
    {
        size_t frame_size;
        struct sns_msg_joystick *msg = NULL;
        ach_status_t r = sns_msg_local_get( &cx.chan_js, (void**)&msg,
                                            &frame_size,
                                            NULL, ACH_O_LAST );

        // validate
        switch(r) {
        case ACH_OK:
        case ACH_MISSED_FRAME:
            if( msg->n == JS_AXES &&
                frame_size == sns_msg_joystick_size(msg) )
            {
                if( msg->buttons & GAMEPAD_BUTTON_BACK ) {
                    if( strcmp(cx.msg_ctrl.mode, "halt") ) {
                        printf("HALT\n");
                    }
                    strcpy( cx.msg_ctrl.mode, "halt" );
                    memset(cx.ref.user, 0, sizeof(cx.ref.user[0])*JS_AXES);
                } else {
                    memcpy(cx.ref.user, msg->axis, sizeof(cx.ref.user[0])*msg->n);
                    cx.ref.user_button = msg->buttons;
                }
            }
        break;
        case ACH_STALE_FRAMES:
        case ACH_TIMEOUT:
        case ACH_CANCELED:
            break;
        default:
            SNS_LOG(LOG_ERR, "Error getting joystick message\n");
        }
    }

    //mode
    set_mode();
}


static void set_mode(void) {
    // poll mode
    size_t frame_size;
    struct pir_msg *msg_ctrl;
    ach_status_t r = sns_msg_local_get( &cx.chan_ctrl, (void**)&msg_ctrl,
                                        &frame_size, NULL, ACH_O_LAST );
    if( ACH_OK == r || ACH_MISSED_FRAME == r ) {
        msg_ctrl->mode[63] = '\0';
        printf("looking for mode: %s\n",msg_ctrl->mode);
        for( size_t i = 0; mode_desc[i].name != NULL; i ++ ) {
            if( 0 == strcmp(msg_ctrl->mode, mode_desc[i].name) ) {
                printf("found mode: %s\n", mode_desc[i].name);
                if( mode_desc[i].setmode ) {
                    if( 0 == mode_desc[i].setmode( &cx, msg_ctrl ) &&
                        mode_desc[i].ctrl )
                    {
                        cx.mode = &mode_desc[i];
                    }
                }
                break;
            }
        }
    }

}

static void control(void) {
    // dispatch
    memset( cx.ref.dq, 0, sizeof(cx.ref.dq[0])*PIR_AXIS_CNT );
    if( cx.mode && cx.mode->ctrl ) {
        cx.mode->ctrl();
    }

    // send ref
    sns_msg_set_time( &cx.msg_ref->header, &cx.now, VALID_NS );
    // torso
    control_n( 1, PIR_AXIS_T, &cx.chan_ref_torso );
    // left
    control_n( 7, PIR_AXIS_L0, &cx.chan_ref_left );
    // right
    control_n( 7, PIR_AXIS_R0, &cx.chan_ref_right );
}

static void control_n( uint32_t n, size_t i, ach_channel_t *chan ) {
    memcpy( &cx.msg_ref->u[0], &cx.ref.dq[i], sizeof(cx.msg_ref->u[0])*n );
    cx.msg_ref->mode = SNS_MOTOR_MODE_VEL;
    cx.msg_ref->n = n;
    ach_put( chan, cx.msg_ref, sns_msg_motor_ref_size(cx.msg_ref) );
    // TODO: check result
}



static void ctrl_joint_torso(void) {
    double u = cx.ref.user[GAMEPAD_AXIS_RT] - cx.ref.user[GAMEPAD_AXIS_LT];
    cx.ref.dq[PIR_AXIS_T] = u * MAXVEL_FACTOR;
}
static void ctrl_joint_left_shoulder(void) {
    ctrl_joint_torso();
    cx.ref.dq[PIR_AXIS_L0] = cx.ref.user[GAMEPAD_AXIS_LY] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_L1] = cx.ref.user[GAMEPAD_AXIS_LX] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_L2] = cx.ref.user[GAMEPAD_AXIS_RY] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_L3] = cx.ref.user[GAMEPAD_AXIS_RX] * MAXVEL_FACTOR;
}
static void ctrl_joint_left_wrist(void) {
    ctrl_joint_torso();
    cx.ref.dq[PIR_AXIS_L3] = cx.ref.user[GAMEPAD_AXIS_LX] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_L4] = cx.ref.user[GAMEPAD_AXIS_LY] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_L5] = cx.ref.user[GAMEPAD_AXIS_RX] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_L6] = cx.ref.user[GAMEPAD_AXIS_RY] * MAXVEL_FACTOR;
}
static void ctrl_joint_right_shoulder(void) {
    ctrl_joint_torso();
    cx.ref.dq[PIR_AXIS_R0] = cx.ref.user[GAMEPAD_AXIS_LY] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_R1] = cx.ref.user[GAMEPAD_AXIS_LX] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_R2] = cx.ref.user[GAMEPAD_AXIS_RY] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_R3] = cx.ref.user[GAMEPAD_AXIS_RX] * MAXVEL_FACTOR;
}
static void ctrl_joint_right_wrist(void) {
    ctrl_joint_torso();
    cx.ref.dq[PIR_AXIS_R3] = cx.ref.user[GAMEPAD_AXIS_LX] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_R4] = cx.ref.user[GAMEPAD_AXIS_LY] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_R5] = cx.ref.user[GAMEPAD_AXIS_RX] * MAXVEL_FACTOR;
    cx.ref.dq[PIR_AXIS_R6] = cx.ref.user[GAMEPAD_AXIS_RY] * MAXVEL_FACTOR;
}


static void ctrl_zero(void) {
    for( size_t i = PIR_AXIS_L0; i <= PIR_AXIS_L6; i ++ ) {
        double k = -cx.ref.user[GAMEPAD_AXIS_LT] + cx.ref.user[GAMEPAD_AXIS_RT];
        cx.ref.dq[i] = - .25* k * cx.state.q[i];
    }
    for( size_t i = PIR_AXIS_R0; i <= PIR_AXIS_R6; i ++ ) {
        double k = -cx.ref.user[GAMEPAD_AXIS_LT] + cx.ref.user[GAMEPAD_AXIS_RT];
        cx.ref.dq[i] = - .25* k * cx.state.q[i];
    }
}


static void ctrl_ws(size_t i, rfx_ctrl_ws_t *G ) {
    // set refs
    AA_MEM_SET( G->ref.dx, 0, 6 );
    if( cx.ref.user_button & GAMEPAD_BUTTON_RB ) {
        G->ref.dx[3] = cx.ref.user[GAMEPAD_AXIS_LX] * .3;
        G->ref.dx[4] = cx.ref.user[GAMEPAD_AXIS_LY] * .3;
        G->ref.dx[5] = cx.ref.user[GAMEPAD_AXIS_RX] * .3;

        G->ref.dx[0] = cx.ref.user[GAMEPAD_AXIS_DX] * .02;
        G->ref.dx[1] = cx.ref.user[GAMEPAD_AXIS_DY] * .02;
        G->ref.dx[2] = cx.ref.user[GAMEPAD_AXIS_RY] * .1;
    } else {
        G->ref.dx[0] = cx.ref.user[GAMEPAD_AXIS_LX] * .1;
        G->ref.dx[1] = cx.ref.user[GAMEPAD_AXIS_LY] * .1;
        G->ref.dx[2] = cx.ref.user[GAMEPAD_AXIS_RX] * .1;
    }

    //printf("--\n");
    //printf("r0:   ");aa_dump_vec(stdout,  G->r, 4 );
    //printf("x0:   ");aa_dump_vec(stdout,  G->x, 3 );
    //printf("r_r0: ");aa_dump_vec(stdout,  G->r_r, 4 );
    ////printf("x_r0: ");aa_dump_vec(stdout,  G->x_r, 3 );
    //printf("dx_r: ");aa_dump_vec(stdout, G->dx_r, 6 );

    // compute stuff
    int r = rfx_ctrl_ws_lin_vfwd( G, &cx.Kx, &cx.ref.dq[i] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }
    //printf("dq:   ");aa_dump_vec(stdout, &cx.ref.dq[i], 7 );

    // integrate
    rfx_ctrl_ws_sdx( G, cx.dt );

    //printf("r_r1: ");aa_dump_vec(stdout,  G->r_r, 4 );
    //printf("x_r1: ");aa_dump_vec(stdout,  G->x_r, 3 );
}

static void ctrl_ws_left(void) {
    ctrl_ws( PIR_AXIS_L0, &cx.G_L );
}

static void ctrl_ws_right(void) {
    ctrl_ws( PIR_AXIS_R0, &cx.G_R );
}

static void ctrl_sin(void) {

    double k = (cx.ref.user[GAMEPAD_AXIS_LT] + cx.ref.user[GAMEPAD_AXIS_RT]) ;

    double v = k*sin(cx.sint);

    //printf("%f\n", v*180/M_PI);

    cx.ref.dq[PIR_AXIS_L6] = v;
    cx.ref.dq[PIR_AXIS_L5] = v;
    cx.ref.dq[PIR_AXIS_L4] = v;
    cx.ref.dq[PIR_AXIS_L3] = v;
    cx.ref.dq[PIR_AXIS_L2] = v;
    cx.ref.dq[PIR_AXIS_L1] = v;
    cx.ref.dq[PIR_AXIS_L0] = v;

    cx.sint += .001 * 2*M_PI;

}

static void ctrl_step(void) {

    if( cx.ref.user[ GAMEPAD_AXIS_LT ] > .5 ) {
        aa_fset(& cx.ref.dq[PIR_AXIS_L0], 15 * M_PI/180, 7 );
    } else if( cx.ref.user[ GAMEPAD_AXIS_RT ] > .5 ) {
        aa_fset(& cx.ref.dq[PIR_AXIS_L0], -15 * M_PI/180, 7 );
    }


}

static void ctrl_trajx(void) {
    double t = aa_tm_timespec2sec( aa_tm_sub( cx.now, cx.t0 ) );


    // get refs
    if( t >= cx.trajx->pt_f->t ) {
        aa_tf_qv2duqu( cx.trajx->pt_f->r, cx.trajx->pt_f->x, cx.G_L.ref.S );
        AA_MEM_SET( cx.G_L.ref.dx, 0, 6 );
    } else {
        rfx_trajx_get_x_duqu( cx.trajx, t, cx.G_L.ref.S );
        rfx_trajx_get_dx( cx.trajx, t, cx.G_L.ref.dx );
    }


    int r = rfx_ctrl_ws_lin_vfwd( &cx.G_L, &cx.Kx, &cx.ref.dq[PIR_AXIS_L0] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }
    //printf("trajx: "); aa_dump_vec(stdout, dx, 6 );
    //printf("dq: "); aa_dump_vec(stdout, &cx.ref.dq[PIR_AXIS_L0], 8 );

}

static void ctrl_trajq(void) {
    double t = aa_tm_timespec2sec( aa_tm_sub( cx.now, cx.t0 ) );

    if( t >= cx.trajq->t_f ) {
        AA_MEM_CPY( cx.G_L.ref.q, cx.trajq->q_f, 7 );
        AA_MEM_SET( cx.G_L.ref.dq, 0, 7 );
    } else {
        // get refs
        rfx_trajq_get_q( cx.trajq, t, cx.G_L.ref.q );
        rfx_trajq_get_dq( cx.trajq, t, cx.G_L.ref.dq );
    }


    int r = rfx_ctrlq_lin_vfwd( &cx.G_L, &cx.Kq, &cx.ref.dq[PIR_AXIS_L0] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }
    //printf("trajx: "); aa_dump_vec(stdout, dx, 6 );
    //printf("dq: "); aa_dump_vec(stdout, &cx.ref.dq[PIR_AXIS_L0], 8 );

}
