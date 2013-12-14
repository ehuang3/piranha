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





#define VALID_NS (1000000000 / 5)

static void set_mode(void);
static void update(void);
static void control(void);


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
    {"ws-left-finger",
     set_mode_ws_left_finger,
     ctrl_ws_left_finger},
    {"ws-right",
     set_mode_ws_right,
     ctrl_ws_right},
    {"ws-right-finger",
     set_mode_ws_right_finger,
     ctrl_ws_right_finger},
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
    {"trajq-left",
     set_mode_trajq_left,
     ctrl_trajq_left},
    {"trajq-right",
     set_mode_trajq_right,
     ctrl_trajq_right},
    {"trajq-lr",
     set_mode_trajq_lr,
     ctrl_trajq_lr},
    {"trajq-torso",
     set_mode_trajq_torso,
     ctrl_trajq_torso},
    {"sdh-set-left",
     sdh_set_left,
     NULL},
    {"sdh-set-right",
     sdh_set_left,
     NULL},
    {"pinch-left",
     sdh_pinch_left,
     NULL},
    {"pinch-right",
     sdh_pinch_right,
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
    sns_init();
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
    sns_chan_open( &cx.chan_js,           "joystick",     NULL );
    sns_chan_open( &cx.chan_ctrl,         "pir-ctrl",     NULL );
    sns_chan_open( &cx.chan_ref_torso,    "ref-torso",    NULL );
    sns_chan_open( &cx.chan_ref_left,     "ref-left",     NULL );
    sns_chan_open( &cx.chan_ref_right,    "ref-right",    NULL );
    sns_chan_open( &cx.chan_sdhref_left,  "sdhref-left",  NULL );
    sns_chan_open( &cx.chan_sdhref_right, "sdhref-right", NULL );
    sns_chan_open( &cx.chan_state_pir,    "pir-state",    NULL );
    sns_chan_open( &cx.chan_complete,     "pir-complete",    NULL );
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
    // left/right controller
    for( int side = 0; side < 2; side++ ) {
        int lwa, sdh;
        PIR_SIDE_INDICES(side, lwa, sdh);
        cx.G[side].n_q = 7;
        cx.G[side].J =  cx.state.J_wp[side];
        cx.G[side].act.q =  &cx.state.q[lwa];
        cx.G[side].act.dq = &cx.state.dq[lwa];
        cx.G[side].act.S = cx.state.S_wp[side];
        cx.G[side].act.F = cx.state.F[side];
        cx.G[side].ref.q =  &cx.ref.q[lwa];
        cx.G[side].ref.dq = &cx.ref.dq[lwa];
        cx.G[side].q_min = &cx.q_min[lwa];
        cx.G[side].q_max = &cx.q_max[lwa];
        cx.G[side].ref.S = AA_NEW0_AR( double, 8 );
        cx.G[side].ref.F = AA_NEW0_AR( double, 6 );
        cx.G[side].ref.dx = AA_NEW0_AR( double, 6 );
        cx.G[side].act.dx = AA_NEW0_AR( double, 6 );
        for( size_t i = 0; i < 3; i ++ ) {
            cx.G[side].x_min[i] = -10;
            cx.G[side].x_max[i] = 10;
        }
        cx.G[side].F_max = 20;
    }

    // LEFT_RIGHT
    _Static_assert( PIR_AXIS_L0 + 7 == PIR_AXIS_R0, "Invalid axis ordering" );
    cx.G_LR.n_q = 14;
    //cx.G_R.J =  cx.state.J_wp_R;
    cx.G_LR.act.q =  &cx.state.q[PIR_AXIS_L0];
    cx.G_LR.act.dq = &cx.state.dq[PIR_AXIS_L0];
    //cx.G_R.act.S = cx.state.S_wp_R;
    //cx.G_R.act.F = cx.state.F_R;
    cx.G_LR.ref.q =  &cx.ref.q[PIR_AXIS_L0];
    cx.G_LR.ref.dq = &cx.ref.dq[PIR_AXIS_L0];
    cx.G_LR.q_min = &cx.q_min[PIR_AXIS_L0];
    cx.G_LR.q_max = &cx.q_max[PIR_AXIS_L0];
    //cx.G_R.ref.S = AA_NEW0_AR( double, 8 );
    //cx.G_R.ref.F = AA_NEW0_AR( double, 6 );
    //cx.G_R.ref.dx = AA_NEW0_AR( double, 6 );
    //cx.G_R.act.dx = AA_NEW0_AR( double, 6 );
    //for( size_t i = 0; i < 3; i ++ ) {
        //cx.G_R.x_min[i] = -10;
        //cx.G_R.x_max[i] = 10;
    //}
    //cx.G_R.F_max = 20;

    // torso
    cx.G_T.n_q = 1;
    cx.G_T.J =  NULL;
    cx.G_T.act.q =  &cx.state.q[PIR_AXIS_T];
    cx.G_T.act.dq = &cx.state.dq[PIR_AXIS_T];
    cx.G_T.act.S = NULL;
    cx.G_T.act.F = NULL;
    cx.G_T.ref.q =  &cx.ref.q[PIR_AXIS_T];
    cx.G_T.ref.dq = &cx.ref.dq[PIR_AXIS_T];
    cx.G_T.q_min = &cx.q_min[PIR_AXIS_T];
    cx.G_T.q_max = &cx.q_max[PIR_AXIS_T];
    cx.G_T.ref.S = NULL;
    cx.G_T.ref.F = NULL;
    cx.G_T.ref.dx = NULL;
    cx.G_T.act.dx = NULL;
    for( size_t i = 0; i < 3; i ++ ) {
        cx.G_T.x_min[i] = -10;
        cx.G_T.x_max[i] = 10;
    }
    cx.G_T.F_max = 20;



    rfx_ctrl_ws_lin_k_init( &cx.Kx, 7 );
    aa_fset( cx.Kx.q, 0.1, 7 );
    cx.Kx.q[1] *= 5; // lower limits
    cx.Kx.q[3] *= 5; // lower limits
    cx.Kx.q[5] *= 5; // lower limits
    cx.Kx.q[6] *= 5; // this module is most sensitive to limits
    //aa_fset( cx.Kx.f, .003, 3 );
    //aa_fset( cx.Kx.f+3, .000, 3 );
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

    cx.Kq_lr.n_q = 14;
    cx.Kq_lr.p = AA_NEW_AR( double, 14 );
    AA_MEM_SET( cx.Kq_lr.p, 0, 14 );

    cx.Kq_T.n_q = 1;
    cx.Kq_T.p = AA_NEW_AR( double, 1 );
    AA_MEM_SET( cx.Kq_T.p, 0, 1 );

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
                if( msg->buttons & GAMEPAD_BUTTON_B ) {
                    if( strcmp(cx.msg_ctrl.mode, "halt") ) {
                        printf("HALT\n");
                    }
                    strcpy( cx.msg_ctrl.mode, "halt" );
                    memset(cx.ref.user, 0, sizeof(cx.ref.user[0])*JS_AXES);
                    cx.mode = NULL;
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
        printf( "ctrl_msg: `%s', %"PRIu64" (%"PRIu64")\n",
                msg_ctrl->mode, msg_ctrl->seq_no, msg_ctrl->salt );
        for( size_t i = 0; mode_desc[i].name != NULL; i ++ ) {
            if( 0 == strcmp(msg_ctrl->mode, mode_desc[i].name) ) {
                printf("found mode: %s\n", mode_desc[i].name);
                if( mode_desc[i].setmode ) {
                    if( 0 == mode_desc[i].setmode( &cx, msg_ctrl ) &&
                        mode_desc[i].ctrl )
                    {
                        memcpy( &cx.msg_ctrl, msg_ctrl, sizeof(cx.msg_ctrl) );
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
        cx.mode->ctrl( &cx );
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
