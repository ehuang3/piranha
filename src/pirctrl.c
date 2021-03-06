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

int set_mode_bisplend(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
void ctrl_bisplend( pirctrl_cx_t *cx );

struct pir_mode_desc mode_desc[] = {
    {"left-shoulder",
     set_mode_cpy,
     ctrl_joint_left_shoulder,
     NULL},
    {"left-wrist",
     set_mode_cpy,
     ctrl_joint_left_wrist,
     NULL},
    {"right-shoulder",
     set_mode_cpy,
     ctrl_joint_right_shoulder,
     NULL},
    {"right-wrist",
     set_mode_cpy,
     ctrl_joint_right_wrist,
     NULL},
    {"ws-left",
     set_mode_ws_left,
     ctrl_ws_left,
     NULL},
    {"ws-left-finger",
     set_mode_ws_left_finger,
     ctrl_ws_left_finger,
     NULL},
    {"ws-right",
     set_mode_ws_right,
     ctrl_ws_right,
     NULL},
    {"ws-right-finger",
     set_mode_ws_right_finger,
     ctrl_ws_right_finger,
     NULL},
    {"zero",
     set_mode_cpy,
     ctrl_zero,
     NULL},
    {"sin",
     set_mode_sin,
     ctrl_sin,
     NULL},
    {"step",
     set_mode_cpy,
     ctrl_step,
     NULL},
    {"trajx-left",
     set_mode_trajx_left,
     ctrl_trajx_left,
     NULL},
    {"trajx-right",
     set_mode_trajx_right,
     ctrl_trajx_right,
     NULL},
    {"trajx-w-left",
     set_mode_trajx_w_left,
     ctrl_trajx_w_left,
     NULL},
    {"trajx-w-right",
     set_mode_trajx_w_right,
     ctrl_trajx_w_right,
     NULL},
    {"trajq-left",
     set_mode_trajq_left,
     ctrl_trajq_left,
     NULL},
    {"trajq-right",
     set_mode_trajq_right,
     ctrl_trajq_right,
     NULL},
    {"trajq-lr",
     set_mode_trajq_lr,
     ctrl_trajq_lr,
     NULL},
    {"trajq-torso",
     set_mode_trajq_torso,
     ctrl_trajq_torso,
     NULL},
    {"servo-cam",
     set_mode_servo_cam,
     ctrl_servo_cam,
     NULL},
    {"biservo-rel",
     set_mode_biservo_rel,
     ctrl_biservo_rel,
     NULL},
    {"bisplend",
     set_mode_bisplend,
     ctrl_bisplend,
     NULL},
    {"sdh-set-left",
     sdh_set_left,
     NULL,
     NULL},
    {"sdh-set-right",
     sdh_set_right,
     NULL,
     NULL},
    {"pinch-left",
     sdh_pinch_left,
     NULL,
     NULL},
    {"pinch-right",
     sdh_pinch_right,
     NULL,
     NULL},
    {"k-pt",
     set_mode_k_pt,
     NULL,
     NULL},
    {"k-pr",
     set_mode_k_pr,
     NULL,
     NULL},
    {"k-f",
     set_mode_k_f,
     NULL,
     NULL},
    {NULL, NULL, NULL, NULL} };


static const double tf_ident[] = {1,0,0, 0,1,0, 0,0,1, 0,0,0};

int main( int argc, char **argv ) {
    sns_init();
    memset(&cx, 0, sizeof(cx));
    cx.dt = 1.0 / 250;

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
    sns_chan_open( &cx.chan_config,       "pir-config",   NULL );
    sns_chan_open( &cx.chan_reg,          "pir-reg",      NULL );
    sns_chan_open( &cx.chan_reg_cam,      "pir-reg-cam",  NULL );
    sns_chan_open( &cx.chan_reg_ee,       "pir-reg-ee",   NULL );
    sns_chan_open( &cx.chan_complete,     "pir-complete", NULL );
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
        (void)sdh;
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
    AA_MEM_SET( cx.Kx.q, 0.1, 7 );
    cx.Kx.q[1] *= 5; // lower limits
    cx.Kx.q[3] *= 5; // lower limits
    cx.Kx.q[5] *= 5; // lower limits
    cx.Kx.q[6] *= 5; // this module is most sensitive to limits
    //AA_MEM_SET( cx.Kx.f, .003, 3 );
    //AA_MEM_SET( cx.Kx.f+3, .000, 3 );
    //AA_MEM_SET( cx.Kx.f, -.000, 6 );
    AA_MEM_SET( cx.Kx.p, 1.0, 3 );
    AA_MEM_SET( cx.Kx.p+3, 1.0, 3 );
    /* AA_MEM_SET( cx.K.p, 0.0, 3 ); */
    /* AA_MEM_SET( cx.K.p+3, 0.0, 3 ); */
    cx.Kx.dls = .005;
    cx.Kx.s2min = .01;
    printf("dls s2min: %f\n", cx.Kx.s2min);
    printf("dls k: %f\n", cx.Kx.dls);

    // joint
    cx.Kq.n_q = 7;
    cx.Kq.p = AA_NEW_AR( double, 7 );
    AA_MEM_SET( cx.Kq.p, .5, 7 );

    cx.Kq_lr.n_q = 14;
    cx.Kq_lr.p = AA_NEW_AR( double, 14 );
    AA_MEM_SET( cx.Kq_lr.p, .5, 14 );

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

#define CASE_HAVE_MSG        \
    case ACH_OK: ;           \
    case ACH_MISSED_FRAME    \


#define CASE_NO_MSG          \
    case ACH_CANCELED: ;     \
    case ACH_STALE_FRAMES: ; \
    case ACH_TIMEOUT         \

static void update(void) {
    // config
    {
        size_t frame_size;
        ach_status_t r = ach_get( &cx.chan_config, &cx.config, sizeof(cx.config), &frame_size,
                                  NULL, ACH_O_LAST );
        switch(r) {
        CASE_HAVE_MSG:
            SNS_REQUIRE( frame_size == sizeof(cx.config), "Invalid config size: %lu\n", frame_size );
            pir_kin( cx.config.q, &cx.tf_rel, &cx.tf_abs );
            break;
        CASE_NO_MSG: break;
        default:
            SNS_LOG(LOG_ERR, "Failed to get config: %s\n", ach_result_to_string(r) );
        }
    }

    // state
    {
        size_t frame_size;
        ach_status_t r = ach_get( &cx.chan_state_pir, &cx.state, sizeof(cx.state), &frame_size,
                                  NULL, ACH_O_LAST );
        switch(r) {
        CASE_HAVE_MSG: break;
        CASE_NO_MSG: break;
            break;
        default:
            SNS_LOG(LOG_ERR, "Failed to get frame: %s\n", ach_result_to_string(r) );
        }
    }

    // registration
    {
        size_t frame_size;
        struct sns_msg_tf *msg;
        enum ach_status r = sns_msg_tf_local_get( &cx.chan_reg, &msg, &frame_size, NULL, ACH_O_LAST );
        switch(r) {
        CASE_HAVE_MSG:
            if( 0 == sns_msg_tf_check_size(msg,frame_size) ) {
                if( 2 == msg->header.n ) {
                    AA_MEM_CPY( cx.bEc, msg->tf[1].data, 7 );
                } else {
                    SNS_LOG(LOG_ERR, "Unexpected registration count\n");
                }
            } else {
                SNS_LOG(LOG_ERR, "Invalid registration size\n");
            }
            break;
        CASE_NO_MSG: break;
        default:
            SNS_LOG(LOG_ERR, "Failed to get frame: %s\n", ach_result_to_string(r) );
        }
    }
    // registration 2
    {
        size_t frame_size;
        struct sns_msg_tf *msg;
        enum ach_status r = sns_msg_tf_local_get( &cx.chan_reg_cam, &msg, &frame_size, NULL, ACH_O_LAST );
        switch(r) {
        CASE_HAVE_MSG:
            if( 0 == sns_msg_tf_check_size(msg,frame_size) ) {
                if( msg->header.n != cx.n_bEc2 ) {
                    cx.bEc2 = (double*)realloc( cx.bEc2, sizeof(cx.bEc2[0]) * 7 * msg->header.n );
                    cx.n_bEc2 = msg->header.n;
                }
                AA_MEM_CPY( cx.bEc2, msg->tf[0].data, 7 * cx.n_bEc2 );
            } else {
                SNS_LOG(LOG_ERR, "Invalid msg size\n");
            }
            break;
        CASE_NO_MSG: break;
        default:
            SNS_LOG(LOG_ERR, "Failed to get frame: %s\n", ach_result_to_string(r) );
        }
    }
    // EE offset
    {
        size_t frame_size;
        struct sns_msg_tf *msg;
        enum ach_status r = sns_msg_tf_local_get( &cx.chan_reg_ee, &msg, &frame_size, NULL, ACH_O_LAST );
        switch(r) {
        CASE_HAVE_MSG:
            if( 0 == sns_msg_tf_check_size(msg,frame_size) ) {
                if( 2 == msg->header.n ) {
                    AA_MEM_CPY( cx.lElp, msg->tf[PIR_LEFT].data, 7 );
                    AA_MEM_CPY( cx.rErp, msg->tf[PIR_RIGHT].data, 7 );
                } else {
                    SNS_LOG(LOG_ERR, "Unexpected EE offset registration count\n");
                }
            } else {
                SNS_LOG(LOG_ERR, "Invalid msg size\n");
            }
            break;
        CASE_NO_MSG: break;
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
        CASE_HAVE_MSG:
            if( msg->header.n == JS_AXES &&
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
                    memcpy(cx.ref.user, msg->axis, sizeof(cx.ref.user[0])*msg->header.n);
                    cx.ref.user_button = msg->buttons;
                }
            }
        break;
        CASE_NO_MSG: break;
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
        printf( "ctrl_msg: `%s', seqno: %"PRIu64", salt: %"PRIu64", n: %"PRIu64"\n",
                msg_ctrl->mode, msg_ctrl->seq_no, msg_ctrl->salt,
                msg_ctrl->n
		);
        for( size_t i = 0; mode_desc[i].name != NULL; i ++ ) {
            if( 0 == strcmp(msg_ctrl->mode, mode_desc[i].name) ) {
                printf("found mode: %s\n", mode_desc[i].name);
                if( mode_desc[i].init ) {
                    if( 0 == mode_desc[i].init( &cx, msg_ctrl ) &&
                        mode_desc[i].run )
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
    if( cx.mode ) {
        if( cx.mode->term&&
            cx.mode->term(&cx) )
        {
            aa_mem_region_release(&cx.modereg);
            cx.mode = NULL;
        } else if ( cx.mode->run ) {
            cx.mode->run( &cx );
        }
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
    cx.msg_ref->header.n = n;
    ach_put( chan, cx.msg_ref, sns_msg_motor_ref_size(cx.msg_ref) );
    // TODO: check result
}
