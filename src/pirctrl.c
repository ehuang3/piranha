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

enum pir_mode {
    MODE_HALT = 0,
    MODE_TORSO = 1,
    MODE_L_SHOULDER = 2,
    MODE_L_WRIST = 3,
    MODE_R_SHOULDER = 4,
    MODE_R_WRIST = 5,
};

struct pir_msg {
    char mode[64];
    union {
        int64_t i;
        double f;
    };
};

#define JS_AXES 8

typedef struct {
    ach_channel_t chan_js;
    ach_channel_t chan_ref_torso;
    ach_channel_t chan_ref_left;
    ach_channel_t chan_ref_right;
    ach_channel_t chan_state_pir;
    ach_channel_t chan_ctrl;
    double dt;

    struct sns_msg_motor_ref *msg_ref;
    struct pir_msg msg_ctrl;
    struct pir_state state;
    struct {
        double  q[PIR_AXIS_CNT];
        double dq[PIR_AXIS_CNT];
        double user[JS_AXES];
        uint64_t user_button;
    } ref;
    struct timespec now;
    rfx_ctrl_t G_L;
    rfx_ctrl_t G_R;
    rfx_ctrl_ws_lin_k_t K;
    double q_min[PIR_AXIS_CNT];
    double q_max[PIR_AXIS_CNT];

    double sint;
} cx_t;

cx_t cx;



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

static void control_n( uint32_t n, size_t i, ach_channel_t *chan );

static const double tf_ident[] = {1,0,0, 0,1,0, 0,0,1, 0,0,0};

int main( int argc, char **argv ) {
    memset(&cx, 0, sizeof(cx));
    memcpy( cx.state.Tee, tf_ident, 12*sizeof(cx.state.Tee[0]) );
    memcpy( cx.state.T0, tf_ident, 12*sizeof(cx.state.T0[0]) );
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
    sns_chan_open( &cx.chan_state_pir,   "pir-state",   NULL );
    {
        ach_channel_t *chans[] = {&cx.chan_state_pir, &cx.chan_js, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    // alloc messages
    cx.msg_ref = sns_msg_motor_ref_alloc( PIR_MAX_MSG_AXES );
    cx.msg_ref->mode = SNS_MOTOR_MODE_VEL;

    // setup reflex controller
    for( size_t i = 0; i < PIR_AXIS_CNT; i ++ ) {
        cx.q_min[i] = -2*M_PI;
        cx.q_max[i] = M_PI;
    }
    // left
    cx.G_L.n_q = 7;
    cx.G_L.J =  cx.state.J_L;
    cx.G_L.q =  &cx.state.q[PIR_AXIS_L0];
    cx.G_L.dq = &cx.state.dq[PIR_AXIS_L0];
    cx.G_L.q_r =  &cx.ref.q[PIR_AXIS_L0];
    cx.G_L.dq_r = &cx.ref.dq[PIR_AXIS_L0];
    cx.G_L.q_min = &cx.q_min[PIR_AXIS_L0];
    cx.G_L.q_max = &cx.q_max[PIR_AXIS_L0];
    for( size_t i = 0; i < 3; i ++ ) {
        cx.G_L.x_min[i] = -10;
        cx.G_L.x_max[i] = 10;
    }
    // right
    cx.G_R.n_q = 7;
    cx.G_R.J =  cx.state.J_R;
    cx.G_R.q =  &cx.state.q[PIR_AXIS_R0];
    cx.G_R.dq = &cx.state.dq[PIR_AXIS_R0];
    cx.G_R.q_r =  &cx.ref.q[PIR_AXIS_R0];
    cx.G_R.dq_r = &cx.ref.dq[PIR_AXIS_R0];
    cx.G_R.q_min = &cx.q_min[PIR_AXIS_R0];
    cx.G_R.q_max = &cx.q_max[PIR_AXIS_R0];
    for( size_t i = 0; i < 3; i ++ ) {
        cx.G_R.x_min[i] = -10;
        cx.G_R.x_max[i] = 10;
    }

    rfx_ctrl_ws_lin_k_init( &cx.K, 7 );
    aa_fset( cx.K.q, 0.001, 7 );
    cx.K.q[6] *= 5; // this module is most sensitive to limits
    aa_fset( cx.K.f, 0, 6 );
    aa_fset( cx.K.p, 0.5, 3 );
    aa_fset( cx.K.p+3, 0.5, 3 );
    cx.K.dls = .005;
    cx.K.s2min = .01;
    printf("dls s2min: %f\n", cx.K.s2min);
    printf("dls k: %f\n", cx.K.dls);


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
        if( 0 == strcmp( "k_s2min", msg_ctrl->mode ) ) {
            printf("k_s2min: %f\n", msg_ctrl->f );
            cx.K.s2min = msg_ctrl->f;
        } else if( 0 == strcmp( "k_pt", msg_ctrl->mode ) ) {
            printf("k_pt: %f\n", msg_ctrl->f );
            aa_fset( cx.K.p, msg_ctrl->f, 3 );
        } else if( 0 == strcmp( "k_pr", msg_ctrl->mode ) ) {
            printf("k_pr: %f\n", msg_ctrl->f );
            aa_fset( cx.K.p+3, msg_ctrl->f, 3 );
        } else if( 0 == strcmp( "k_q", msg_ctrl->mode ) ) {
            printf("k_q: %f\n", msg_ctrl->f );
            aa_fset( cx.K.q, msg_ctrl->f, 7 );
        } else {
            // actual mode setting
            memcpy( &cx.msg_ctrl, msg_ctrl, sizeof(cx.msg_ctrl) );
            cx.msg_ctrl.mode[63] = '\0';
            printf("ctrl: %s %"PRId64"\n", cx.msg_ctrl.mode, cx.msg_ctrl.i);

            if( 0 == strcmp("ws-left", cx.msg_ctrl.mode ) ) {
                AA_MEM_CPY( cx.G_L.x_r, &cx.state.T_L[9],  3 );
                aa_tf_rotmat2quat( cx.state.T_L, cx.G_L.r_r );
            }
            if( 0 == strcmp("ws-right", cx.msg_ctrl.mode ) ) {
                AA_MEM_CPY( cx.G_R.x_r, &cx.state.T_R[9],  3 );
                aa_tf_rotmat2quat( cx.state.T_R, cx.G_R.r_r );
            }
            if( 0 == strcmp("sin", cx.msg_ctrl.mode ) ) {
                cx.sint=0;
            }
        }
    }
}

static void control(void) {
    // dispatch
    memset( cx.ref.dq, 0, sizeof(cx.ref.dq[0])*PIR_AXIS_CNT );
    static const struct  {
        const char *name;
        ctrl_fun_t fun;
    } cmds[] = {
        {"left-shoulder", ctrl_joint_left_shoulder},
        {"left-wrist", ctrl_joint_left_wrist},
        {"right-shoulder", ctrl_joint_right_shoulder},
        {"right-wrist", ctrl_joint_right_wrist},
        {"ws-left", ctrl_ws_left},
        {"ws-right", ctrl_ws_right},
        {"zero", ctrl_zero},
        {"sin", ctrl_sin},
        {"step", ctrl_step},
        {NULL, NULL} };

    for( size_t i = 0; cmds[i].name != NULL; i ++ ) {
        if( 0 == strcmp(cx.msg_ctrl.mode, cmds[i].name) ) {
            cmds[i].fun();
            break;
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


static void ctrl_ws(size_t i, rfx_ctrl_ws_t *G, double *T) {
    // set actual FK
    AA_MEM_CPY( G->x, T+9, 3 );
    aa_tf_rotmat2quat( T, G->r );

    // set refs
    AA_MEM_SET( G->dx_r, 0, 6 );
    if( cx.ref.user_button & GAMEPAD_BUTTON_A ) {
        G->dx_r[3] = cx.ref.user[GAMEPAD_AXIS_LX] * .3;
        G->dx_r[4] = cx.ref.user[GAMEPAD_AXIS_LY] * .3;
        G->dx_r[5] = cx.ref.user[GAMEPAD_AXIS_RX] * .3;
    } else {
        G->dx_r[0] = cx.ref.user[GAMEPAD_AXIS_LX] * .1;
        G->dx_r[1] = cx.ref.user[GAMEPAD_AXIS_LY] * .1;
        G->dx_r[2] = cx.ref.user[GAMEPAD_AXIS_RX] * .1;
    }

    // compute stuff
    int r = rfx_ctrl_ws_lin_vfwd( G, &cx.K, &cx.ref.dq[i] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }

    // integrate
    aa_la_axpy(3, cx.dt, G->dx_r, G->x_r );
    double xr[3];
    double zero[3] = {0,0,0};
    aa_tf_quat2rotvec_near( G->r_r, zero, xr );
    aa_la_axpy(3, cx.dt, G->dx_r + 3, xr );
    aa_tf_rotvec2quat( xr, G->r_r );

}

static void ctrl_ws_left(void) {
    ctrl_ws( PIR_AXIS_L0, &cx.G_L, cx.state.T_L );
}

static void ctrl_ws_right(void) {
    ctrl_ws( PIR_AXIS_R0, &cx.G_R, cx.state.T_R );
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
