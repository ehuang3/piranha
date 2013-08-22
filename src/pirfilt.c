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
#include "piranha.h"



typedef struct {
    ach_channel_t chan_state_torso;
    ach_channel_t chan_state_left;
    ach_channel_t chan_state_right;
    ach_channel_t chan_ft_left;
    ach_channel_t chan_ft_right;
    ach_channel_t chan_ftbias_left;
    ach_channel_t chan_ftbias_right;
    ach_channel_t chan_state_pir;


    double F_raw_L[6]; ///< raw F/T reading, left
    double F_raw_R[6]; ///< raw F/T reading, right

    double r_ft_rel[4];    ///< Rotation from E.E. to F/T

    double r_ft_L[4];    ///< Absolute F/T rotation
    double r_ft_R[4];    ///< Absolute F/T rotation

    struct pir_state state;
    struct timespec now;

    double See[8];
    double S0[8];

    sig_atomic_t rebias;
} cx_t;

cx_t cx;

static void update(void);
static int update_n(size_t n, size_t i, ach_channel_t *chan, struct timespec *ts);

static void sighandler_hup ( int sig );
static int bias_ft( void );

static const double tf_ident[] = {1,0,0, 0,1,0, 0,0,1, 0,0,0};
static const double tf_0[] = {0,1,0, 0,0,-1, -1,0,0, 0,0,0};

int main( int argc, char **argv ) {
    memset(&cx, 0, sizeof(cx));

    assert( aa_tf_isrotmat( tf_0 ) );

    aa_tf_tfmat2duqu( tf_ident, cx.See );
    aa_tf_tfmat2duqu( tf_0, cx.S0 );


    /*-- args --*/
    for( int c; -1 != (c = getopt(argc, argv, "V?hH" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES;
        default:
            SNS_DIE( "Invalid argument: %s\n", optarg );
        }
    }

    sns_init();
    sns_start();

    // open channel
    sns_chan_open( &cx.chan_state_torso, "state-torso", NULL );
    sns_chan_open( &cx.chan_state_left,  "state-left",  NULL );
    sns_chan_open( &cx.chan_state_right, "state-right", NULL );
    sns_chan_open( &cx.chan_ft_left,      "ft-left",  NULL );
    sns_chan_open( &cx.chan_ft_right,     "ft-right", NULL );
    sns_chan_open( &cx.chan_ftbias_left,  "ft-bias-left",  NULL );
    sns_chan_open( &cx.chan_ftbias_right, "ft-bias-right", NULL );
    sns_chan_open( &cx.chan_state_pir,   "pir-state",  NULL );
    {
        ach_channel_t *chans[] = {&cx.chan_state_left, &cx.chan_state_torso, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    /*-- Init constants --*/
    {
        // F/T rotation
        double R0[9] = { 0, 1, 0,
                         0, 0, 1,
                         1, 0, 0 };
        assert(aa_tf_isrotmat(R0));
        double r0[4];
        aa_tf_rotmat2quat(R0, r0);

        //double Rrot[9];
        //aa_tf_zangle2rotmat(15*M_PI/180, Rrot);
        //aa_tf_9mul( R0, Rrot, cx.R_ft_rel );

        double r_rel[4];
        aa_tf_zangle2quat(15*M_PI/180, r_rel);
        aa_tf_qmul( r0, r_rel, cx.r_ft_rel );
    }


    /* Register signal handler */
    {
        struct sigaction act;
        memset(&act, 0, sizeof(act));
        act.sa_handler = &sighandler_hup;
        if( sigaction(SIGHUP, &act, NULL) ) {
            SNS_DIE( "Could not install signal handler\n");
        }
    }


    /* -- RUN -- */
    while (!sns_cx.shutdown) {
        update();
        if( cx.rebias ) {
            bias_ft();
        }
        aa_mem_region_local_release();
    }

    sns_end();
    return 0;
}

static int update_n(size_t n, size_t i, ach_channel_t *chan, struct timespec *ts ) {
    size_t frame_size;
    void *buf = NULL;
    ach_status_t r = sns_msg_local_get( chan, &buf,
                                        &frame_size,
                                        ts, ACH_O_LAST | (ts ? ACH_O_WAIT : 0) );

    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
    {
        struct sns_msg_motor_state *msg = (struct sns_msg_motor_state*)buf;
        // TODO: better validation
        if( n == msg->n &&
            frame_size == sns_msg_motor_state_size_n(n) )
        {
            for( size_t j = 0; j < n; j++ ) {
                if( fabs(cx.state.q[i+j] -  msg->X[j].pos) > 1*M_PI/180 ) {
                    printf("delta %"PRIuPTR": %f -> %f (%f)\n",
                           i+j,
                           cx.state.q[i+j], msg->X[j].pos,
                           cx.state.q[i+j] -  msg->X[j].pos);
                }
                cx.state.q[i+j] =  msg->X[j].pos;
                cx.state.dq[i+j] = msg->X[j].vel;
            }
            return 1;
        } else {
            SNS_LOG(LOG_ERR, "Invalid motor_state message\n");
        }
        break;
    }
    case ACH_TIMEOUT:
    case ACH_STALE_FRAMES:
    case ACH_CANCELED:
        break;
    default:
        SNS_LOG(LOG_ERR, "Failed ach_get: %s\n", ach_result_to_string(r) );
    }
    return 0;
}

static int update_ft(double *F, ach_channel_t *chan, struct timespec *ts ) {
    size_t frame_size;
    void *buf = NULL;
    ach_status_t r = sns_msg_local_get( chan, &buf,
                                        &frame_size,
                                        ts, ACH_O_LAST | (ts ? ACH_O_WAIT : 0) );
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
    {
        struct sns_msg_vector *msg = (struct sns_msg_vector*)buf;
        // TODO: better validation
        if( 6 == msg->n &&
            frame_size == sns_msg_vector_size_n(6) )
        {
            for( size_t i = 0; i < 6; i++ ) {
                F[i] = -msg->x[i];
            }
            return 1;
        } else {
            SNS_LOG(LOG_ERR, "Invalid F/T message\n");
        }
        break;
    }
    case ACH_TIMEOUT:
    case ACH_STALE_FRAMES:
    case ACH_CANCELED:
        break;
    default:
        SNS_LOG(LOG_ERR, "Failed ach_get: %s\n", ach_result_to_string(r) );
    }
    return 0;
}

static void rotate_ft( const double r_ft[4], const double *F_raw, double *F ) {
    // rotate
    aa_tf_qrot( r_ft, F_raw,   F);
    aa_tf_qrot( r_ft, F_raw+3, F+3 );

    // subtract end-effector mass
    F[2] -= PIR_FT_WEIGHT;

    // TODO: torque
}

static void update(void) {

    if( clock_gettime( ACH_DEFAULT_CLOCK, &cx.now ) )
        SNS_LOG( LOG_ERR, "clock_gettime failed: '%s'\n", strerror(errno) );

    struct timespec timeout = sns_time_add_ns( cx.now, 1000 * 1000 * 1 );
    int is_updated = 0;

    // axes
    int u_l = update_n(7, PIR_AXIS_L0, &cx.chan_state_left, &timeout);
    int u_r = update_n(7, PIR_AXIS_R0, &cx.chan_state_right, &timeout);
    is_updated = is_updated || u_l || u_r;

    // force-torque
    int u_fl = update_ft( cx.F_raw_L, &cx.chan_ft_left, &timeout );
    int u_fr = update_ft( cx.F_raw_R, &cx.chan_ft_right, &timeout );

    // update kinematics
    if( u_l ) {
        lwa4_kin_duqu( &cx.state.q[PIR_AXIS_L0], cx.S0, cx.See,
                       cx.state.S_L, cx.state.J_L  );
        aa_tf_qmul( cx.state.S_L, cx.r_ft_rel, cx.r_ft_L );
    }
    if( u_r ) {
        lwa4_kin_duqu( &cx.state.q[PIR_AXIS_R0], cx.S0, cx.See,
                       cx.state.S_R, cx.state.J_R  );
        aa_tf_qmul( cx.state.S_R, cx.r_ft_rel, cx.r_ft_R );
    }

    if( u_l || u_fl ) rotate_ft( cx.r_ft_L, cx.F_raw_L, cx.state.F_L );
    if( u_r || u_fr ) rotate_ft( cx.r_ft_R, cx.F_raw_R, cx.state.F_R );



    //if( u_l || u_fl ) aa_dump_vec( stdout, cx.state.F_L, 3 );

    // compute
    if( is_updated ) {

        // send
        ach_status_t r = ach_put( &cx.chan_state_pir, &cx.state,
                                  sizeof(cx.state) );

        if( ACH_OK != r ) {
            SNS_LOG( LOG_ERR, "Couldn't put ach frame: %s\n", ach_result_to_string(r) );
        }
    }
}


static int bias_ft( void ) {
    // F = -weight * k
    size_t n_msg = sns_msg_vector_size_n(6);
    struct sns_msg_vector *msg = (struct sns_msg_vector*) alloca(n_msg);
    msg->n = 6;
    sns_msg_header_fill(&msg->header);
    //sns_msg_set_time( &msg->header, &t_actual, 2*period_ns );

    // Expected force in absolute frame
    double F[6] = {0};
    F[2] = - PIR_FT_WEIGHT;

    // Rotate force to F/T frame
    // F_abs = R_ft * F_ft  => R_ft^T * F_abs = F_ft

    double r_ft_inv[4];
    aa_tf_qconj(cx.r_ft_L, r_ft_inv);
    aa_tf_qrot( r_ft_inv, F, msg->x );
    aa_tf_qrot( r_ft_inv, F+3, msg->x+3 );

    // send message
    ach_status_t r = ach_put( &cx.chan_ftbias_left, msg, n_msg );
    if( ACH_OK != r ) {
        SNS_LOG( LOG_ERR, "Couldn't send ach frame: %s\n", ach_result_to_string(r) );
    }

    // reset bias flag
    cx.rebias = 0;

    return 0;
}

static void sighandler_hup ( int sig ) {
    (void)sig;
    if( SNS_LOG_PRIORITY(LOG_DEBUG) ) {
        const char buf[] = "Received sighup\n";
        write(STDERR_FILENO, buf, sizeof(buf)); /* write is async safe, xprintf family is not */
    }
    cx.rebias = 1;
}
