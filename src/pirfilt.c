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
    ach_channel_t chan_state_pir;


    double F_raw_L[6]; ///< raw F/T reading, left
    double F_raw_R[6]; ///< raw F/T reading, right

    double R_ft[9];    ///< Rotation from E.E. to F/T

    struct pir_state state;
    struct timespec now;
} cx_t;

cx_t cx;

static void update(void);
static int update_n(size_t n, size_t i, ach_channel_t *chan, struct timespec *ts);

static const double tf_ident[] = {1,0,0, 0,1,0, 0,0,1, 0,0,0};

int main( int argc, char **argv ) {
    memset(&cx, 0, sizeof(cx));
    memcpy( cx.state.Tee, tf_ident, 12*sizeof(cx.state.Tee[0]) );
    memcpy( cx.state.T0, tf_ident, 12*sizeof(cx.state.T0[0]) );

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
    sns_chan_open( &cx.chan_state_torso, "state-torso", NULL );
    sns_chan_open( &cx.chan_state_left,  "state-left",  NULL );
    sns_chan_open( &cx.chan_state_right, "state-right", NULL );
    sns_chan_open( &cx.chan_ft_left,  "ft-left",  NULL );
    sns_chan_open( &cx.chan_ft_right, "ft-right", NULL );
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

        double Rrot[9];
        aa_tf_zangle2rotmat(15*M_PI/180, Rrot);
        aa_tf_9mul( R0, Rrot, cx.R_ft );

        //AA_MEM_CPY( cx.R_ft, R0, 9 );

    }

    /* -- RUN -- */
    while (!sns_cx.shutdown) {
        update();
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
                F[i] = msg->x[i];
            }
            return 1;
        } else {
            SNS_LOG(LOG_ERR, "Invalid F/T message\n");
        }
        break;
    }
    case ACH_TIMEOUT:
    case ACH_STALE_FRAMES:
        break;
    default:
        SNS_LOG(LOG_ERR, "Failed ach_get: %s\n", ach_result_to_string(r) );
    }
    return 0;
}

static void rotate_ft( const double *R_arm, const double *F_raw, double *F ) {
    double F_tmp[6];
    aa_tf_9rot( cx.R_ft, F_raw,   F_tmp );
    aa_tf_9rot( cx.R_ft, F_raw+3, F_tmp+3 );

    //aa_dump_vec( stdout, F_tmp, 3 );
    /* printf("%f\t%f\t%f\t|\t%f\t%f\t%f\n", */
    /*         F_raw[0], F_raw[1], F_raw[2], */
    /*         F_tmp[0], F_tmp[1], F_tmp[2] ); */


    aa_tf_9rot( R_arm, F_tmp,   F );
    aa_tf_9rot( R_arm, F_tmp+3, F+3 );
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
    if( u_l ) lwa4_kin_( &cx.state.q[PIR_AXIS_L0], cx.state.T0, cx.state.Tee,
                         cx.state.T_L, cx.state.J_L );
    if( u_r ) lwa4_kin_( &cx.state.q[PIR_AXIS_R0], cx.state.T0, cx.state.Tee,
                         cx.state.T_R, cx.state.J_R );

    if( u_l || u_fl ) rotate_ft( cx.state.T_L, cx.F_raw_L, cx.state.F_L );
    if( u_r || u_fr ) rotate_ft( cx.state.T_R, cx.F_raw_R, cx.state.F_R );

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
