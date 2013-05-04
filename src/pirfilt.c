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
    int64_t i;
};

typedef struct {
    ach_channel_t chan_state_torso;
    ach_channel_t chan_state_left;
    ach_channel_t chan_state_pir;
    struct sns_msg_motor_state *msg_state;

    struct pir_state state;

    struct timespec now;
} cx_t;

cx_t cx;

static void update(void);
static void update_n(size_t n, size_t i, ach_channel_t *chan, struct timespec *ts);

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
            sns_die( 0, "Invalid argument: %s\n", optarg );
        }
    }

    sns_start();

    // open channel
    sns_chan_open( &cx.chan_state_torso, "state-torso", NULL );
    sns_chan_open( &cx.chan_state_left,  "state-left",  NULL );
    sns_chan_open( &cx.chan_state_pir,   "pir-state",  NULL );

    // alloc messages
    cx.msg_state = sns_msg_motor_state_alloc( PIR_MAX_MSG_AXES );

    /* -- RUN -- */
    while (!sns_cx.shutdown) {
        update();
    }

    sns_end();
    return 0;
}

static void update_n(size_t n, size_t i, ach_channel_t *chan, struct timespec *ts ) {
    size_t frame_size;
    size_t expected = sns_msg_motor_state_size_n(n);
    ach_status_t r = ach_get( chan, cx.msg_state,
                              expected, &frame_size,
                              ts, ACH_O_LAST | (ts ? ACH_O_WAIT : 0));
    // TODO: better validation
    if( (ACH_OK == r || ACH_MISSED_FRAME == r) &&
        n == cx.msg_state->n &&
        frame_size == expected )
    {
        for( size_t j = 0; j < n; j++ ) {
            cx.state.q[i+j] =  cx.msg_state->X[j].pos;
            cx.state.dq[i+j] = cx.msg_state->X[j].vel;
        }
    }
}

static void update(void) {

    if( clock_gettime( ACH_DEFAULT_CLOCK, &cx.now ) )
        SNS_LOG( LOG_ERR, "clock_gettime failed: '%s'\n", strerror(errno) );

    struct timespec timeout = sns_time_add_ns( cx.now, 1000 * 1000 * 10 );

    // axes
    update_n(7, PIR_AXIS_L0, &cx.chan_state_left, &timeout);
    update_n(1, PIR_AXIS_T,  &cx.chan_state_torso, NULL);

    // compute
    lwa4_kin_( &cx.state.q[PIR_AXIS_L0], cx.state.T0, cx.state.Tee,
               cx.state.T, cx.state.J );

    // send
    ach_status_t r = ach_put( &cx.chan_state_pir, &cx.state,
                              sizeof(cx.state) );

    if( ACH_OK != r ) {
        SNS_LOG( LOG_ERR, "Couldn't put ach frame: %s\n", ach_result_to_string(r) );
    }
}
