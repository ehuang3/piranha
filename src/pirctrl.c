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

/** Author: jscholz, Neil Dantam
 */

#include <argp.h>
#include <syslog.h>
#include <sns.h>
#include <signal.h>
#include <unistd.h>
#include <inttypes.h>


typedef struct {
    ach_channel_t chan_js;
    ach_channel_t chan_torso;
    struct sns_msg_joystick *jsmsg;
    struct sns_msg_motor_ref *torso_msg;
} cx_t;

cx_t cx;

#define MAXVEL_DEG_PER_SEC 20
#define JS_AXES 8

int main( int argc, char **argv ) {
    memset(&cx, 0, sizeof(cx));

    /*-- args --*/
    for( int c; -1 != (c = getopt(argc, argv, "V?hH" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES;
        default:
            sns_die( 0, "Invalid argument: %s\n", optarg );
        }
    }


    cx.jsmsg = sns_msg_joystick_alloc( JS_AXES );
    cx.torso_msg = sns_msg_motor_ref_alloc( 1 );
    cx.torso_msg->mode = SNS_MOTOR_MODE_VEL;

    sns_start();

    sns_chan_open( &cx.chan_js, "joystick", NULL );
    sns_chan_open( &cx.chan_torso, "ref-torso", NULL );

    /* -- RUN -- */
    while (!sns_cx.shutdown) {
        size_t frame_size;
        ach_status_t r = ach_get( &cx.chan_js, cx.jsmsg, sns_msg_joystick_size(cx.jsmsg), &frame_size,
                                  NULL, ACH_O_WAIT | ACH_O_LAST );
        SNS_REQUIRE( ACH_OK == r || ACH_MISSED_FRAME == r || ACH_TIMEOUT == r,
                     "Failed to get frame: %s\n", ach_result_to_string(r) );

        if( SNS_LOG_PRIORITY( LOG_DEBUG + 1 ) ) {
            sns_msg_joystick_dump( stderr, cx.jsmsg );
        }

        // validate js message
        if( cx.jsmsg->n == JS_AXES && frame_size == sns_msg_joystick_size(cx.jsmsg) ) {

            cx.torso_msg->u[0] = cx.jsmsg->axis[0] * MAXVEL_DEG_PER_SEC * M_PI / 180;
            r = ach_put( &cx.chan_torso, cx.torso_msg, sns_msg_motor_ref_size(cx.torso_msg) );
        }
    }

    sns_end();
    return 0;
}
