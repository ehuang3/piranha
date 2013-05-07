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
#include <socanmatic.h>
#include <reflex.h>
#include "piranha.h"


int main( int argc, char **argv ) {
    // open iface
    struct canmat_iface *cif = canmat_iface_new( "ntcan" );
    SNS_REQUIRE( cif, "Couldn't create interface of type: %s\n", "ntcan" );

    {
        canmat_status_t r = canmat_iface_open( cif, "8" );
        SNS_REQUIRE( CANMAT_OK == r, "Couldn't open interface: %s\n", canmat_iface_strerror(cif, r) );
    }

    // read and check
    while(1) {
        struct can_frame can = {0};
        canmat_status_t r = canmat_iface_recv( cif, &can );
        SNS_REQUIRE( CANMAT_OK == r, "Couldn't get frame: %s\n", canmat_iface_strerror(cif, r) );

        if( 0x300 == (can.can_id & ~CANMAT_NODE_MASK) ) {
            SNS_REQUIRE( can.can_dlc >= 2, "Frame too short\n");
            uint8_t node = can.can_id & CANMAT_NODE_MASK;
            int16_t raw = can.data[0] | (can.data[1] << 8);
            double vel_deg_s = raw / 1000.0;
            if( fabs(vel_deg_s) > 10 ) {
                printf("%x: %f\n", node, vel_deg_s );
            }
        }
    }

    return 0;
}
