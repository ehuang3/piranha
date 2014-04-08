/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
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

#include <amino.h>
#include <sns.h>
#include <getopt.h>
#include <reflex.h>
#include "piranha.h"

#define N_MARKERS 32

int dump_tf( size_t i, double *tf_abs )
{
    printf("%s: ", pir_tf_names[i]);
    aa_dump_vec( stdout, &tf_abs[7*i], 7 );
    return 0;
}

int main( int argc, char **argv )
{
    (void) argc; (void)argv;
    sns_init();
    ach_channel_t chan_config;
    sns_chan_open( &chan_config, "pir-config", NULL );

    while(!sns_cx.shutdown) {

        double *config;
        // get config
        {
            size_t frame_size;
            ach_status_t r = sns_msg_local_get( &chan_config, (void**)&config,
                                                &frame_size, NULL, ACH_O_WAIT | ACH_O_LAST );
            SNS_REQUIRE( r == ACH_OK || r == ACH_MISSED_FRAME,
                         "Error getting config: %s\n", ach_result_to_string(r) );
            size_t expected_size = 2*PIR_TF_CONFIG_MAX*sizeof(double);
            SNS_REQUIRE( expected_size == frame_size,
                         "Unexpected frame size: saw %lu, wanted %lu\n",
                         frame_size, expected_size );

        }

        // compute TFs
        double *tf_rel = (double*)aa_mem_region_local_alloc( 7 * PIR_TF_FRAME_MAX * sizeof(tf_rel[0]) );
        double *tf_abs = (double*)aa_mem_region_local_alloc( 7 * PIR_TF_FRAME_MAX * sizeof(tf_abs[0]) );
        pir_tf_rel( config, tf_rel );
        pir_tf_abs( tf_rel, tf_abs );

        printf("--\n");
        dump_tf( PIR_TF_RIGHT_SDH_L_2, tf_abs );
        dump_tf( PIR_TF_RIGHT_SDH_L_K0P, tf_abs );
        dump_tf( PIR_TF_RIGHT_SDH_R_2, tf_abs );
        dump_tf( PIR_TF_RIGHT_SDH_R_K0M, tf_abs );
        dump_tf( PIR_TF_RIGHT_SDH_R_K1M, tf_abs );

        // write data
        aa_mem_region_local_release();
    }
}
