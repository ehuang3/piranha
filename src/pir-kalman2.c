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
#include <signal.h>
#include "piranha.h"
#include <sys/types.h>
#include <unistd.h>

#define N_MARKERS 64


double opt_wt_thresh = 1;
size_t opt_k = 1;

#define N_MAX 8

//#define MK0 (32+5)
//#define MK1 (32+7)
//#define MK2 (32+8)
//#define MK3 (32+2)


#define MK_R_L0M 2
#define MK_R_L1M 1
#define MK_R_R0P 8
#define MK_R_R1P 3

//TODO: no copy paste
int marker2frame( size_t marker_id ) {
    switch(marker_id) {
    /* case 5:  return PIR_TF_RIGHT_SDH_L_K0M; */
    /* case 7:  return PIR_TF_RIGHT_SDH_L_K1M; */
    /* case 8:  return PIR_TF_RIGHT_SDH_R_K0P; */
    /* case 2:  return PIR_TF_RIGHT_SDH_R_K1P; */

    case MK_R_L0M:  return PIR_TF_RIGHT_SDH_L_K0M;
    case MK_R_L1M:  return PIR_TF_RIGHT_SDH_L_K1M;
    case MK_R_R0P:  return PIR_TF_RIGHT_SDH_R_K0P;
    case MK_R_R1P:  return PIR_TF_RIGHT_SDH_R_K1P;

        // TODO: find correspondences
    /*     /\* Left *\/ */
    /* case -1: return PIR_TF_LEFT_SDH_L_K0M; */
    //case 0: return PIR_TF_LEFT_SDH_L_K0P;
    /* case -1: return PIR_TF_LEFT_SDH_L_K1M; */
    /* case -1: return PIR_TF_LEFT_SDH_L_K1P; */

    /*     // T */
    /* case -1: return PIR_TF_LEFT_SDH_T_K0M; */
    /* case -1: return PIR_TF_LEFT_SDH_T_K0P; */
    /* case -1: return PIR_TF_LEFT_SDH_T_K1M; */
    /* case -1: return PIR_TF_LEFT_SDH_T_K1P; */

    /*     // R */
    //case 1: return PIR_TF_LEFT_SDH_R_K0M;
    /* case -1: return PIR_TF_LEFT_SDH_R_K0P; */
    /* case -1: return PIR_TF_LEFT_SDH_R_K1M; */
    //case 9: return PIR_TF_LEFT_SDH_R_K1P;

    /*     /\* Right *\/ */
    /*     // L */
    /* case -1: return PIR_TF_RIGHT_SDH_L_K0M; */
    /* case 0: return PIR_TF_RIGHT_SDH_L_K0P; */
    /* case -1: return PIR_TF_RIGHT_SDH_L_K1M; */
    /* case -1: return PIR_TF_RIGHT_SDH_L_K1P; */

    /*     // T */
    /* case -1: return PIR_TF_RIGHT_SDH_T_K0M; */
    /* case -1: return PIR_TF_RIGHT_SDH_T_K0P; */
    /* case -1: return PIR_TF_RIGHT_SDH_T_K1M; */
    /* case -1: return PIR_TF_RIGHT_SDH_T_K1P; */

    /*     // R */
    /* case 1: return PIR_TF_RIGHT_SDH_R_K0M; */
    /* case -1: return PIR_TF_RIGHT_SDH_R_K0P; */
    /* case 9: return PIR_TF_RIGHT_SDH_R_K1M; */
    /* case -1: return PIR_TF_RIGHT_SDH_R_K1P; */

    default: return -1;
    }
}


rfx_tf_dx XX_true;
rfx_tf_dx XX_est;
rfx_tf_dx ZZ;
rfx_tf_dx UU;


double Pb[13*13] = {0};
double Wb[7*7] = {0};
double Vb[13*13] = {0};

#define REC_RAW 0
#define REC_MED (REC_RAW+4)
#define REC_KF (REC_MED+1)

struct cor_samp {
    double bEk[7*N_MAX];
    double cEo[7*N_MAX];
    size_t n;
};

struct cor_samp *get_samp( ach_channel_t *chan_config,
                           ach_channel_t *chan_marker,
                           struct sns_msg_wt_tf **_wt_tf,
                           double **_tf_abs )
{
    struct cor_samp *samp = AA_MEM_REGION_LOCAL_NEW( struct cor_samp );
    // get marker
    struct sns_msg_wt_tf *wt_tf;
    {
        size_t marker_frame_size;
        ach_status_t r = sns_msg_wt_tf_local_get( chan_marker, &wt_tf,
                                                  &marker_frame_size, NULL, ACH_O_WAIT | ACH_O_LAST );
        SNS_REQUIRE( r == ACH_OK || r == ACH_MISSED_FRAME,
                     "Error getting markers: %s\n", ach_result_to_string(r) );
        SNS_REQUIRE( 0 == sns_msg_wt_tf_check_size(wt_tf, marker_frame_size),
                     "Invalid wt_tf message size: %lu \n", marker_frame_size );
    }
    // get config
    double *config;
    {
        size_t frame_size;
        ach_status_t r = sns_msg_local_get( chan_config, (void**)&config,
                                            &frame_size, NULL, ACH_O_WAIT | ACH_O_LAST );
        SNS_REQUIRE( r == ACH_OK || r == ACH_MISSED_FRAME,
                     "Error getting config: %s\n", ach_result_to_string(r) );
        size_t expected_size = 2*PIR_TF_CONFIG_MAX*sizeof(double);
        SNS_REQUIRE( expected_size == frame_size,
                     "Unexpected frame size: saw %lu, wanted %lu\n",
                     frame_size, expected_size );
    }

    // get kinematics
    double *tf_rel = AA_MEM_REGION_LOCAL_NEW_N( double, 7*PIR_TF_FRAME_MAX );
    double *tf_abs = AA_MEM_REGION_LOCAL_NEW_N( double, 7*PIR_TF_FRAME_MAX );
    pir_tf_rel( config, tf_rel );
    pir_tf_abs( tf_rel, tf_abs );

    // get correspondences
    samp->n = 0;
    for( size_t j = 0; j < wt_tf->header.n; j ++ ) {
        ssize_t frame = marker2frame(j);
        if( frame < 0 || wt_tf->wt_tf[j].weight < opt_wt_thresh ) continue;
        assert( samp->n < N_MAX );
        AA_MEM_CPY( AA_MATCOL(samp->bEk, 7, samp->n), AA_MATCOL(tf_abs,7,frame), 7 );
        AA_MEM_CPY( AA_MATCOL(samp->cEo, 7, samp->n), wt_tf->wt_tf[j].tf.data, 7 );
        samp->n++;
    }

    *_wt_tf = wt_tf;
    *_tf_abs = tf_abs;
    return samp;
}

int main( int argc, char **argv )
{
    sns_init();
    /* Parse */
    for( int c; -1 != (c = getopt(argc, argv, "?k:" )); ) {
        switch(c) {
        case 'k':
            opt_k = (size_t) atoi(optarg);
            break;
        case '?':   /* help     */
            puts( "Usage: pir-kalman\n"
                  "Kalman filter for piranha arm\n"
                  "\n"
                  "Options:\n"
                  "  -?,                Help text\n"
                  "\n"
                  "Report bugs to <ntd@gatech.edu>"
                );
            exit(EXIT_SUCCESS);
            break;
        default:
            printf("Unknown argument: `%s'\n", optarg);
            exit(EXIT_FAILURE);
        }
    }

    SNS_REQUIRE( opt_k > 0, "Must have positive sample count" );

    // init
    ach_channel_t chan_config, chan_marker, chan_reg, chan_reg_rec;
    sns_chan_open( &chan_config, "pir-config", NULL );
    sns_chan_open( &chan_marker, "markers", NULL );
    sns_chan_open( &chan_reg, "pir-reg", NULL );
    sns_chan_open( &chan_reg_rec, "pir-reg-rec", NULL );
    {
        ach_channel_t *chans[] = {&chan_reg, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    struct cor_samp *prev_samps = (struct cor_samp*)malloc(sizeof(struct cor_samp) *(size_t) opt_k);
    AA_MEM_ZERO( prev_samps, opt_k );

    aa_la_diag( 13, Pb, 1 );

    memset(&XX_true,0,sizeof(XX_true));
    memset(&XX_est,0,sizeof(XX_est));
    memset(&ZZ,0,sizeof(ZZ));
    memset(&UU,0,sizeof(UU));
    printf("foo\n");

    // run
    while( !sns_cx.shutdown ) {

        // sample
        double *tf_abs;
        struct sns_msg_wt_tf *wt_tf;
        struct cor_samp *samp = get_samp(&chan_config, &chan_marker, &wt_tf, &tf_abs);

        // shift into previous
        for( size_t i = 1; i < opt_k; i ++ ) {
            prev_samps[i-1] = prev_samps[i];
        }
        prev_samps[opt_k-1] = *samp;

        // collect data
        double *X = AA_MEM_REGION_LOCAL_NEW_N(double,7*opt_k*N_MAX);
        double *Y = AA_MEM_REGION_LOCAL_NEW_N(double,7*opt_k*N_MAX);
        size_t n = 0;
        for( size_t i = 0; i < opt_k; i++ ) {
            for( size_t j = 0; j < prev_samps[i].n; j++ ) {
                AA_MEM_CPY( AA_MATCOL(X,7,n), prev_samps[i].bEk, 7 );
                AA_MEM_CPY( AA_MATCOL(Y,7,n), prev_samps[i].cEo, 7 );
                n++;
            }
        }

        if( 0 == n ) {
            printf("none\n");
            // no markers
            continue;
        }

        printf("n: %d\n",n);
        // compute correspondence
        double E_cor[7];
        rfx_tf_cor( RFX_TF_COR_O_ROT_MEDIAN | RFX_TF_COR_O_TRANS_MEDIAN,
                    n,
                    X, 7, X+4, 7,
                    Y, 7, Y+4, 7,
                    E_cor );

        //aa_tf_qminimize(E_cor);

        // filter correspondence
        AA_MEM_CPY( ZZ.tf.data, E_cor, 7 );
        static int first = 1;
        if( first ) {
            first = 0;
            AA_MEM_CPY( XX_est.tf.data, E_cor, 7 );
        }
        double dt = 1e-1;
        rfx_lqg_qutr_process_noise( dt, 1, 1, XX_est.tf.data, Vb );
        aa_la_diag( 7, Wb, dt*1e1 );

        rfx_lqg_qutr_predict( dt, XX_est.tf.data, XX_est.dx.data, Pb, Vb );

        rfx_lqg_qutr_correct( 1,
                              XX_est.tf.data, XX_est.dx.data,
                              ZZ.tf.data,
                              Pb, Wb );

        // send message
        {
            struct sns_msg_tf *tfmsg = sns_msg_tf_local_alloc(2);
            struct timespec now;
            clock_gettime( ACH_DEFAULT_CLOCK, &now );
            sns_msg_set_time( &tfmsg->header, &now, 0 );
            AA_MEM_CPY( tfmsg->tf[0].data, E_cor, 7 );
            AA_MEM_CPY( tfmsg->tf[1].data, XX_est.tf.data, 7 );
            enum ach_status r = sns_msg_tf_put(&chan_reg, tfmsg);
            SNS_REQUIRE( r == ACH_OK, "Couldn't put message\n");
        }


        // extra message
        {
            struct sns_msg_tf *tfmsg = sns_msg_tf_local_alloc(6);
            struct timespec now;
            clock_gettime( ACH_DEFAULT_CLOCK, &now );
            sns_msg_set_time( &tfmsg->header, &now, 0 );
            size_t j[4] = {MK_R_L0M, MK_R_L1M, MK_R_R0P, MK_R_R1P};
            for( size_t i = 0; i < 4; i ++ ) {
                size_t frame = (size_t)marker2frame(j[i]);
                aa_tf_qutr_mulc( AA_MATCOL(tf_abs,7,frame),
                                 wt_tf->wt_tf[j[i]].tf.data,
                                 tfmsg->tf[REC_RAW+i].data );
            }
            AA_MEM_CPY( tfmsg->tf[REC_MED].data, E_cor, 7 );
            AA_MEM_CPY( tfmsg->tf[REC_KF].data, XX_est.tf.data, 7 );
            enum ach_status r = sns_msg_tf_put(&chan_reg_rec, tfmsg);
            SNS_REQUIRE( r == ACH_OK, "Couldn't put message\n");
        }
        aa_mem_region_local_release();
    }
}
