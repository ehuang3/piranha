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
size_t opt_k = 16;
size_t *opt_fixed_markers = NULL;
size_t opt_n_fixed_markers = 0;

size_t opt_n_cam = 0;
char **opt_cam = NULL;

#define N_MAX 8

#define MK_R_L0M 2
#define MK_R_L1M 1
#define MK_R_R0P 8
#define MK_R_R1P 3

#define MK_L_L0M 23
#define MK_L_L1M 24
#define MK_L_R0P 25
#define MK_L_R1P 26

int marker2frame( size_t marker_id ) {
    switch(marker_id) {
        // right
    case MK_R_L0M:  return PIR_TF_RIGHT_SDH_L_K0M;
    case MK_R_L1M:  return PIR_TF_RIGHT_SDH_L_K1M;
    case MK_R_R0P:  return PIR_TF_RIGHT_SDH_R_K0P;
    case MK_R_R1P:  return PIR_TF_RIGHT_SDH_R_K1P;

        // left
    case MK_L_L0M:  return PIR_TF_LEFT_SDH_L_K0M;
    case MK_L_L1M:  return PIR_TF_LEFT_SDH_L_K1M;
    case MK_L_R0P:  return PIR_TF_LEFT_SDH_R_K0P;
    case MK_L_R1P:  return PIR_TF_LEFT_SDH_R_K1P;

    default: return -1;
    }
}

struct madqg_state {
    double E[7];
    double dx[6];
    double *delta_theta;
    double *delta_x;
    size_t n;
    size_t max;
    size_t i;
    double P[13*13];
};

void init_state( struct madqg_state *state, size_t k ) {
    AA_MEM_CPY( state->E, aa_tf_quat_ident, 4 );
    AA_MEM_ZERO( state->E+4, 3 );
    AA_MEM_ZERO( state->dx, 6 );
    state->delta_theta = AA_NEW0_AR( double, k );
    state->delta_x = AA_NEW0_AR( double, k );
    state->n = 0;
    state->max = opt_k;
    state->i = 0;
    aa_la_diag( 13, state->P, 1 );
}

struct madqg_state *state_bEc; ///< camera poses
struct madqg_state *state_bEm; ///< fixed marker poses
struct madqg_state state_bEr;  ///< right-hand kinematic pose
struct madqg_state state_bEl;  ///< left-hand kinematic pose
struct madqg_state state_rErp; ///< right-hand correction
struct madqg_state state_lElp; ///< left-hand correction


double state_tf_rel[7*PIR_TF_FRAME_MAX];
double state_tf_abs[7*PIR_TF_FRAME_MAX];

double Pb[13*13] = {0};
double Wb[7*7] = {0};
double Vb[13*13] = {0};

#define REC_RAW 0
#define REC_MED (REC_RAW+4)
#define REC_KF (REC_MED+1)

int correct2( struct madqg_state *state, size_t n_obs,
              const double *bEm, const double *cEo )
{
    // FIXME: dt
    /* printf("correct2()\n"); */
    /* for( size_t i = 0; i < n_obs; i ++ ) { */
    /*     printf("bEm[%lu]: ", i); aa_io_d_print( stdout, 7, bEm + 7*i, 1 ); */
    /*     printf("cEo[%lu]: ", i); aa_io_d_print( stdout, 7, cEo + 7*i, 1 ); */
    /* } */
    //FIXME: dt
    int r = rfx_tf_madqg_correct2( 1,
                                   state->max, state->delta_theta, state->delta_x,
                                   &state->n, &state->i,
                                   state->E, state->dx,
                                   n_obs, bEm, cEo,
                                   state->P, Wb );
    //AA_MEM_ZERO( state->dx, 6 );
    if( r ) {
        SNS_LOG( LOG_ERR, "MADQG Correct2 failed: %d\n", r );
    }
    return r;
}

int correct1( struct madqg_state *state, const double *E_obs )
{
    return rfx_tf_madqg_correct( 0,
                                 state->max, state->delta_theta, state->delta_x,
                                 &state->n, &state->i,
                                 state->E, state->dx,
                                 1, E_obs,
                                 state->P, Wb );
}

int predict( double dt, struct madqg_state *state )
{

    rfx_lqg_qutr_process_noise( dt, 1, 10, state->E, Vb );
    return rfx_lqg_qutr_predict( dt, state->E, state->dx, state->P, Vb );
}

void update_camera( double *tf_abs, size_t i_cam, struct sns_msg_wt_tf *wt_tf )
{
    // Can initialze camera pose off of arm position, since we know
    // approximate E.E. pose in body frame from kinematics alone


    // FIXME: buffer overflow
    double cor_body[7*16];
    double cor_cam[7*16];
    size_t i_cor = 0;

    // TODO: E.E. relative
    for( size_t j = 0; j < wt_tf->header.n; j ++ ) {
        ssize_t frame = marker2frame(j);
        if( wt_tf->wt_tf[j].weight < opt_wt_thresh ) continue;
        if( frame  ) {
            /**** ARM ****/
            AA_MEM_CPY( AA_MATCOL(cor_body, 7, i_cor), AA_MATCOL(tf_abs,7,frame), 7 );
            AA_MEM_CPY( AA_MATCOL(cor_cam, 7, i_cor), wt_tf->wt_tf[j].tf.data, 7 );
            i_cor++;
        } else {
            /**** FIXED MARKERS ****/
            for( size_t k = 0; k < opt_n_fixed_markers; k ++ ) {
                if( opt_fixed_markers[k] == j ) {
                    // found kth marker
                    // use as a correspondance
                    AA_MEM_CPY( cor_body+7*i_cor, state_bEm[k].E, 7 );
                    AA_MEM_CPY( cor_cam+7*i_cor, wt_tf->wt_tf[j].tf.data, 7 );
                    i_cor++;
                    double E_obs[7];
                    aa_tf_qutr_mul( state_bEc[i_cam].E, wt_tf->wt_tf[j].tf.data, E_obs );
                    correct1( state_bEm+k, E_obs );
                }
            }
        }
    }
    /* Correct Camera */
    correct2( state_bEc+i_cam, i_cor, cor_body, cor_cam );
}

void update( ach_channel_t *chan_config, ach_channel_t *chan_cam, size_t n_cam )
{
    SNS_LOG( LOG_DEBUG + 2, "update()\n");
    /**** KINEMATICS ****/
    /* Wait Sample */
    double *config = NULL;
    ach_status_t r_config;
    {
        size_t frame_size;
        r_config = sns_msg_local_get( chan_config, (void**)&config,
                                      &frame_size, NULL, ACH_O_WAIT | ACH_O_LAST );
        SNS_LOG( LOG_DEBUG + 2, "r_config: %s\n", ach_result_to_string(r_config));
        switch( r_config ) {
        case ACH_OK:
        case ACH_MISSED_FRAME:
        case ACH_STALE_FRAMES:
            break;
        case ACH_CANCELED:
            SNS_LOG(LOG_NOTICE, "canceled\n");
            return;
        default:
            SNS_DIE( "Error getting config: %s\n", ach_result_to_string(r_config) );
        }
        /* Maybe update kinematics */
        if( config ) {
            SNS_LOG( LOG_DEBUG + 2, "got config\n");
            size_t expected_size = 2*PIR_TF_CONFIG_MAX*sizeof(double);
            SNS_REQUIRE( expected_size == frame_size,
                         "Unexpected frame size: saw %lu, wanted %lu\n",
                         frame_size, expected_size );
            pir_tf_rel( config, state_tf_rel );
            pir_tf_abs( state_tf_rel, state_tf_abs );
            AA_MEM_CPY( state_bEl.E, state_tf_abs+7*PIR_TF_LEFT_WRIST0,7 );
            AA_MEM_CPY( state_bEr.E, state_tf_abs+7*PIR_TF_LEFT_WRIST0,7 );
        }
    }

    /* PREDICT */
    SNS_LOG( LOG_DEBUG + 2, "update(): predict\n");
    double dt = 1e-3; // FIXME
    for( size_t i = 0; i < n_cam; i ++ ) {
        predict( dt, state_bEc );
    }
    for( size_t i = 0; i < opt_n_fixed_markers; i ++ ) {
        predict( dt, state_bEm );
    }
    predict( dt, &state_bEr );
    predict( dt, &state_bEl );
    /* predict( dt, &state_rErp ); */
    /* predict( dt, &state_lElp ); */

    /**** CAMERAS ****/
    SNS_LOG( LOG_DEBUG + 2, "update(): cam\n");
    /* Get Sample */
    for( size_t i = 0; i < n_cam; i ++ ) {
        struct sns_msg_wt_tf *wt_tf = NULL;
        /* Get message */
        size_t marker_frame_size;
        ach_status_t r = sns_msg_wt_tf_local_get( &chan_cam[i], &wt_tf,
                                                  &marker_frame_size, NULL, ACH_O_LAST );
        switch( r ) {
        case ACH_OK:
        case ACH_MISSED_FRAME:
        case ACH_STALE_FRAMES:
            break;
        default:
            SNS_DIE( "Error getting marker: %s\n", ach_result_to_string(r) );
        }
        /* CORRECT */
        if( wt_tf ) {
            ssize_t size_delta = sns_msg_wt_tf_check_size( wt_tf, marker_frame_size );
            if( size_delta ) {
                SNS_LOG( LOG_ERR, "Bad frame size, delta of %ld\n", size_delta);
            } else {
                SNS_LOG( LOG_DEBUG + 1, "got cam: %lu (%s)\n", i, ach_result_to_string(r));
                update_camera( state_tf_abs, i, wt_tf );
            }
        }
    }

    /* CLEANUP */
    aa_mem_region_local_release();
}

int output_chan( struct timespec now,
                 ach_channel_t *chan,
                 struct madqg_state *state,
                 size_t n )
{
    struct sns_msg_tf *msg = sns_msg_tf_local_alloc( (uint32_t) n );
    sns_msg_set_time( &msg->header, &now, 0 );
    for( size_t i = 0; i < n; i ++ )
        AA_MEM_CPY( msg->tf[i].data, state[i].E, 7 );

    enum ach_status r = sns_msg_tf_put(chan, msg);
    SNS_REQUIRE( ACH_OK == r, "Couldn't put message\n");
    return 0;
}

int output( ach_channel_t *chan_reg_cam, ach_channel_t *chan_reg_marker, ach_channel_t *chan_reg_ee  ) {
    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    SNS_LOG( LOG_DEBUG+2, "output()\n");

    output_chan( now, chan_reg_cam, state_bEc, opt_n_cam );
    output_chan( now, chan_reg_marker, state_bEc, opt_n_fixed_markers );
    //output_chan( now, chan_reg_e, state_bEc, opt_n_fixed_markers );

    /* struct sns_msg_tf *tf_cam = sns_msg_tf_local_alloc( (uint32_t) opt_n_cam ); */
    /* struct sns_msg_tf *tf_marker = sns_msg_tf_local_alloc( (uint32_t) opt_n_fixed_markers ); */
    /* struct sns_msg_tf *tf_ee = sns_msg_tf_local_alloc( 2 ); */

    /* sns_msg_set_time( &tfmsg->header, &now, 0 ); */

    /* AA_MEM_CPY( tfmsg->tf[0].data, state_bEl.E, 7 ); */
    /* AA_MEM_CPY( tfmsg->tf[1].data, state_bEr.E, 7 ); */
    /* for( size_t i = 0; i < opt_n_cam; i ++ ) */
    /*     AA_MEM_CPY( tfmsg->tf[2+i].data, state_bEc[i].E, 7 ); */
    /* for( size_t i = 0; i < opt_n_fixed_markers; i ++ ) */
    /*     AA_MEM_CPY( tfmsg->tf[2+opt_n_cam+i].data, state_bEm[i].E, 7 ); */

    /* enum ach_status r = sns_msg_tf_put(chan_reg, tfmsg); */
    /* SNS_REQUIRE( r == ACH_OK, "Couldn't put message\n"); */

    return 0;
}


int main( int argc, char **argv )
{
    sns_init();
    /* Parse */
    for( int c; -1 != (c = getopt(argc, argv, "?k:c:" SNS_OPTSTRING )); ) {
        switch(c) {
            SNS_OPTCASES
        case 'k':
            opt_k = (size_t) atoi(optarg);
            break;
        case 'c':
            opt_cam = (char**)realloc( opt_cam, (sizeof(char*)) * ++opt_n_cam );
            opt_cam[opt_n_cam-1] = optarg;
            break;
        case '?':   /* help     */
            puts( "Usage: pir-kalman\n"
                  "Kalman filter for piranha arm\n"
                  "\n"
                  "Options:\n"
                  "  -k number          MAD windown count\n"
                  "  -c channel         Channel to read markers from\n"
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

    for( size_t i = 0; i < opt_n_cam; i ++ ) {
        SNS_LOG( LOG_DEBUG, "marker channel %lu: %s\n",
                 i, opt_cam[i] );
    }
    SNS_LOG( LOG_DEBUG, "%lu cameras\n", opt_n_cam);
    SNS_LOG( LOG_DEBUG, "%lu fixed markers\n", opt_n_fixed_markers);

    // init
    ach_channel_t chan_config, *chan_marker, chan_reg_cam, chan_reg_marker, chan_reg_ee;
    sns_chan_open( &chan_config, "pir-config", NULL );
    sns_chan_open( &chan_reg_cam, "pir-reg-cam", NULL );
    sns_chan_open( &chan_reg_marker, "pir-reg-marker", NULL );
    sns_chan_open( &chan_reg_ee, "pir-reg-ee", NULL );

    chan_marker = (ach_channel_t*)malloc( sizeof(ach_channel_t) * opt_n_cam );
    for( size_t i = 0; i < opt_n_cam; i ++ ) {
        sns_chan_open( &chan_marker[i], opt_cam[i], NULL );
    }

    {
        ach_channel_t *chans[] = {&chan_config, NULL, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    SNS_LOG( LOG_DEBUG, "initializing state structs\n");
    state_bEc = AA_NEW_AR(struct madqg_state, opt_n_cam);
    for( size_t i = 0; i < opt_n_cam; i ++ ) {
        init_state( &state_bEc[i], opt_k );
    }

    state_bEm = AA_NEW_AR(struct madqg_state, opt_n_fixed_markers);
    for( size_t i = 0; i < opt_n_fixed_markers; i ++ ) {
        init_state( &state_bEm[i], opt_k );
    }
    init_state( &state_bEl, opt_k );
    init_state( &state_bEr, opt_k );
    init_state( &state_lElp, opt_k );
    init_state( &state_rErp, opt_k );

    // FIXME: dt
    aa_la_diag( 7, Wb, .5 );
    aa_la_diag( 13, Pb, 10 );

    SNS_LOG( LOG_INFO, "starting main loop\n");
    // run
    while( !sns_cx.shutdown ) {

        update( &chan_config, chan_marker, opt_n_cam );
        output( &chan_reg_cam, &chan_reg_marker, &chan_reg_ee );

        aa_mem_region_local_release();
    }
    SNS_LOG( LOG_NOTICE, "Exiting gracefully\n");
    return 0;
}
