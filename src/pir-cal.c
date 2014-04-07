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

//const char *opt_file_cam = NULL;
//const char *opt_file_fk = NULL;
//const char *opt_file_out = NULL;
//size_t opt_test = 0;
//int opt_verbosity = 0;
//double opt_d_theta = 0;
//double opt_d_x = 0;

const char *opt_file_config = NULL;
const char *opt_file_marker = NULL;

const char *opt_file_cam = NULL;
const char *opt_file_fk = NULL;
const char *opt_file_id = NULL;

const char *opt_chan_take = "pir-cal-take";
const char *opt_chan_done = "pir-cal-done";

double opt_wt_thresh = 1;
int opt_samples = 1;

int opt_run = 0;
int opt_comp = 0;

sig_atomic_t is_signaled = 0;


static void
run_cal( void );

static void
compute_cal( void );


/* static void sighandler( int sig ) { */
/*     (void)sig; */
/*     is_signaled = 1; */
/* } */


/* static void sigregister( void ) { */
/*     struct sigaction act; */
/*     memset(&act, 0, sizeof(act)); */
/*     act.sa_handler = &sighandler; */
/*     if( sigaction(SIGUSR1, &act, NULL) ) { */
/*         SNS_DIE( "Could not install signal handler\n"); */
/*     } */
/* } */


int main( int argc, char **argv )
{
    /* Parse */
    for( int c; -1 != (c = getopt(argc, argv, "q:m:RCc:k:i:n:")); ) {
        switch(c) {
        case 'q':
            opt_file_config = optarg;
            break;
        case 'm':
            opt_file_marker = optarg;
            break;
        case 'c':
            opt_file_cam = optarg;
            break;
        case 'k':
            opt_file_fk = optarg;
            break;
        case 'i':
            opt_file_id = optarg;
            break;
        case 'R':
            opt_run = 1;
            break;
        case 'C':
            opt_comp = 1;
            break;
        case 'n':
            opt_samples = atoi(optarg);
            break;
        case '?':   /* help     */
            puts( "Usage: rfx-camcal -k FK_POSE_FILE -c CAM_POSE_FILE \n"
                  "Calibrate a camera from list of kinematics and camera transforms"
                  "\n"
                  "Options:\n"
                  "  -q CONFIG-FILE,             Config file\n"
                  "  -m MARKER-FILE,             Marker file\n"
                  "  -c CAM-FILE,                Marker Pose file\n"
                  "  -k FK-FILE,                 FK Pose file\n"
                  "  -i ID-FILE,                 Frame id file\n"
                  "  -n samples,                 Number of samples to take\n"
                  "  -R,                         Run a calibration\n"
                  "  -C,                         Compute a calibration\n"
                  "\n"
                  "\n"
                  "Examples:\n"
                  "pir-cal -q config.dat -m marker.dat -R -n 15            Run a calibration\n"
                  "\n"
                  "pir-cal -q config.dat -m marker.dat -C -c cam.dat -k fk.dat -i id.dat  Compute calibration\n"
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
    if( opt_run ) {
        run_cal();
    }

    if( opt_comp ) {
        compute_cal();
    }

    return 0;
}

static FILE *
open_out( const char *name  )
{
    if( NULL == name || 0 == strcmp("-",name) ) return stdout;
    FILE *f = fopen( name, "w" );
    SNS_REQUIRE( NULL != f, "Error opening %s\n", name );
    return f;
}


static void
run_cal( void )
{
    sns_init();
    //sigregister();
    ach_channel_t chan_config, chan_marker, chan_take, chan_done;
    sns_chan_open( &chan_config, "pir-config", NULL );
    sns_chan_open( &chan_marker, "pir-marker", NULL );
    sns_chan_open( &chan_take, "pir-cal-take", NULL );
    sns_chan_open( &chan_done, "pir-cal-done", NULL );

    {
        ach_channel_t *chans[] = {&chan_take, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }


    FILE *f_q = fopen( opt_file_config, "w" );
    SNS_REQUIRE( NULL != f_q, "Error opening %s\n", opt_file_config );

    FILE *f_m = fopen( opt_file_marker, "w" );
    SNS_REQUIRE( NULL != f_m, "Error opening %s\n", opt_file_marker );

    printf("COLLECTING CALIBRATION DATA\n"
           "===========================\n" );

    //char *lineptr = NULL;
    //size_t n = 0;
    //pid_t my_pid = getpid();
    while(!sns_cx.shutdown) {
        printf("> Send message sample\n");

        void *take_buf;
        size_t take_frame_size;
        {
            enum ach_status r = sns_msg_local_get( &chan_take, &take_buf, &take_frame_size,
                                                   NULL,
                                                   ACH_O_WAIT | ACH_O_LAST );
            switch(r) {
            case ACH_OK:
            case ACH_MISSED_FRAME:
            case ACH_CANCELED:
                break;
            default:
                SNS_DIE("Ach error: `%s'\n", ach_result_to_string(r));
            }
        }

        if( sns_cx.shutdown ) break;
        for( int i = 0; i < opt_samples; i ++ ) {
        // get marker
        {
            size_t frame_size;
            struct sns_msg_wt_tf *wt_tf;
            ach_status_t r = sns_msg_wt_tf_local_get( &chan_marker, &wt_tf,
                                                      &frame_size, NULL, ACH_O_WAIT | ACH_O_LAST );
            SNS_REQUIRE( r == ACH_OK || r == ACH_MISSED_FRAME,
                         "Error getting markers: %s\n", ach_result_to_string(r) );
            SNS_REQUIRE( frame_size > sizeof( sns_msg_header_t ),
                         "Invalid wt_tf message size, too small\n" );
            SNS_REQUIRE( frame_size == sns_msg_wt_tf_size(wt_tf),
                         "Invalid wt_tf message size: actual %lu, indicated %u\n",
                         frame_size, sns_msg_wt_tf_size(wt_tf) );

            aa_dump_vec( f_m, (double*)&(wt_tf->wt_tf[0]), wt_tf->header.n*8 );
        }
        printf("  sample %d: got marker\n", i);
        // get config
        {
            size_t frame_size;
            double *config;
            ach_status_t r = sns_msg_local_get( &chan_config, (void**)&config,
                                                &frame_size, NULL, ACH_O_WAIT | ACH_O_LAST );
            SNS_REQUIRE( r == ACH_OK || r == ACH_MISSED_FRAME,
                         "Error getting config: %s\n", ach_result_to_string(r) );
            size_t expected_size = 2*PIR_TF_CONFIG_MAX*sizeof(double);
            SNS_REQUIRE( expected_size == frame_size,
                         "Unexpected frame size: saw %lu, wanted %lu\n",
                         frame_size, expected_size );

            aa_dump_vec( f_q, config, PIR_TF_CONFIG_MAX );

            // get marker
        }
        printf("  sample %d: got config\n", i);
        }
        // send done
        {
            enum ach_status r = ach_put( &chan_done, take_buf, take_frame_size );
            SNS_REQUIRE( ACH_OK == r, "Error putting done message: `%s'\n", ach_result_to_string(r) );
        }

        // write data
        aa_mem_region_local_release();
    }
    printf("Terminating.\n");
    fclose(f_q);
    fclose(f_m);
}


struct marker_pair {
    int fk_frame;
    int marker_id;
};

int marker2frame( size_t marker_id ) {
    switch(marker_id) {
    /* case 5:  return PIR_TF_RIGHT_SDH_L_K0M; */
    /* case 7:  return PIR_TF_RIGHT_SDH_L_K1M; */
    /* case 8:  return PIR_TF_RIGHT_SDH_R_K0P; */
    /* case 2:  return PIR_TF_RIGHT_SDH_R_K1P; */

    case 32+5:  return PIR_TF_RIGHT_SDH_L_K0M;
    case 32+7:  return PIR_TF_RIGHT_SDH_L_K1M;
    case 32+8:  return PIR_TF_RIGHT_SDH_R_K0P;
    case 32+2:  return PIR_TF_RIGHT_SDH_R_K1P;

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

static void
compute_cal( void )
{
    // open files
    FILE *f_q = fopen( opt_file_config, "r" );
    SNS_REQUIRE( NULL != f_q, "Error opening %s\n", opt_file_config );

    FILE *f_m = fopen( opt_file_marker, "r" );
    SNS_REQUIRE( NULL != f_m, "Error opening %s\n", opt_file_marker );

    FILE *f_c = open_out( opt_file_cam );
    FILE *f_k = open_out( opt_file_fk );
    FILE *f_i = open_out( opt_file_id );

    double *Q, *M;
    size_t lines;
    size_t marker_elts = N_MARKERS * 8;
    {
        ssize_t lines_q = aa_io_fread_matrix_heap( f_q, PIR_TF_CONFIG_MAX, &Q, NULL);
        ssize_t lines_m = aa_io_fread_matrix_heap( f_m, marker_elts,
                                                   &M, NULL);
        SNS_REQUIRE( lines_q > 0 && lines_m > 0 && lines_q == lines_m,
                     "Error reading files: config-lines: %ld, marker-lines: %ld\n",
                     lines_q, lines_m );
        lines = (size_t)lines_q;
    }

    printf("Read %lu lines\n", lines );

    // test markers
    printf("Using frames:\n");
    for( size_t j = 0; j < N_MARKERS; j ++ ) {
        ssize_t frame = marker2frame(j);
        if( frame > 0 ) printf(" %s with marker %ld\n", pir_tf_names[frame], j );
    }


    // loop through lines and write correspondenses
    int output = 0;
    for( size_t i = 0; i < lines; i ++ )
    {
        //printf("Line %lu:\n", i+1 );
        double *q = &Q[i*PIR_TF_CONFIG_MAX];
        sns_wt_tf *wt_tf = (sns_wt_tf*) AA_MATCOL(M,marker_elts,i);

        double *tf_rel = (double*)aa_mem_region_local_alloc( 7 * PIR_TF_FRAME_MAX * sizeof(tf_rel[0]) );
        double *tf_abs = (double*)aa_mem_region_local_alloc( 7 * PIR_TF_FRAME_MAX * sizeof(tf_abs[0]) );
        pir_tf_rel( q, tf_rel );
        pir_tf_abs( tf_rel, tf_abs );

        for( size_t j = 0; j < N_MARKERS; j ++ ) {
            ssize_t frame = marker2frame(j);
            double norm = aa_tf_qnorm( wt_tf[j].tf.r.data );
            if( frame > 0 &&
                wt_tf[j].weight >= opt_wt_thresh &&
                aa_feq( norm, 1, 1e-3 ))
            {
                output++;
                //printf("\t%s / marker %ld\n", pir_tf_names[frame], j );
                fprintf(f_c, "# %d: (marker) %s / marker %ld\n", output, pir_tf_names[frame], j );
                aa_dump_vec( f_c, wt_tf[j].tf.data, 7);
                fprintf(f_k, "# %d: (fk) %s / marker %ld\n", output, pir_tf_names[frame], j );
                aa_dump_vec( f_k, &tf_abs[7*frame], 7);
                fprintf(f_i,"%ld\n", frame);
            }
        }
        aa_mem_region_local_pop(tf_rel);
    }

    SNS_REQUIRE( feof(f_q) && feof(f_m),
                 "Error reading marker and config files\n" );

    fclose(f_c);
    fclose(f_k);
    fclose(f_i);
}


/* static struct marker_pair pairs[] = */
/* { */
/*     /\** Left **\/ */
/*     // L */
/*     {.fk_frame=PIR_TF_LEFT_SDH_L_K0M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_L_K0P, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_L_K1M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_L_K1P, .marker_id=-1}, */

/*     // T */
/*     {.fk_frame=PIR_TF_LEFT_SDH_T_K0M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_T_K0P, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_T_K1M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_T_K1P, .marker_id=-1}, */

/*     // R */
/*     {.fk_frame=PIR_TF_LEFT_SDH_T_R0M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_T_R0P, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_T_R1M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_LEFT_SDH_T_R1P, .marker_id=-1}, */

/*     /\** Right **\/ */
/*     // L */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_L_K0M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_L_K0P, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_L_K1M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_L_K1P, .marker_id=-1}, */

/*     // T */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_T_K0M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_T_K0P, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_T_K1M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_T_K1P, .marker_id=-1}, */

/*     // R */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_T_R0M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_T_R0P, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_T_R1M, .marker_id=-1}, */
/*     {.fk_frame=PIR_TF_RIGHT_SDH_T_R1P, .marker_id=-1}, */
/* }; */
