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

#ifndef PIRANHA_H
#define PIRANHA_H

#include <amino.h>
#include <reflex.h>

#include "pir-frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PIR_MAX_MSG_AXES 7

#define PIR_FT_WEIGHT 4

#define PIR_SDH_AXIAL 0
// F1
#define PIR_SDH_L0    1
#define PIR_SDH_L1    2
// F2
#define PIR_SDH_T0    3
#define PIR_SDH_T1    4
// F3
#define PIR_SDH_R0    5
#define PIR_SDH_R1    6

#define PIR_SDH_CENTER  0
#define PIR_SDH_L_AXIAL 7
#define PIR_SDH_R_AXIAL 8

#define PIR_SDH_L2    9
#define PIR_SDH_T2    10
#define PIR_SDH_R2    11
#define PIR_SDH_SIZE  12


enum pir_sdh_id {
    PIR_SDH_ID_CENTER,
    PIR_SDH_ID_T0,
    PIR_SDH_ID_T1,
    PIR_SDH_ID_TK0M,
    PIR_SDH_ID_TK0P,
    PIR_SDH_ID_TK1M,
    PIR_SDH_ID_TK1P,
    PIR_SDH_ID_T2,
    PIR_SDH_ID_L_AXIAL,
    PIR_SDH_ID_L0,
    PIR_SDH_ID_L1,
    PIR_SDH_ID_LK0M,
    PIR_SDH_ID_LK0P,
    PIR_SDH_ID_LK1M,
    PIR_SDH_ID_LK1P,
    PIR_SDH_ID_L2,
    PIR_SDH_ID_R_AXIAL,
    PIR_SDH_ID_R0,
    PIR_SDH_ID_R1,
    PIR_SDH_ID_RK0M,
    PIR_SDH_ID_RK0P,
    PIR_SDH_ID_RK1M,
    PIR_SDH_ID_RK1P,
    PIR_SDH_ID_R2,
    PIR_SDH_ID_SIZE
};


typedef enum pir_side  {
    PIR_LEFT,
    PIR_RIGHT
} pir_side_t;

enum pir_axis {

    PIR_AXIS_T  =       0,

    PIR_AXIS_L0 =       1,
    PIR_AXIS_L1 =       2,
    PIR_AXIS_L2 =       3,
    PIR_AXIS_L3 =       4,
    PIR_AXIS_L4 =       5,
    PIR_AXIS_L5 =       6,
    PIR_AXIS_L6 =       7,

    PIR_AXIS_R0 =       8,
    PIR_AXIS_R1 =       9,
    PIR_AXIS_R2 =      10,
    PIR_AXIS_R3 =      11,
    PIR_AXIS_R4 =      12,
    PIR_AXIS_R5 =      13,
    PIR_AXIS_R6 =      14,

    PIR_AXIS_SDH_L0 =  15,
    PIR_AXIS_SDH_L1 =  16,
    PIR_AXIS_SDH_L2 =  17,
    PIR_AXIS_SDH_L3 =  18,
    PIR_AXIS_SDH_L4 =  19,
    PIR_AXIS_SDH_L5 =  20,
    PIR_AXIS_SDH_L6 =  21,

    PIR_AXIS_SDH_R0 =  22,
    PIR_AXIS_SDH_R1 =  23,
    PIR_AXIS_SDH_R2 =  24,
    PIR_AXIS_SDH_R3 =  25,
    PIR_AXIS_SDH_R4 =  26,
    PIR_AXIS_SDH_R5 =  27,
    PIR_AXIS_SDH_R6 =  28,

    PIR_AXIS_CNT =     29
};


#define PIR_SIDE_INDICES( side, lwa, sdh )      \
    {                                           \
        if( PIR_LEFT == (side) ) {              \
            (lwa) = PIR_AXIS_L0;                \
            (sdh) = PIR_AXIS_SDH_L0;            \
        } else if( PIR_RIGHT == (side) ) {      \
            (lwa) = PIR_AXIS_R0;                \
            (sdh) = PIR_AXIS_SDH_R0;            \
        } else { assert(0); }                   \
    }

// TODO: rationalize pir_config and pir_state
struct pir_config {
    double q[PIR_TF_CONFIG_MAX];
    double dq[PIR_TF_CONFIG_MAX];
};

struct pir_state {
    double q[PIR_AXIS_CNT];
    double dq[PIR_AXIS_CNT];

    double F[2][6];
    double S_wp[2][8];
    double J_wp[2][7*6];
    double S_eer[2][8];
};

void lwa4_kin_( const double *q, const double *T0, const double *Tee, double *T, double *J );
void lwa4_tf_( const double *q, double *TT );
void lwa4_tf_abs_( const double *q, const double *T0, double *TT );


void lwa4_kin2_( const double *q, const double *T0, const double *Tee, double *T, double *J );

void lwa4_kin_duqu( const double *q, const double S0[8], const double Tee[8], double T[8], double *J );
void lwa4_duqu( const double *q, double *S_rel );


int pir_kin_solve( double q0[7], double S1[8], double q1[7] );

int pir_kin_arm( struct pir_state *X );
int pir_kin_ft( double *tf_abs, struct pir_state *X, double F_raw[2][6], double r_ft[2][4] );

void pir_kin( const double *q, double **tf_rel, double **tf_abs );


struct pir_msg {
    char mode[64];
    uint64_t salt;
    uint64_t seq_no;
    uint64_t n;
    union {
        int64_t i;
        double f;
    } x [1];
};

struct pir_msg_complete {
    uint64_t salt;
    uint64_t seq_no;
};

struct pir_mode_desc;
struct pir_mode;

#define JS_AXES 8
typedef struct {
    ach_channel_t chan_js;
    ach_channel_t chan_ref_torso;
    ach_channel_t chan_ref_left;
    ach_channel_t chan_ref_right;
    ach_channel_t chan_state_pir;
    ach_channel_t chan_ctrl;
    ach_channel_t chan_complete;
    ach_channel_t chan_config;
    ach_channel_t chan_reg;

    ach_channel_t chan_reg_cam;
    ach_channel_t chan_reg_ee;

    ach_channel_t chan_sdhref_left;
    ach_channel_t chan_sdhref_right;

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

    struct pir_config config;
    double *tf_rel;
    double *tf_abs;
    double bEc[7];

    double *bEc2;
    size_t n_bEc2;

    double lElp[7];
    double rErp[7];

    struct pir_mode_desc *mode;
    void *mode_cx;
    struct timespec t0;
    aa_mem_region_t modereg;
    struct rfx_trajx_seg_list *trajx_segs;

    rfx_trajq_t *trajq;
    struct rfx_trajq_seg_list *trajq_segs;

    struct timespec now;
    rfx_ctrl_t G[2];
    rfx_ctrl_t G_LR;
    rfx_ctrl_t G_T;
    rfx_ctrl_ws_lin_k_t Kx;
    rfx_ctrlq_lin_k_t Kq;
    rfx_ctrlq_lin_k_t Kq_lr;
    rfx_ctrlq_lin_k_t Kq_T;
    double q_min[PIR_AXIS_CNT];
    double q_max[PIR_AXIS_CNT];

    double sint;

} pirctrl_cx_t;

static inline double *pir_tfa( pirctrl_cx_t *cx, size_t i ) {
    return AA_MATCOL(cx->tf_abs, 7, i );
}

/*------ MODES --------*/
typedef void (*pir_mode_run_fun_t)( pirctrl_cx_t *);
typedef int (*pir_mode_init_fun_t)(pirctrl_cx_t *, struct pir_msg *);
typedef int (*pir_mode_terminate_fun_t)(pirctrl_cx_t *);

struct pir_mode_desc {
    const char *name;
    pir_mode_init_fun_t init;
    pir_mode_run_fun_t run;
    pir_mode_terminate_fun_t term;
};

/* struct pir_mode { */
/*     struct pir_mode_desc *desc; */
/*     // more data */
/* } */

void pir_zero_refs(pirctrl_cx_t *cx);

int set_mode_k_s2min(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_k_pt(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_k_pr(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_k_q(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_k_f(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_cpy(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_ws_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_ws_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_ws_left_finger(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_ws_right_finger(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_sin(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajx_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajx_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajx_w_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajx_w_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajq_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajq_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajq_lr(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajq_torso(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );


// all the different control modes
typedef void (*ctrl_fun_t)(void);
void ctrl_joint_left_shoulder( pirctrl_cx_t *cx );
void ctrl_joint_left_wrist( pirctrl_cx_t *cx );
void ctrl_joint_right_shoulder( pirctrl_cx_t *cx );
void ctrl_joint_right_wrist( pirctrl_cx_t *cx );
void ctrl_ws_left( pirctrl_cx_t *cx );
void ctrl_ws_right( pirctrl_cx_t *cx );
void ctrl_ws_left_finger( pirctrl_cx_t *cx );
void ctrl_ws_right_finger( pirctrl_cx_t *cx );
void ctrl_zero( pirctrl_cx_t *cx );
void ctrl_sin( pirctrl_cx_t *cx );
void ctrl_step( pirctrl_cx_t *cx );
void ctrl_trajx_left( pirctrl_cx_t *cx );
void ctrl_trajx_right( pirctrl_cx_t *cx );
void ctrl_trajx_w_left( pirctrl_cx_t *cx );
void ctrl_trajx_w_right( pirctrl_cx_t *cx );
void ctrl_trajq_left( pirctrl_cx_t *cx );
void ctrl_trajq_right( pirctrl_cx_t *cx );
void ctrl_trajq_lr( pirctrl_cx_t *cx );
void ctrl_trajq_torso( pirctrl_cx_t *cx );

struct servo_cam_cx {
    double cEo[7];
    double bEe[7];
};
int set_mode_servo_cam(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
void ctrl_servo_cam( pirctrl_cx_t *cx );


struct biservo_rel_cx {
    double rElt[7];
    double b_q_lt[4];
};
int set_mode_biservo_rel(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
void ctrl_biservo_rel( pirctrl_cx_t *cx );


int sdh_pinch_left( pirctrl_cx_t *cx, struct pir_msg * );
int sdh_pinch_right( pirctrl_cx_t *cx, struct pir_msg * );
int sdh_set_left( pirctrl_cx_t *cx, struct pir_msg * );
int sdh_set_right( pirctrl_cx_t *cx, struct pir_msg * );


#ifdef __cplusplus
}
#endif

#endif //PIRANHA_H
