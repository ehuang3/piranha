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

#define PIR_MAX_MSG_AXES 7

#define PIR_FT_WEIGHT 4

#define PIR_SDH_AXIAL 0
#define PIR_SDH_L0    1
#define PIR_SDH_L1    2
#define PIR_SDH_T0    3
#define PIR_SDH_T1    4
#define PIR_SDH_R0    5
#define PIR_SDH_R1    6


// TODO: subtract inseration depth of tool changer
#define LWA4_FT_L 82.2e-3 ///< Length of F/T Sensor

#define LWA4_FT_ANGLE (45*M_PI/180)


#define LWA4_L_0 (0.3000) /* pedestal to shoulder */
#define LWA4_L_1 (0.6280 - LWA4_L_0) /* shoulder to elbow */
#define LWA4_L_2 (0.9510 - LWA4_L_1 - LWA4_L_0) /* elbow to wrist */
#define LWA4_L_e (1.0334 - LWA4_L_2 - LWA4_L_1 - LWA4_L_0) /* wrist to E.E end of powerball */


// TODO: Adjust for new tool changer

#define SDH_LB 98e-3      ///< Base connector to finger joint
#define SDH_L1 0.0865     ///< Lower finger joint
#define SDH_L2 0.0675     ///< Upper finger joint
#define SDH_B 66e-3       ///< distance between fingers
#define SDH_TC 38.105e-3  ///< Thumb to center point
#define SDH_FC 19.053e-3  ///< Fingers line to center point

#define SDH_MASS 1.955                    ///< SDH Mass (kilograms)
#define SDH_WEIGHT (SDH_MASS*AA_K_STD_G)  ///< SDH Weight (Newton)

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

// TODO: all transforms, jacobians, forces, in global frame

struct pir_state {
    double q[PIR_AXIS_CNT];
    double dq[PIR_AXIS_CNT];

    double F_L[6];
    double F_R[6];

    double S_wp_L[8];
    double S_wp_R[8];

    double J_wp_L[7*6];
    double J_wp_R[7*6];

    double S_eer_L[8];
    double S_eer_R[8];
};

void lwa4_kin_( const double *q, const double *T0, const double *Tee, double *T, double *J );
void lwa4_tf_( const double *q, double *TT );
void lwa4_tf_abs_( const double *q, const double *T0, double *TT );


void lwa4_kin2_( const double *q, const double *T0, const double *Tee, double *T, double *J );

void lwa4_kin_duqu( const double *q, const double S0[8], const double Tee[8], double T[8], double *J );
void lwa4_duqu( const double *q, double *S_rel );


int pir_kin_solve( double q0[7], double S1[8], double q1[7] );


struct pir_msg {
    char mode[64];
    uint64_t n;
    union {
        int64_t i;
        double f;
    } x [1];
};

struct pir_msg_complete {
    uint64_t n;
};

struct pir_mode_desc;

#define JS_AXES 8
typedef struct {
    ach_channel_t chan_js;
    ach_channel_t chan_ref_torso;
    ach_channel_t chan_ref_left;
    ach_channel_t chan_ref_right;
    ach_channel_t chan_state_pir;
    ach_channel_t chan_ctrl;
    ach_channel_t chan_complete;

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

    struct pir_mode_desc *mode;
    struct timespec t0;
    aa_mem_region_t modereg;
    rfx_trajx_t *trajx;
    rfx_trajq_t *trajq;
    rfx_trajq_seg_list_t *trajq_segs;

    struct timespec now;
    rfx_ctrl_t G_L;
    rfx_ctrl_t G_R;
    rfx_ctrl_t G_T;
    rfx_ctrl_ws_lin_k_t Kx;
    rfx_ctrlq_lin_k_t Kq;
    rfx_ctrlq_lin_k_t Kq_T;
    double q_min[PIR_AXIS_CNT];
    double q_max[PIR_AXIS_CNT];

    double sint;
} pirctrl_cx_t;

/*------ MODES --------*/
typedef void (*pir_ctrl_fun_t)( pirctrl_cx_t *);
typedef int (*pir_mode_fun_t)(pirctrl_cx_t *, struct pir_msg *);

struct pir_mode_desc {
    const char *name;
    pir_mode_fun_t setmode;
    pir_ctrl_fun_t ctrl;
};

int set_mode_k_s2min(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_k_pt(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_k_pr(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_k_q(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_k_f(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_cpy(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_ws_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_ws_left_finger(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_ws_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_sin(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajx(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajq_left(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajq_right(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );
int set_mode_trajq_torso(pirctrl_cx_t *cx, struct pir_msg *msg_ctrl );


// all the different control modes
typedef void (*ctrl_fun_t)(void);
void ctrl_joint_left_shoulder( pirctrl_cx_t *cx );
void ctrl_joint_left_wrist( pirctrl_cx_t *cx );
void ctrl_joint_right_shoulder( pirctrl_cx_t *cx );
void ctrl_joint_right_wrist( pirctrl_cx_t *cx );
void ctrl_ws_left( pirctrl_cx_t *cx );
void ctrl_ws_left_finger( pirctrl_cx_t *cx );
void ctrl_ws_right( pirctrl_cx_t *cx );
void ctrl_zero( pirctrl_cx_t *cx );
void ctrl_sin( pirctrl_cx_t *cx );
void ctrl_step( pirctrl_cx_t *cx );
void ctrl_trajx( pirctrl_cx_t *cx );
void ctrl_trajq_left( pirctrl_cx_t *cx );
void ctrl_trajq_right( pirctrl_cx_t *cx );
void ctrl_trajq_torso( pirctrl_cx_t *cx );


int sdh_zero( pirctrl_cx_t *cx, struct pir_msg * );
int sdh_set( pirctrl_cx_t *cx, struct pir_msg * );
int sdh_pinch( pirctrl_cx_t *cx, struct pir_msg * );

#endif //PIRANHA_H
