#include <sns.h>
#include <gamepad.h>
#include "piranha.h"

// 20 deg/s
#define MAXVEL_FACTOR 20 * M_PI/180

static void pir_complete( pirctrl_cx_t *cx ) {
    struct pir_msg_complete msg = { .salt = cx->msg_ctrl.salt,
                                    .seq_no = cx->msg_ctrl.seq_no };
    msg.seq_no = cx->msg_ctrl.seq_no;
    ach_status_t r = ach_put( &cx->chan_complete, &msg, sizeof(msg) );
    if( ACH_OK != r ) {
        SNS_LOG( LOG_ERR, "couldn't put pir_msg_complete: `%s'\n",
                 ach_result_to_string(r) );
    }
}

void ctrl_joint_torso( pirctrl_cx_t *cx ) {
    double u = cx->ref.user[GAMEPAD_AXIS_RT] - cx->ref.user[GAMEPAD_AXIS_LT];
    cx->ref.dq[PIR_AXIS_T] = u * MAXVEL_FACTOR;
}
void ctrl_joint_left_shoulder( pirctrl_cx_t *cx ) {
    ctrl_joint_torso(cx);
    cx->ref.dq[PIR_AXIS_L0] = cx->ref.user[GAMEPAD_AXIS_LY] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_L1] = cx->ref.user[GAMEPAD_AXIS_LX] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_L2] = cx->ref.user[GAMEPAD_AXIS_RY] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_L3] = cx->ref.user[GAMEPAD_AXIS_RX] * MAXVEL_FACTOR;
}
void ctrl_joint_left_wrist( pirctrl_cx_t *cx ) {
    ctrl_joint_torso(cx);
    cx->ref.dq[PIR_AXIS_L3] = cx->ref.user[GAMEPAD_AXIS_LX] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_L4] = cx->ref.user[GAMEPAD_AXIS_LY] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_L5] = cx->ref.user[GAMEPAD_AXIS_RX] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_L6] = cx->ref.user[GAMEPAD_AXIS_RY] * MAXVEL_FACTOR;
}
void ctrl_joint_right_shoulder( pirctrl_cx_t *cx ) {
    ctrl_joint_torso(cx);
    cx->ref.dq[PIR_AXIS_R0] = cx->ref.user[GAMEPAD_AXIS_LY] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_R1] = cx->ref.user[GAMEPAD_AXIS_LX] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_R2] = cx->ref.user[GAMEPAD_AXIS_RY] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_R3] = cx->ref.user[GAMEPAD_AXIS_RX] * MAXVEL_FACTOR;
}
void ctrl_joint_right_wrist( pirctrl_cx_t *cx ) {
    ctrl_joint_torso(cx);
    cx->ref.dq[PIR_AXIS_R3] = cx->ref.user[GAMEPAD_AXIS_LX] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_R4] = cx->ref.user[GAMEPAD_AXIS_LY] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_R5] = cx->ref.user[GAMEPAD_AXIS_RX] * MAXVEL_FACTOR;
    cx->ref.dq[PIR_AXIS_R6] = cx->ref.user[GAMEPAD_AXIS_RY] * MAXVEL_FACTOR;
}


void ctrl_zero( pirctrl_cx_t *cx ) {
    for( size_t i = PIR_AXIS_L0; i <= PIR_AXIS_L6; i ++ ) {
        double k = -cx->ref.user[GAMEPAD_AXIS_LT] + cx->ref.user[GAMEPAD_AXIS_RT];
        cx->ref.dq[i] = - .25* k * cx->state.q[i];
    }
    for( size_t i = PIR_AXIS_R0; i <= PIR_AXIS_R6; i ++ ) {
        double k = -cx->ref.user[GAMEPAD_AXIS_LT] + cx->ref.user[GAMEPAD_AXIS_RT];
        cx->ref.dq[i] = - .25* k * cx->state.q[i];
    }
}


void ctrl_ws( pirctrl_cx_t *cx, size_t i, double S[8], double S_rel[8], int side ) {
    rfx_ctrl_ws_t *G = &cx->G[side];
    // set refs
    AA_MEM_SET( G->ref.dx, 0, 6 );
    double dx[6];
    if( cx->ref.user_button & GAMEPAD_BUTTON_RB ) {
        dx[3] = cx->ref.user[GAMEPAD_AXIS_LX] * .3;
        dx[4] = cx->ref.user[GAMEPAD_AXIS_LY] * .3;
        dx[5] = cx->ref.user[GAMEPAD_AXIS_RX] * .3;

        dx[0] = cx->ref.user[GAMEPAD_AXIS_DX] * .02;
        dx[1] = cx->ref.user[GAMEPAD_AXIS_DY] * .02;
        dx[2] = cx->ref.user[GAMEPAD_AXIS_RY] * .1;
    } else {
        dx[0] = cx->ref.user[GAMEPAD_AXIS_LX] * .1;
        dx[1] = cx->ref.user[GAMEPAD_AXIS_LY] * .1;
        dx[2] = cx->ref.user[GAMEPAD_AXIS_RX] * .1;
    }
    if( S_rel ) {
        double S_tmp[8];
        rfx_kin_duqu_relvel( S, S_rel, dx, S_tmp, G->ref.dx );
    } else {
        memcpy( G->ref.dx, dx, sizeof(dx) );

    }

    // compute stuff
    int r = rfx_ctrl_ws_lin_vfwd( G, &cx->Kx, &cx->ref.dq[i] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }
    // integrate
    rfx_ctrl_ws_sdx( G, cx->dt );
}

void ctrl_ws_left( pirctrl_cx_t *cx ) {
    ctrl_ws( cx, PIR_AXIS_L0, NULL, NULL, PIR_LEFT );
}

void ctrl_ws_right( pirctrl_cx_t *cx ) {
    ctrl_ws( cx, PIR_AXIS_R0,  NULL, NULL, PIR_RIGHT );
}

void ctrl_ws_left_finger( pirctrl_cx_t *cx ) {
    ctrl_ws( cx, PIR_AXIS_L0, cx->state.S_wp[PIR_LEFT], cx->state.S_eer[PIR_LEFT], PIR_LEFT );
}


void ctrl_ws_right_finger( pirctrl_cx_t *cx ) {
    ctrl_ws( cx, PIR_AXIS_R0, cx->state.S_wp[PIR_RIGHT], cx->state.S_eer[PIR_RIGHT], PIR_RIGHT );
}

void ctrl_sin( pirctrl_cx_t *cx ) {

    double k = (cx->ref.user[GAMEPAD_AXIS_LT] + cx->ref.user[GAMEPAD_AXIS_RT]) ;

    double v = k*sin(cx->sint);

    //printf("%f\n", v*180/M_PI);

    cx->ref.dq[PIR_AXIS_L6] = v;
    cx->ref.dq[PIR_AXIS_L5] = v;
    cx->ref.dq[PIR_AXIS_L4] = v;
    cx->ref.dq[PIR_AXIS_L3] = v;
    cx->ref.dq[PIR_AXIS_L2] = v;
    cx->ref.dq[PIR_AXIS_L1] = v;
    cx->ref.dq[PIR_AXIS_L0] = v;

    cx->sint += .001 * 2*M_PI;

}

void ctrl_step( pirctrl_cx_t *cx ) {

    if( cx->ref.user[ GAMEPAD_AXIS_LT ] > .5 ) {
        aa_fset(& cx->ref.dq[PIR_AXIS_L0], 15 * M_PI/180, 7 );
    } else if( cx->ref.user[ GAMEPAD_AXIS_RT ] > .5 ) {
        aa_fset(& cx->ref.dq[PIR_AXIS_L0], -15 * M_PI/180, 7 );
    }


}

void ctrl_trajx( pirctrl_cx_t *cx ) {
    int side = PIR_LEFT;
    int lwa, sdh;
    PIR_SIDE_INDICES( side, lwa, sdh );
    (void)sdh;


    double t = aa_tm_timespec2sec( aa_tm_sub( cx->now, cx->t0 ) );


    // get refs
    double S_traj[8], dx[6] = {0};

    if( t >= cx->trajx->pt_f->t ) {
        //aa_tf_qv2duqu( cx->trajx->pt_f->r, cx->trajx->pt_f->x, cx->G_L.ref.S );
        aa_tf_qv2duqu( cx->trajx->pt_f->r, cx->trajx->pt_f->x, S_traj );
        AA_MEM_SET( cx->G[side].ref.dx, 0, 6 );
        pir_complete(cx);
    } else {
        rfx_trajx_get_x_duqu( cx->trajx, t, S_traj );
        //rfx_trajx_get_dx( cx->trajx, t, cx->G_L.ref.dx );
        rfx_trajx_get_dx( cx->trajx, t, dx );
    }

    // convert to wrist frame
    aa_tf_duqu_mulc( S_traj, cx->state.S_eer[side], cx->G[side].ref.S  );
    double S_tmp[8];
    rfx_kin_duqu_relvel( cx->G[side].ref.S, cx->state.S_eer[side], dx, S_tmp, cx->G[side].ref.dx );


    int r = rfx_ctrl_ws_lin_vfwd( &cx->G[side], &cx->Kx, &cx->ref.dq[lwa] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }
    //printf("trajx: "); aa_dump_vec(stdout, dx, 6 );
    //printf("dq: "); aa_dump_vec(stdout, &cx->ref.dq[PIR_AXIS_L0], 8 );

}

static void ctrl_trajq_doit( pirctrl_cx_t *cx, rfx_ctrl_t *G, rfx_ctrlq_lin_k_t *K, size_t off ) {
    double t = aa_tm_timespec2sec( aa_tm_sub( cx->now, cx->t0 ) );

    // don't go past the end
    double t_f = rfx_trajq_seg_list_get_t_f(cx->trajq_segs);
    if( t >= t_f ) {
        t = t_f;
        pir_complete(cx);
    }

    // get refs
    rfx_trajq_seg_list_get_dq( cx->trajq_segs, t, G->ref.q,G->ref.dq );

    int r = rfx_ctrlq_lin_vfwd( G, K, &cx->ref.dq[off] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }

}
void ctrl_trajq_left( pirctrl_cx_t *cx ) {
    ctrl_trajq_doit( cx, &cx->G[PIR_LEFT], &cx->Kq, PIR_AXIS_L0 );
}

void ctrl_trajq_right( pirctrl_cx_t *cx ) {
    ctrl_trajq_doit( cx, &cx->G[PIR_RIGHT], &cx->Kq, PIR_AXIS_R0 );
}

void ctrl_trajq_lr( pirctrl_cx_t *cx ) {
    _Static_assert( PIR_AXIS_L0 + 7 == PIR_AXIS_R0, "Invalid axis ordering" );
    ctrl_trajq_doit( cx, &cx->G_LR, &cx->Kq_lr, PIR_AXIS_L0 );
}

void ctrl_trajq_torso( pirctrl_cx_t *cx ) {
    double t = aa_tm_timespec2sec( aa_tm_sub( cx->now, cx->t0 ) );

    if( t >= cx->trajq->t_f ) {
        AA_MEM_CPY( cx->G_T.ref.q, cx->trajq->q_f, 1 );
        AA_MEM_SET( cx->G_T.ref.dq, 0, 1 );
        pir_complete(cx);
    } else {
        // get refs
        rfx_trajq_get_q( cx->trajq, t, cx->G_T.ref.q );
        rfx_trajq_get_dq( cx->trajq, t, cx->G_T.ref.dq );
    }

    int r = rfx_ctrlq_lin_vfwd( &cx->G_T, &cx->Kq_T, &cx->ref.dq[PIR_AXIS_T] );
    (void)r;
    //printf("trajq: "); aa_dump_vec(stdout, &cx->ref.dq[PIR_AXIS_T], 1);

}
