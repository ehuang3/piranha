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


static void servo_cam( pirctrl_cx_t *cx, struct servo_cam_cx *mode_cx )
{
    //printf("--\n");
    double wEe[7];
    aa_tf_qutr_cmul( AA_MATCOL(cx->tf_abs, 7, PIR_TF_RIGHT_WRIST2),
                     AA_MATCOL(cx->tf_abs, 7, PIR_TF_RIGHT_SDH_FINGERTIP),
                     wEe );
    rfx_ctrl_ws_t *G = &cx->G[PIR_RIGHT];

    // finger ref in body frame
    double bEer[7];
    aa_tf_qutr_mul( cx->bEc, mode_cx->cEo, bEer );
    AA_MEM_CPY( bEer, mode_cx->bEe, 4 );
    //printf("bEe: "); aa_dump_vec(stdout,bEer,7);
    for( size_t i = 0; i < 3; i++ ) bEer[4+i] += mode_cx->bEe[4+i];

    // wrist ref
    double bEwr[7];
    aa_tf_qutr_mulc( bEer, wEe, bEwr );

    // fill controller
    aa_tf_qutr2duqu( bEwr, G->ref.S );
    AA_MEM_ZERO( G->ref.dx, 6 );

    int r = rfx_ctrl_ws_lin_vfwd( G, &cx->Kx, &cx->ref.dq[PIR_AXIS_R0] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }
    //printf("--\n");
}


void ctrl_biservo_rel( pirctrl_cx_t *cx )
{
    struct biservo_rel_cx *mode_cx = (struct biservo_rel_cx *)cx->mode_cx;
    // fixup end-effector pose
    double bElwp[7], bErwp[7];
    aa_tf_qutr_mul( AA_MATCOL(cx->tf_abs, 7, PIR_TF_LEFT_WRIST2 ),
                    cx->lElp, bElwp );
    aa_tf_qutr_mul( AA_MATCOL(cx->tf_abs, 7, PIR_TF_RIGHT_WRIST2 ),
                    cx->rErp, bErwp );

    // rel ef:  bEe = bEw * wEe => wEe = conj(bEw) * bEe
    double rwEre[7], lwEle[7];
    aa_tf_qutr_cmul( AA_MATCOL(cx->tf_abs, 7, PIR_TF_RIGHT_WRIST2),
                     AA_MATCOL(cx->tf_abs, 7, PIR_TF_RIGHT_SDH_FINGERTIP),
                     rwEre );
    aa_tf_qutr_cmul( AA_MATCOL(cx->tf_abs, 7, PIR_TF_LEFT_WRIST2),
                     AA_MATCOL(cx->tf_abs, 7, PIR_TF_LEFT_SDH_FINGERTIP),
                     lwEle );


    // compute target pose
    double bErep[7], bElep[7], bElwtp[7];

    aa_tf_qutr_mul( bErwp, rwEre, bErep );
    aa_tf_qutr_mul( bErep, mode_cx->rElt, bElep );

    if( aa_tf_qnorm(mode_cx->b_q_lt) > 0 ) {
        //printf("copy quat\n");
        AA_MEM_CPY( bElep, mode_cx->b_q_lt, 4 );
    }
    aa_tf_qutr_mulc( bElep, lwEle, bElwtp );

    //double Et[7];
    //aa_tf_qutr_mulc( bElwtp, cx->lElp, Et );

    // fill controller
    rfx_ctrl_ws_t *G = &cx->G[PIR_LEFT];
    aa_tf_qutr2duqu( bElwtp, G->ref.S );
    aa_tf_qutr2duqu( bElwp, G->act.S );
    AA_MEM_ZERO( G->ref.dx, 6 );

    //printf("act: "); aa_dump_vec( stdout, bElwp, 7 );
    //printf("ref: "); aa_dump_vec( stdout, bElwtp, 7 );

    int r = rfx_ctrl_ws_lin_vfwd( G, &cx->Kx, &cx->ref.dq[PIR_AXIS_L0] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }
    //printf("--\n");
}


void ctrl_servo_cam( pirctrl_cx_t *cx )
{
    servo_cam(cx, (struct servo_cam_cx*)cx->mode_cx);
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
        AA_MEM_SET(& cx->ref.dq[PIR_AXIS_L0], 15 * M_PI/180, 7 );
    } else if( cx->ref.user[ GAMEPAD_AXIS_RT ] > .5 ) {
        AA_MEM_SET(& cx->ref.dq[PIR_AXIS_L0], -15 * M_PI/180, 7 );
    }


}


void ctrl_trajx_side( pirctrl_cx_t *cx, int side, int eer ) {
    int lwa, sdh;
    PIR_SIDE_INDICES( side, lwa, sdh );
    (void)sdh;

    double t = aa_tm_timespec2sec( aa_tm_sub( cx->now, cx->t0 ) );
    double t_f = rfx_trajx_seg_list_get_t_f(cx->trajx_segs);
    if( t > t_f ) {
        t = t_f;
        pir_complete(cx);
    }

    // get refs
    double S_traj[8], dx[6] = {0};
    rfx_trajx_seg_list_get_dx_duqu( cx->trajx_segs, t, S_traj, dx );

    if( eer ) {
        // convert to wrist frame
        aa_tf_duqu_mulc( S_traj, cx->state.S_eer[side], cx->G[side].ref.S  );
        double S_tmp[8];
        rfx_kin_duqu_relvel( cx->G[side].ref.S, cx->state.S_eer[side], dx, S_tmp, cx->G[side].ref.dx );
    } else {
        memcpy( cx->G[side].ref.S, S_traj, sizeof(S_traj) );
        memcpy( cx->G[side].ref.dx, dx, sizeof(dx) );
    }

    int r = rfx_ctrl_ws_lin_vfwd( &cx->G[side], &cx->Kx, &cx->ref.dq[lwa] );
    if( RFX_OK != r ) {
        SNS_LOG( LOG_ERR, "ws error: %s\n",
                 rfx_status_string((rfx_status_t)r) );
    }
}

void ctrl_trajx_w_left( pirctrl_cx_t *cx ) {
    ctrl_trajx_side( cx, PIR_LEFT, 0 );
}
void ctrl_trajx_w_right( pirctrl_cx_t *cx ) {
    ctrl_trajx_side( cx, PIR_RIGHT, 0 );
}
void ctrl_trajx_left( pirctrl_cx_t *cx ) {
    ctrl_trajx_side( cx, PIR_LEFT, 1 );
}
void ctrl_trajx_right( pirctrl_cx_t *cx ) {
    ctrl_trajx_side( cx, PIR_RIGHT, 1 );
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
