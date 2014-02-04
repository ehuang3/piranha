#include <amino.h>
#include <reflex.h>
#include <sns.h>
#include "piranha.h"

double q0[7] = {-1.0010510457738677e0,
                -0.4106410664092259e0,
                1.0136872295583066e0,
                -1.6339772457170914e0,
                1.2310156280166404e0,
                1.5006515441572446e0,
                -1.8219317528643606e0};

double e0[] = {0.18301270189221935,-0.1830127018922193,-0.6830127018922192,0.6830127018922194,0.402999997138977,0.16499999910593033,0.0010000000474974548};

double e1[] = {0.18301270189221935,-0.1830127018922193,-0.6830127018922192,0.6830127018922194,0.40299999713897705,0.06499999761581421,0.0010000000474974513};

double e2[] = {0.7010573846499779,-0.7010573846499778,-0.09229595564125725,0.09229595564125731,0.40299999713897705,0.061999997615814206,0.0010000000474974513};

double e3[] = {0.35355339059327384,-0.3535533905932738,0.6123724356957945,-0.6123724356957945,0.40299999713897705,0.058999997615814204,0.0010000000474974515};


static const double lwa4_axis[][3] = {
    {-1,0,0},   /* 01 */
    {0,-1,0},  /* 12 */
    {-1,0,0},  /* 23 */
    {0,-1,0},  /* 34 */
    {-1,0,0},  /* 45 */
    {0,1,0}, /* 56 */
    {1,0,0}  /* 67 */
};

static const double lwa4_trans[][3] = {
    {0,0,0},   /* 01 */
    {0,0,0},  /* 12 */
    {0,0,0},  /* 23 */
    {LWA4_L_1,0,0},  /* 34 */
    {LWA4_L_2,0,0},  /* 45 */
    {0,0,0}, /* 56 */
    {0,0,0}  /* 67 */
};


int kin_fun ( const void *cx, const double *q, double E[7], double *J)
{
    (void)cx;
    double *Q = AA_MEM_REGION_LOCAL_NEW_N(double, PIR_TF_CONFIG_MAX);
    double *tf_rel = AA_MEM_REGION_LOCAL_NEW_N(double, 7 * PIR_TF_FRAME_MAX);
    double *tf_abs = AA_MEM_REGION_LOCAL_NEW_N(double, 7 * PIR_TF_FRAME_MAX);

    AA_MEM_ZERO(Q,PIR_TF_CONFIG_MAX);
    AA_MEM_CPY( Q+PIR_TF_LEFT_Q_SHOULDER0, q, 7 );

    pir_tf_rel( Q, tf_rel );
    pir_tf_abs( tf_rel, tf_abs );

    // Hack in fingertips
    {
        double *v_ll = &tf_abs[7*PIR_TF_LEFT_SDH_L_2 + 4];
        double *v_lr = &tf_abs[7*PIR_TF_LEFT_SDH_R_2 + 4];

        double *v_rl = &tf_abs[7*PIR_TF_RIGHT_SDH_L_2 + 4];
        double *v_rr = &tf_abs[7*PIR_TF_RIGHT_SDH_R_2 + 4];


        for( size_t i = 0; i < 3; i ++ ) {
            tf_abs[7*PIR_TF_LEFT_SDH_FINGERTIP  + 4 + i ] = (v_ll[i] + v_lr[i]) / 2;
            tf_abs[7*PIR_TF_RIGHT_SDH_FINGERTIP + 4 + i ] = (v_rl[i] + v_rr[i]) / 2;
        }
    }
    AA_MEM_CPY(E, AA_MATCOL(tf_abs,7,PIR_TF_LEFT_SDH_FINGERTIP), 7);

    if( J ) {
        double *pe = E+4;
        size_t indices[7];
        for( size_t i = 0; i < 7; i ++ ) indices[i] = i;

        AA_MEM_ZERO(J,6*7);
        rfx_tf_rev_jacobian( AA_MATCOL(tf_abs,7, PIR_TF_LEFT_SHOULDER0),
                             pir_tf_axes[PIR_TF_LEFT_Q_SHOULDER0],
                             7, indices, pe, J, 6 );
    }

    aa_mem_region_local_pop(Q);
    return 0;
}

int main(void) {

    aa_mem_region_t reg;
    aa_mem_region_init( &reg, 1024*32 );


    // make trajectory
    double t = 0;
    struct rfx_trajx_point_list *plist = rfx_trajx_point_list_alloc( &reg );
    double E0[7];
    kin_fun(NULL, q0, E0, NULL );
    rfx_trajx_point_list_addb_qv( plist, t, 1, E0, E0+4 );
    t += 12;
    rfx_trajx_point_list_addb_qv( plist, t, 1, e0, e0+4 );
    t += 12;
    rfx_trajx_point_list_addb_qv( plist, t, 1, e1, e1+4 );
    t += 10;
    rfx_trajx_point_list_addb_qv( plist, t, 1, e2, e2+4 );
    t += 10;
    rfx_trajx_point_list_addb_qv( plist, t, 1, e3, e3+4 );

    struct rfx_trajx_seg_list *seglist =
        rfx_trajx_splend_generate( plist, &reg );


    // setup WS controller
    rfx_ctrlx_lin_t * ctrlx = rfx_ctrlx_lin_alloc( aa_mem_region_local_get(), 7, kin_fun, NULL );
    struct rfx_trajx_plot_opts xopts = {0};
    xopts.ctrlx_fun = &rfx_ctrlx_fun_lin_vfwd;
    xopts.ctrlx_cx = ctrlx;
    xopts.kin_fun = &kin_fun;
    xopts.n_q = 7;
    xopts.q_0 = q0;
    xopts.to_file = 1;


    rfx_ctrl_t *G = ctrlx->ctrl;
    for( size_t i = 0; i < 3; i ++ ) {
        G->x_min[i] = -10;
        G->x_max[i] = 10;
    }
    for( size_t i = 0; i < 7; i ++ ) {
        G->q_min[i] = -2*M_PI;
        G->q_max[i] = M_PI;
    }

    rfx_ctrl_ws_lin_k_t *K = ctrlx->k;
    AA_MEM_SET( K->q, 0.1, 7 );
    K->q[1] *= 5; // lower limits
    K->q[3] *= 5; // lower limits
    K->q[5] *= 5; // lower limits
    K->q[6] *= 5; // this module is most sensitive to limits
    //AA_MEM_SET( cx.Kx.f, .003, 3 );
    //AA_MEM_SET( cx.Kx.f+3, .000, 3 );
    //AA_MEM_SET( cx.Kx.f, -.000, 6 );
    AA_MEM_SET( K->p, 1.0, 3 );
    AA_MEM_SET( K->p+3, 1.0, 3 );
    /* AA_MEM_SET( cx.K.p, 0.0, 3 ); */
    /* AA_MEM_SET( cx.K.p+3, 0.0, 3 ); */
    K->dls = .005;
    K->s2min = .01;


    // plot
    rfx_trajx_seg_list_plot( seglist, .025, &xopts );
}
