
#include <amino.h>
#include <ach.h>
#include <piranha.h>

static const double axis[][3] = {
    {-1,0,0},   /* 01 */
    {0,-1,0,},  /* 12 */
    {-1,0,0,},  /* 23 */
    {0,-1,0,},  /* 34 */
    {-1,0,0,},  /* 45 */
    {0,1,0,}, /* 56 */
    {1,0,0,}  /* 67 */
};

void lwa4_kin2_( const double *q, const double *T0, const double *Tee, double *T, double *J ) {
    double T_rel[3*4*7];
    double T_abs[3*4*7];

    lwa4_tf_( q, T_rel );
    rfx_kin_revchain( 7, T0, T_rel, Tee, axis[0], T_abs, J, 6 );
    AA_MEM_CPY( T, T_abs+12*6, 12 );
}

#define L_0 (0.300) /* pedestal to shoulder */
#define L_1 (0.628 - L_0) /* shoulder to elbow */
#define L_2 (0.951 - L_1 - L_0) /* elbow to wrist */
#define L_e (1.0334 - L_2 - L_1 - L_0) /* wrist to E.E end of powerball */


void lwa4_duqu( const double *q, double *S_rel ) {
    aa_tf_xxyz2duqu( -q[0],   0, 0, 0, S_rel + 0*8 );
    aa_tf_yxyz2duqu( -q[1],   0, 0, 0, S_rel + 1*8);
    aa_tf_xxyz2duqu( -q[2],   0, 0, 0, S_rel + 2*8);
    aa_tf_yxyz2duqu( -q[3], L_1, 0, 0, S_rel + 3*8);
    aa_tf_xxyz2duqu( -q[4], L_2, 0, 0, S_rel + 4*8);
    aa_tf_yxyz2duqu(  q[5],   0, 0, 0, S_rel + 5*8);
    aa_tf_xxyz2duqu(  q[6], L_e, 0, 0, S_rel + 6*8);
}


void lwa4_kin_duqu( const double *q, const double S0[8], const double See[8], double S[8], double *J ) {

    double S_rel[8*7];
    lwa4_duqu(q, S_rel );

    double S_abs[8*7];
    if( J ) {
        rfx_kin_duqu_revchain( 7, S0, S_rel, See, axis[0], S_abs, S, J, 6 );
    } else {

        rfx_kin_duqu_chain( 7, S0, S_rel, S_abs );
        aa_tf_duqu_mul( S_abs+8*6, See, S );
    }
}



static double qref[7] = {0};
static double dq_dt[7] = {1e0, 5e0, 1e0, 5e0, 1e0, 5e0, 5e0};

struct rfx_kin_solve_opts pir_kin_solve_opts = {
    .dt0 = 1,                      // initial timestep
    .theta_tol = .1 * M_PI/180,    // angle error tolerate
    .x_tol = 1e-3,                 // translation error tolerance
    .dq_tol = 1e-4,                // configuration error tolerance
    .s2min_dls = 1e-2,             // minimum square singular value for damped least squares
    .dx_dt = 1,                    // scaling for cartesian error
    .dq_dt = dq_dt,                // scaling for joint projection
    .q_ref = qref
};


// TODO: initial and final tf
int pir_kin_fun( const void *cx, const double q[7], double S[8], double J[6*7] ) {
    (void)cx;
    lwa4_kin_duqu( q, aa_tf_duqu_ident, aa_tf_duqu_ident, S, J );
    return 0;
}


int pir_kin_solve( double q0[7], double S1[8], double q1[7] ) {
    rfx_kin_solve( 7, q0, S1, &pir_kin_fun,
                   q1, &pir_kin_solve_opts );
    return 0;
}
