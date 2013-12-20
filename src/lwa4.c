
#include <amino.h>
#include <ach.h>
#include <piranha.h>

/* void lwa4_kin2_( const double *q, const double *T0, const double *Tee, double *T, double *J ) { */
/*     double T_rel[3*4*7]; */
/*     double T_abs[3*4*7]; */

/*     lwa4_tf_( q, T_rel ); */
/*     rfx_kin_revchain( 7, T0, T_rel, Tee, axis[0], T_abs, J, 6 ); */
/*     AA_MEM_CPY( T, T_abs+12*6, 12 ); */
/* } */



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
