
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

void lwa4_kin_duqu( const double *q, const double S0[8], const double See[8], double S[8], double *J ) {
    double S_rel[8*7];
    {
        double T_rel[3*4*7];
        lwa4_tf_( q, T_rel );
        for( size_t i = 0; i < 7; i ++ ) {
            aa_tf_tfmat2duqu( T_rel + 12*i, S_rel + 8*i );
        }
    }

    double S_abs[8*7];
    rfx_kin_duqu_revchain( 7, S0, S_rel, See, axis[0], S_abs, J, 6 );
    AA_MEM_CPY( S, S_abs+8*6, 8 );
}
