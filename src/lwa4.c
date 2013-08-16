
#include <amino.h>
#include <ach.h>
#include <piranha.h>


void lwa4_kin2_( const double *q, const double *T0, const double *Tee, double *T, double *J ) {
    double T_rel[3*4*7];
    double T_abs[3*4*7];

    static const double axis[][3] = { {-1,0,0},   /* 01 */
                                      {0,-1,0,},  /* 12 */
                                      {-1,0,0,},  /* 23 */
                                      {0,-1,0,},  /* 34 */
                                      {-1,0,0,},  /* 45 */
                                      {0,1,0,}, /* 56 */
                                      {1,0,0,}  /* 67 */
    };

    lwa4_tf_( q, T_rel );
    rfx_kin_revchain( 7, T0, T_rel, Tee, axis[0], T_abs, J, 6 );
    AA_MEM_CPY( T, T_abs+12*6, 12 );
}
