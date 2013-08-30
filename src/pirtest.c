#include <assert.h>
#include <amino.h>
#include <ach.h>
#include <reflex.h>
#include "piranha.h"


#define N 1000

int main(void) {

    double q[7] = {0};

    q[0] = M_PI/2;
    q[4] = .25*M_PI;

    double T_rel[3*4*7] = {0};
    double T_abs[3*4*7] = {0};
    double J[6*7];
    double J2[6*7];
    double Te[3*4];

    double axis[][3] = { {-1,0,0},   /* 01 */
                         {0,-1,0,},  /* 12 */
                         {-1,0,0,},  /* 23 */
                         {0,-1,0,},  /* 34 */
                         {-1,0,0,},  /* 45 */
                         {0,1,0,}, /* 56 */
                         {1,0,0,}  /* 67 */
    };

    aa_tick("null: ");
    aa_tock();

    /* aa_tick("symbolic: "); */
    /* for( size_t i = 0; i < N; i ++ ) */
    /*     lwa4_kin_( q, aa_tf_ident, aa_tf_ident, Te, J ); */
    /* aa_tock(); */
    /* aa_dump_mat(stdout, Te, 3, 4); */
    /* printf("\n"); */

    /* aa_tick("tf abs: "); */
    /* for( size_t i = 0; i < N; i ++ ) */
    /*     lwa4_tf_abs_( q, aa_tf_ident, T_abs ); */
    /* aa_tock(); */
    /* aa_dump_mat(stdout, T_abs+12*6, 3, 4); */
    /* printf("\n"); */

    aa_tick("memset: ");
    for( size_t i = 0; i < N; i ++ )
        memset(T_rel, 0, sizeof(T_rel) );
    aa_tock();

    aa_tick("numeric-tf: ");
    for( size_t i = 0; i < N; i ++ )
        lwa4_tf_( q, T_rel );
    aa_tock();

    aa_tick("numeric-chain: ");
    for( size_t i = 0; i < N; i ++ ) {
        rfx_kin_tf_chain( 7, aa_tf_ident, T_rel, T_abs );
        aa_tf_12chain( T_abs+6*12, aa_tf_ident, Te );
    }
    aa_tock();

    aa_tick("numeric-jac: ");
    for( size_t i = 0; i < N; i ++ )
        rfx_kin_tf_jac_rev( 7, T_abs, axis[0], Te, J, 6 );
    aa_tock();

    aa_tick("numeric-all: ");
    for( size_t i = 0; i < N; i ++ )
        lwa4_kin2_( q, aa_tf_ident, aa_tf_ident, Te, J );
    aa_tock();
    aa_dump_mat(stdout, Te, 3, 4);
    printf("\n");

    double S_rel[8*7];
    double S_abs[8*7];
    double Se[8];

    aa_tick("duqu-rel: ");
    for( size_t i = 0; i < N; i ++ )
        lwa4_duqu( q, S_rel );
    aa_tock();

    aa_tick("duqu-chain: ");
    for( size_t i = 0; i < N; i ++ ) {
        rfx_kin_duqu_chain( 7, aa_tf_duqu_ident, S_rel, S_abs );
        aa_tf_duqu_mul( T_abs+6*8, aa_tf_duqu_ident, Se );
    }
    aa_tock();

    aa_tick("duqu-jac: ");
    for( size_t i = 0; i < N; i ++ )
        rfx_kin_duqu_jac_rev( 7, S_abs, axis[0], Se, J, 6 );
    aa_tock();


    aa_tick("duqu-all: ");
    for( size_t i = 0; i < N; i ++ )
        lwa4_kin_duqu( q, aa_tf_duqu_ident, aa_tf_duqu_ident, Se, J );
    aa_tock();
    aa_tf_duqu_minimize(Se );
    aa_tf_duqu2tfmat(Se, Te);
    aa_dump_mat(stdout, Te, 3, 4);
    printf("\n");

    /* printf("\n\n"); */
    /* aa_dump_mat(stdout, J, 6, 7); */
    /* printf("\n"); */
    /* aa_dump_mat(stdout, J2, 6, 7 ); */

    /* printf("\n\n"); */
    /* aa_dump_mat(stdout, axis[0], 3, 7 ); */
}
