#include <assert.h>
#include <amino.h>
#include <ach.h>
#include <reflex.h>
#include "piranha.h"



int main(void) {


    /* double q0[7] =  {0, .3, 0, 0.3, 0, 0, 0}; */

    /* double r1[4] = {0.0e0, -0.7071067811865475e0, 0.0e0, 0.7071067811865476e0}; */
    /* double x1[4] = {.33, .1, .405 }; */

    /* double S1[8] = {0.0e0, -0.7071067811865475e0, 0.0e0, 0.7071067811865476e0, */
    /*                 0.25929605666110705e0,  0.0e0, 0.027365032431919442e0, -0.0e0 }; */

    /* aa_tf_qv2duqu( r1, x1, S1 ); */

    double q0[7] = {.3*M_PI, -.25*M_PI, -.25*M_PI, -.25*M_PI, -.25*M_PI, .35*M_PI, 0};
    printf("q0:  ");aa_dump_vec( stdout, q0, 7 );
    double S0[8];
    lwa4_kin_duqu( q0, aa_tf_duqu_ident, aa_tf_duqu_ident, S0, NULL );
    printf("sg:  ");aa_dump_vec( stdout, S0, 8 );
    aa_tf_duqu_minimize( S0  );
    printf("sgm: ");aa_dump_vec( stdout, S0, 8 );

    double q1[7];
    pir_kin_solve( q0, S0, q1 );

    double S1[8];
    lwa4_kin_duqu( q1, aa_tf_duqu_ident, aa_tf_duqu_ident, S1, NULL );

    printf("sg:  ");aa_dump_vec( stdout, S0, 8 );
    printf("sat: ");aa_dump_vec( stdout, S1, 8 );
    printf("q0:  ");aa_dump_vec( stdout, q0, 7 );
    printf("q1:  ");aa_dump_vec( stdout, q1, 7 );
    printf("dot: %f\n", aa_la_dot(7, q1, q1 ) );
    return 0;
}
