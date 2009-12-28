#define   TAUTAR                 40.0
#define   LAMBDA_TAR(ttar, dt)    ( (real) ( 1.0 / ( (real) (ttar)*(dt) ) ) )

/*
 * MM
 */
#define   CLOSE             20
#define   CLOSE2TAR         100

/*
 * PAM/TIC
 */ 
#define   V_TAR             5.0

#define   LAMBDA_V          2.0

/*
 * SECONDS
 */
#define   T2C               125.0
#define   T2TAR             7.0

/*
 * FUNCTION PROTOTYPE
 */
real target_dynamics(real, real, real, real*);
