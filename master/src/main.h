/*
 *
 *
 */
#define f(s1,s2)    cat(s1,s2)
#define cat(s1,s2)  s1 ## s2
#define proc(X)     f(process_,X)
#define prName(X)   f(prName_,X)


/*
 * Definition of processes and related information
 */
#define N_PROC      4
#define LED         0
#define OBSTAR      1
#define SENS        2
#define MOT         3

uint32 vIDProcess[N_PROC];
/*
static void initial_computations();
*/
/*
 *
 *
 */
#define PI          M_PI
#define SQUARE(x)   ( (x) * (x) )

#define MIN(a,b)    ( (a) < (b) ? (a) : (b) )
#define MAX(a,b)    ( (a) > (b) ? (a) : (b) )
/*
 *
 *
 */
#define IR_ASSOC_STRING           "IR sens"
#define IR_ASSOC_ERROR_STRING     "The IR sens association doesn't exist!\r\n"

#define IR_ASSOC_GEN_ERROR        10
#define IR_ASSOC_GEN_ERROR_STRING "It is impossible to generate an IR sens association!\r\n"
