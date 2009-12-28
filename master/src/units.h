/********************************************************
 *******************                  *******************
 ******************* UNIT CONVERSIONS *******************
 *******************                  *******************
 **************** last updated: 14.03.2002 **************
 ********************************************************/


/*
 *   TYPE DEFINITIONS
 */
typedef unsigned int natural;
typedef          int integer;
typedef       double real;

/*
 * TIME
 *
 * user unit:    seconds (SEC)
 * program unit: miliseconds (MS)
 *               TIC ( 1/100 second )
 */

#define  SEC2MS(t)            ( (natural) rint ( ( t ) * 1000.0 ) )
#define  MS2SEC(t)            ( (real)           ( t ) / 1000.0 ) 

#define  SEC2TIC(t)           ( (real) ( t ) * 100.0 )
#define  TIC2SEC(t)           ( (real) ( t ) / 100.0 )

#define  INVSEC_2_INVTIC(t)   ( (real) ( t ) / 100.0 )
#define  INVTIC_2_INVSEC(t)   ( (real) ( t ) * 100.0 )

/*
 * SPACE
 *
 * user unit:     milimeter (MM)
 * program unit:  PAM (MM)
 * 
 * Q: What is PAM?
 * A: It is the name I gave to the robot's internal unit of distance, 
 *    in memory of Pamela Anderson, the great...actress.
 *    ( all credits to Sergio Monteiro )
 *
 * 
 */ 
#define  MM2PAM(s)                 ( (integer) rint( ( s ) / 0.08 ) )
#define  PAM2MM(s)                 ( (real)    ( s ) * 0.08 ) 

/*
 * VELOCITY
 *
 * user unit: mm/s (milimeter per second)
 * program unit: PAM/TIC (8mm/s)
 */
#define  PAMperSEC_2_PAMperTIC(v)  ( (real) ( v ) / 100.0 )
#define  PAMperTIC_2_PAMperSEC(v)  ( (real) ( v ) * 100.0 )

/*
 * ANGULAR
 *
 */
#define  RAD_2_DEG(alpha)          ( (real) ( alpha ) * 180 / PI )
#define  DEG_2_RAD(alpha)          ( (real) ( alpha ) * PI / 180  )

/*
 * Return an equivalent angle in the [ 0 , 2*PI [ interval 
 */
#define  REDUCE_ANGLE(alpha)\
           (real) ( (alpha) > 0 ? ( (alpha) > 2*PI ? (alpha) - 2*PI : (alpha) )\
		                          :   (alpha) + 2*PI );

#define   SIN(x)            sin(x)
#define   COS(x)            cos(x)

/*
 * ANGULAR SPEED
 *
 * user unit: rad/s (radians per second)
 * program unit: rad/TIC
 */
#define  RADpSEC2RADpTIC(w)        ( (real) ( w ) / 100.0  )
#define  RADpTIC2RADpSEC(w)        ( (real) ( w ) * 100.0  )
