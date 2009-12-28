#include <sys/kos.h>
#include <sys/radio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "obstar.h"
#include "units.h"
#include "robot.h"
#include "obs.h"
#include "tar.h"
#include "noise.h"
#include "acquisition.h"
#include "config.h"


static void initial_computations (struct config *);

/*
void obstar('slave|master',configuration)
*/

void
proc(OBSTAR) ()
{
   natural   status;
   IRSENSOR *sensor;

   struct config *cfg;

   integer i;
   integer j;

   /* UNIT: milliseconds */
   natural dt              = 0;
   natural dt1             = 0;
   natural dt2             = 0;
   natural time_cicle_init = 0;
   natural time_cicle_end  = 0;

   natural time_at_init;
   natural time_at_end;
   natural total_time;

   /* UNIT: tens of milliseconds */
   real    dt_tic          = 0.0;

   /* UNIT: RAD/TIC*/
   real    W_times_r       = 0.0;
   real    true_dynamics   = 0.0;
   real    tot_dynamics    = 0.0;
   real    tar_dynamics    = 0.0;
   real    obs_dynamics    = 0.0;
   real    obs_potential   = 0.0;

   /* UNIT: PAM */
   integer dX              = 0;
   integer dY              = 0;
   integer ds              = 0;

   integer Xtarget         = MM2PAM(100);
   integer Ytarget         = MM2PAM(100);

   /* UNIT: RAD */
   real    dphi            = 0.0;

   /* */
   natural sensBuf[ NIR ] = { 0, 0, 0, 0, 0, 0};
   natural indexBuf[NIR ] = { 0, 0, 0, 0, 0, 0};

   real    obsBuf[ NIR]   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   real    potBuf[ NIR]   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   real    disBuf[ NIR]   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   uint8   buffer[18]     = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
   /*
    * TIC
    */
   natural min_dis         = 0;
   /*
    * BEHAVIOUR VARIABLES
    */
   real    v_cm            = 0.0;
   real    true_v_cm       = 0.0;
   real    old_v_cm        = 0.0;
   real    v_des           = 0.0;
   real    v_tar           = 0.0;

   integer v_right_wheel   = 0;
   integer v_left_wheel    = 0;

   real    Psi_Tar         = 0.0;

   real    noise           = 0.0;

   real    dtarget         = 0.0;
   
   /* Get the pointer to where the values of the IR sensors are stored */
   do {
      status = tim_find_association(IR_ASSOC_STRING);
   } while ( status < 0 );
   sensor = (IRSENSOR*) status;

   cfg = init();
   initial_computations (cfg);

   /*
    SYNCHRONISATION WITH MASTER
    */
   while ( !( radio_getStatus() & 2 ) )
   {
      tim_suspend_task(100);
   }
   /* get coordinates */

   radio_recBuffer(buffer);

   Xtarget = buffer[2] + ( buffer[3] << 8 ) + ( buffer[4] << 16 ) + ( buffer[5] << 24 ); 
   Ytarget = buffer[6] + ( buffer[7] << 8 ) + ( buffer[8] << 16 ) + ( buffer[9] << 24 );

   /*tim_suspend_task(20000);*/
   time_at_init = tim_get_ticcount();

   /* Do obstacle avoidance 
    *
    * The value '30' to dt2 is an approximation, valid only for first cicle
    * problems concerning division by zero..
    */
    
   printf("Xtarget: %d Ytarget: %d\r\n", Xtarget, Ytarget);

   for ( j = 0, dt2 = 30,
                dX  = Xtarget - (cfg->Xrobot),
                dY  = Ytarget - (cfg->Yrobot),
                dtarget  = sqrt( SQUARE( dX ) + SQUARE( dY ) ),
                time_cicle_end = tim_get_ticcount() ;
         ( j < cfg->nr_cicles ) && ( dtarget > cfg->close );
         j++ ) 
   {
      /* Concentrate all resources to the calculations in this cicle */
      tim_lock();
      time_cicle_init = tim_get_ticcount();
      dt1             = time_cicle_init - time_cicle_end;
      /* 
       * 'dt' is the value of time that will be used in the Euler step 
       *      method in the dead reckoning process. It's is the amount
       *      of time during which the robot has travelled using the 
       *      current values of the behaviour variables.
       *      It is computed from two othe values:
       *      
       *      dt1 - the time that has gone from the end of the last
       *            cicle until the beginning of the present one.
       *      
       *      dt2 - the time that has gone from the beginning until the end 
       *            of the last cicle. We postulate this value to be constant,
       *            since that at the beginning of every cicle we tell the
       *            robot's scheduler not to interrupt it.
       */
      dt        = dt1 + dt2;
      /*
       * The value of dt is expressed in milliseconds, so, in order to do the
       * dead reckoning, and since scalar velocity is expressed in PAM/TIC and
       * and angular speed is expressed in RAD/TIC, it has to be converted to TIC.
       */

      dt_tic    = dt * 0.1;      
      /*
       * Since the wheels only accept discrete values for their velocities, and
       * they are computed from a desired angular speed (which yelds values real
       * values) , they incorporate an error. The dead reckoning process should
       * avoid these errors, and base itself on the actual values of the velocities
       * of each wheel.
       */
      true_dynamics   = ( v_right_wheel - v_left_wheel ) / ( RADIUS * 2.0 );
      true_v_cm       = ( v_right_wheel + v_left_wheel ) / ( 2.0 );
      
      /*
       *
       */
      ds              = true_v_cm     * dt_tic;
      dphi            = true_dynamics * dt_tic;
      cfg->Phi       += dphi;
 
      /*
       * Maintain the angle in the [ 0 , 2*PI [ interval 
       */
      cfg->Phi        = REDUCE_ANGLE(cfg->Phi);

      cfg->Xrobot     = cfg->Xrobot  + (integer) rint( ds * COS( cfg->Phi ) );
      cfg->Yrobot     = cfg->Yrobot  + (integer) rint( ds * SIN( cfg->Phi ) );

      dX              = Xtarget - (cfg->Xrobot);
      dY              = Ytarget - (cfg->Yrobot);
      dtarget         = sqrt( SQUARE( dX ) + SQUARE( dY ) );

      /*
       * TARGET thing
       */
      tar_dynamics    = LAMBDA_TAR( cfg->tautar, dt_tic) *
                          target_dynamics( (real) dX, (real) dY, cfg->Phi, &Psi_Tar );
            
      /*
       *  NO_OBS_DETECTED
       */
      for ( i = 0, min_dis       = MM2PAM( NO_OBS_DETECTED ),
                   obs_dynamics  = 0.0,
                   obs_potential = 0.0;
            i < NIR ;
            i++ )
      {
         sensBuf[i]     = sensor->oProximitySensor[i];
         indexBuf[i]    = IR2INDEX( sensBuf[i] );
         disBuf[i]      = distances[i][ indexBuf[i] ];
         obsBuf[i]      = obsavoid[i][  indexBuf[i] ];
         potBuf[i]      = potential[i][ indexBuf[i] ];

         /* descobrir a distancia minima */
         min_dis        = MIN( min_dis , disBuf[i] ); 
         obs_dynamics  += obsBuf[i];
         obs_potential += potBuf[i];
         
      }

      /*   
       * Desired velocity for target contribution
       */
      v_tar    = ( dtarget < cfg->close2tar ? dtarget/(cfg->t2tar * dt) : cfg->vtar );
      /*   
       * Desired velocity considering target and obstacle contributions
       */
      v_des    = ( obs_potential > 0     ? ( min_dis - ( 25.0 / 0.08 ) )/(cfg->t2c * dt)    : v_tar );
		/*
      v_des    = ( indexBuf[2] == 16 ||  indexBuf[3] == 16 ) ?   -1   : v_des; 
		*/
      old_v_cm = v_cm;
      v_cm     = old_v_cm - dt_tic * INVSEC_2_INVTIC(cfg->lambdav) * ( old_v_cm - v_des );
      
      /*   
      printf("dtarget: %f, dtarget/T2C(dt): %f, vdes: %f\r\n", dtarget,
                                                               dtarget/T2C(dt),
                                                               v_des );
       */

      /* */
      noise           = ( obs_potential > 0 ? 3 : 1 ) * cfg->sqrt_q * gaussian_noise[ j % N_NOISE_SAMPLES ];

      obs_dynamics   *= BETA1( cfg->tauobs, dt_tic);

      /* integrate target following and obstacle avoidance */
      tot_dynamics    = obs_dynamics + tar_dynamics + noise;

      /* unit is Radians/tic, and it should be discrete. */
      W_times_r       = tot_dynamics * RADIUS;

      /* Compute right and left wheels' velocities from the dynamics values */
      /* FIXME */
      v_right_wheel   = (integer) rint( v_cm + W_times_r );
      v_left_wheel    = (integer) rint( v_cm - W_times_r );

      /* Tell the robot to follow the computed velocities */
      mot_new_speed_2m( v_right_wheel, v_left_wheel);

      /*
       * Determination of the cicle end and the start of the new one. 
       * dt2 is also computed, so that it can be used as an approximation of
       * the next cicle length.
       */
      time_cicle_end  = tim_get_ticcount();
      dt2             = time_cicle_end - time_cicle_init;

      /* release resources */
      tim_unlock();

      if ( radio_getStatus() & 2 )
      {
         radio_recBuffer(buffer);
                                                                                                                     
         Xtarget    = buffer[2] + ( buffer[3] << 8 ) + ( buffer[4] << 16 ) + ( buffer[5] << 24 ); 
         Ytarget    = buffer[6] + ( buffer[7] << 8 ) + ( buffer[8] << 16 ) + ( buffer[9] << 24 );
         
         v_master   = buffer[10]; /* NOT! */
         phi_master = buffer[11]; /* NOT! */
      }
   }

   /* STOP */
   time_at_end = tim_get_ticcount();
   total_time  = time_at_end - time_at_init;

   /*
   printf("Xrobot:%f, Yrobot:%f\r\n", PAM2MM(Xrobot), PAM2MM(Yrobot) );
   printf("end:%d, init:%d, j:%d, dt_medio: %f\r\n", time_at_end, time_at_init, j, (1.0*total_time)/j );
   */
   printf("DONE\r\n"
          "Xrobot: %f, Yrobot: %f\r\n", PAM2MM(cfg->Xrobot), PAM2MM(cfg->Yrobot));
   mot_stop();
   exit(0);
}


/*
 * Performs all necessary initial computations, namely the conversion
 * of units of distance and angles. But, more important, it computes
 * the values of the lookuptable, which will determine the behaviour of
 * the robot.
 */

static void
initial_computations (struct config *cfg)
{
   integer i,j;
   real    sigma;
   real    sigma_square;
   real    lambda;
   real    gaussian;
   real    one_over_sqrt_e = exp( -0.5);

   for ( i = 0 ; i < NIR ; i++ )
   {
      for ( j = 0 ; j < N_LOOKUP_VALUES ; j++ )
      {
         lambda          = LAMBDA( distances[i][j] , cfg->beta2  );
         sigma           = SIGMA(  distances[i][j], i);
         sigma_square    = SQUARE( sigma             );
         gaussian        = exp( - 0.5 * theta_square[i] / sigma_square );

         obsavoid[i][j]  = lambda * theta[i] * gaussian;
         potential[i][j] = lambda * sigma_square * ( gaussian - one_over_sqrt_e );
      }
   }
   cfg->updated = 1;
}

/*
 * indexes   - array of NIR indexes for the lookuptable
 * Phi_Robot - angle, in radians, representing the direction of navigation        
 * Psi_Tar   - angle, in radians, representing the direction of the target
 * dt        - cicle time in miliseconds
 */

static void
send ( natural *indexes, real Phi_Robot, real Psi_Tar, natural dt)
{
   int i;
   int DATA_BYTES = NIR + 2 + 2 + 1;
   int NBYTES     = DATA_BYTES + 2 + 1;
   unsigned char *angle;
   unsigned char strout[ NBYTES ];

   strout[0]      = SYNC;
   strout[1]      = DATA_BYTES;

   for (i=0; i < NIR ; i++)
      strout[i+2] = indexes[i] + OFFSET;

   /* Conversion of Phi_Robot */
   angle           = radians_2_ascii(Phi_Robot, 32);
   strout[NIR+2  ] = angle[0] + OFFSET;
   strout[NIR+2+1] = angle[1] + OFFSET;
   /*
   printf("%f %d %d %d %d - ",  Phi_Robot, angle[0], strout[NIR+2  ],
                                           angle[1], strout[NIR+2+1]);
   */
   free(angle);

   /* Conversion of Psi_Tar */
   angle           = radians_2_ascii(Psi_Tar  , 32);
   strout[NIR+2+2] = angle[0] + OFFSET;
   strout[NIR+2+3] = angle[1] + OFFSET;
   /*
   printf("%f %d %d %d %d\r\n", Psi_Tar  , angle[0], strout[NIR+2+2],
                                           angle[1], strout[NIR+2+3]);
   */
   free(angle);

   /* dt */
   strout[NIR+2+4] = dt + OFFSET;

   /* last byte */
   strout[NBYTES - 1] = STOP_BYTE;

   /*
   printf("%4d%4d%4d%4d%4d%4d %f %4d%4d %f %4d%4d %4d\r\n", strout[2] ,
                                                      strout[3] ,
                                                      strout[4] ,
                                                      strout[5] ,
                                                      strout[6] ,
                                                      strout[7] ,

                                                      Phi_Robot ,

                                                      strout[8] ,
                                                      strout[9] ,

                                                      Psi_Tar   ,

                                                      strout[10],
                                                      strout[11],

                                                      strout[12]);
 
   */
   /*
   printf("phi: %f, psi_tar: %f, dt: %d\r\n", Phi_Robot, Psi_Tar, dt);
   printf("%s",strout);
   */
   fflush( stdout );
}

/*
 * returns array of two integers representing another integer
 * value in the base 'base', resulting from the conversion of
 * alpha to Degrees.
 */
unsigned char *radians_2_ascii(float alpha, natural base) 
{
   /* */
   natural alpha_deg;

   unsigned char *output = (unsigned char*)calloc(2, sizeof(natural));

   alpha_deg = (natural) rint( RAD_2_DEG(alpha) );

   output[1] = alpha_deg % base;
   output[0] = alpha_deg / base;

   /*
   printf("%f %d %d\r\n", alpha, output[0], output[1]);
   */
   
   return output;
}

/* Efficient calculation of the target dynamics 
 */ 
real target_dynamics(real dx, real dy, real Phi_Robot, real* Psi_Tar)
{
   real sine_approx;
   /* */
   if ( dx < 0 )
   {
      (*Psi_Tar) = atan( dy/dx ) + PI;
      /* 2nd Quadrant */
      if ( (*Psi_Tar) < 3/2*PI)
      {
         if ( Phi_Robot > (*Psi_Tar) + PI/2 )
            sine_approx = Phi_Robot - (*Psi_Tar) - PI;
         else if ( Phi_Robot > (*Psi_Tar) - PI/2 )
            sine_approx = (*Psi_Tar) - Phi_Robot;
         else
            sine_approx = Phi_Robot - (*Psi_Tar) + PI;
      }
      else { /* 3rd Quadrant */
         if ( Phi_Robot < (*Psi_Tar) - PI/2 )
            sine_approx = Phi_Robot - ( (*Psi_Tar) - PI);
         else if ( Phi_Robot < (*Psi_Tar) + PI/2)
            sine_approx = (*Psi_Tar) - Phi_Robot;
         else
            sine_approx = Phi_Robot - (*Psi_Tar) - PI;
      }
   }
   else if ( dx > 0 )
   {
      /* 4th Quadrant */
      if( ( (*Psi_Tar) = atan( dy/dx ) ) < 0 )
      {
         (*Psi_Tar) += 2*PI;
         if ( Phi_Robot > (*Psi_Tar) - PI/2 )
            sine_approx = (*Psi_Tar) - Phi_Robot;
         else if ( Phi_Robot > (*Psi_Tar) - 3/2*PI)
            sine_approx = Phi_Robot - (*Psi_Tar) + PI;
         else
            sine_approx = (*Psi_Tar) - 2*PI - Phi_Robot;
      } /* 1st Quadrant */
      else {
         if ( Phi_Robot < (*Psi_Tar) + PI/2)
            sine_approx = (*Psi_Tar) - Phi_Robot;
         else if ( Phi_Robot < (*Psi_Tar) + 3/2*PI)
            sine_approx = Phi_Robot - (*Psi_Tar) + PI;
         else
            sine_approx = (*Psi_Tar) + 2*PI - Phi_Robot;
      }
   }
   else
   {
      if (dy > 0)
      {
         (*Psi_Tar) = PI/2;
         sine_approx = 0;
      }
      else if ( dy < 0)
      {
         (*Psi_Tar) = -PI/2;
         sine_approx = 0;
      }
      else { /* CHEGOU!!!! */
         (*Psi_Tar) = 0;
         sine_approx = 0;
      }
   }
   return sine_approx;
}
