#include <sys/kos.h>
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
#include "interpreter.h"
#include "history.h"


static void initial_computations (struct config *);
static struct history *obstar(integer, integer, struct config *, struct history *);

void
proc(OBSTAR) ()
{

   char comm_line[LINEMAX];
   char buffer[LINEMAX];
   char *answer;

   char *clone_line ;
   char *token;

   char *argv[MAX_ARGS];
   int   argc;
   int   i;
   int   h,k;
   int   proceed = 1;

   struct config  *cfg;
   struct history *hist = NULL;

   real     real_tmp = 0.0;
   integer  int_tmp  = 0;

   cfg = init();
   
   while ( proceed && printf( PROMPT ) && GETLINE(comm_line) )  
   {
      /* argv and argc should be reset? Or just argc? */
      argc       = 0;
      clone_line = strdup( comm_line );
      
      token      = strtok( clone_line , SEPS);
     
      do {
         argv[argc] = token;
         argc++;
      } while ( ( token = strtok(NULL, SEPS) ) );
   
   /* 
    * Next we shall identify the command, from the following list
    * Maybe the order should be modified with a quicksort like method...
    * 
    *    exit
    *    set
    *    goto
    *    get
    */
      if ( EQUALS( "set",  argv[0] ) )
      {
         if ( argc == 1 )
            printf("set what?\r\n");

         /* Possibilities of parameters to be set
          *   TODO update list
          *   Xrobot
          *   Yrobot
          *   Phi
          *   Q - gaussian noise
          *   tauobs
          *   b2
          *   tautar
          *   
          *   velocities parameters...
          *      LambdaV
          *      Vtar
          */
         for ( i = 1 ; i < argc - 1 ; i++ )
         {
            if ( EQUALS( "tobs", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               if ( VALID_TOBS( real_tmp ) )
                  cfg->tauobs   = real_tmp;
               else
                  printf( INVALID_TOBS );

            } 
            else if ( EQUALS( "ttar", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               if ( VALID_TTAR( real_tmp ) )
                  cfg->tautar   = real_tmp;
               else
                  printf( INVALID_TTAR );
            } 
            else if ( EQUALS( "t2c", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               if ( VALID_T2C( real_tmp ) )
                  cfg->t2c      = real_tmp;
               else
                  printf( INVALID_T2C );
            }
            else if ( EQUALS( "lambdav", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               if ( VALID_LAMBDAV( real_tmp ) )
                  cfg->lambdav  = real_tmp;
               else
                  printf( INVALID_LAMBDAV );
            } 
            else if ( EQUALS( "t2tar", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               if ( VALID_T2TAR( real_tmp ) )
                  cfg->t2tar    = real_tmp;
               else
                  printf( INVALID_T2TAR );
            } 
            else if ( EQUALS( "b2", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               if ( VALID_B2( real_tmp ) )
               {
                  cfg->beta2   = MM2PAM( real_tmp );
                  cfg->updated = 0;
               }
               else
                  printf( INVALID_B2 );
            } 
            else if ( EQUALS( "q", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               if ( VALID_Q( real_tmp ) )
                  cfg->sqrt_q = real_tmp;
               else
                  printf( INVALID_Q );
            }
            else if ( EQUALS( "x", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               cfg->Xrobot = MM2PAM( real_tmp );
            }
            else if ( EQUALS( "y", argv[i] ) )
            {
               ++i;
               real_tmp = (real) atof( argv[i] );
               cfg->Yrobot = MM2PAM( real_tmp );
            }
            else if ( EQUALS( "phi", argv[i] ) )
            {
               ++i;
               int_tmp  = (integer) atoi( argv[i] );
               if ( VALID_PHI( real_tmp ) )
                  cfg->Phi = DEG_2_RAD( int_tmp );
               else
                  printf( INVALID_PHI );
            }
            else if ( EQUALS( "close", argv[i] ) )
            {
               ++i;
               real_tmp  = (real) atof( argv[i] );
               if ( VALID_CLOSE( real_tmp ) )
                  cfg->close = MM2PAM( real_tmp );
               else
                  printf( INVALID_CLOSE );
            }
            else if ( EQUALS( "close2tar", argv[i] ) )
            {
               ++i;
               real_tmp  = (real) atof( argv[i] );
               if ( VALID_CLOSE( real_tmp ) )
                  cfg->close2tar = MM2PAM( real_tmp );
               else
                  printf( INVALID_CLOSE );
            }
            else if ( EQUALS( "nrcic", argv[i] ) )
            {
               ++i;
               cfg->nr_cicles  = (natural) atoi( argv[i] );
            }
            else if ( EQUALS( "cicacq", argv[i] ) )
            {
               ++i;
               cfg->cicles_acq = (natural) atoi( argv[i] );
            }
            else {
               printf("Unknown parameter to be set. Try one of these instead: tobs, b2, q, x, y\r\n");
            }
         }
   
      } else 
      /* Performs initial calculations if they haven't been done yet */
      if ( EQUALS( "update", argv[0]) )
      {
         if (! (cfg->updated) )
            initial_computations( cfg );
      } else 
      /* DUMP HISTORY */
      if ( EQUALS( "dump", argv[0]) )
      {
         if( hist )
            dump_history( hist );
         else 
            printf( NO_HIST_MESSAGE );
         /*
         if (! (cfg->updated) )
            initial_computations( cfg );
         */
      } else 
      /* Prints the lookuptable */
      if ( EQUALS( "print", argv[0]) )
      {
         for ( h=0; h < N_LOOKUP_VALUES ; h++ )
         {
            printf("%d:", h);
            for ( k=0; k < NIR ; k++ )
            {
                printf(" %.8f", obsavoid[k][h]);
            }
            printf("\r\n");
         }
      } else 
      /* Tells the robot to do obsavoidance and target following.
       * The robot expects target coordinates to be integers representing
       * values in milimeters.
       *
       *    goto xtar ytar
       */
      if ( EQUALS( "goto", argv[0]) )
      {
         if ( argc < 3 )
            printf("Bad usage of command GOTO. Should be GOTO Xtar Ytar\r\n");
         else
            hist = obstar( MM2PAM(atoi( argv[1] )) , MM2PAM( atoi( argv[2] ) ), cfg, hist );
         
      } else 
      /* Asks the robot for the values of certain parameters or variables.
       * Should no value be specified, then it will show the values of all
       * variables/parameters.
       */
      if ( EQUALS( "get",  argv[0]) )
      {
         if ( argc == 1 )
         {
            printf("Tau_obs:        %.2f * dt\r\n"
                   "Tau_tar:        %.2f * dt\r\n"
                   "Time 2 contact: %.2f * dt\r\n"
                   "Time 2 target:  %.2f * dt\r\n"
                   "LambdaV:        %.2f\r\n"
                   "B2:             %.2f mm\r\n"
                   "Q:              %f\r\n"
                   "Xrobot:         %.2f mm\r\n"
                   "Yrobot:         %.2f mm\r\n"
                   "Phi:            %d degrees\r\n"
                   "close:          %.2f mm\r\n"
                   "close2tar:      %.2f mm\r\n"
                   "nr cicles:      %d\r\n"
                   "cicles acquisition: %d\r\n",
                           cfg->tauobs  ,
                           cfg->tautar  ,
                           cfg->t2c     ,
                           cfg->t2tar   ,
                           cfg->lambdav ,
                   PAM2MM( cfg->beta2  ),
                           cfg->sqrt_q  ,
                   PAM2MM( cfg->Xrobot ),
                   PAM2MM( cfg->Yrobot ),
       (integer)RAD_2_DEG( cfg->Phi    ),
                   PAM2MM( cfg->close  ),
                   PAM2MM( cfg->close2tar ),
                           cfg->nr_cicles,
                           cfg->cicles_acq
                  );
         }
         else
         for ( i = 1 ; i < argc ; i++ )
         {
            if ( EQUALS( "tobs", argv[i] ) )
            {
               printf("Tau_obs: %.2f * dt\r\n", cfg->tauobs);
            } 
            else if ( EQUALS( "ttar", argv[i] ) )
            {
               printf("Tau_tar: %.2f * dt\r\n", cfg->tautar);
            } 
            else if ( EQUALS( "t2c", argv[i] ) )
            {
               printf("Time 2 contact: %.2f * dt\r\n", cfg->t2c);
            } 
            else if ( EQUALS( "t2tar", argv[i] ) )
            {
               printf("Time 2 target:  %.2f * dt\r\n", cfg->t2tar);
            } 
            else if ( EQUALS( "b2", argv[i] ) )
            {
               printf("B2: %.2f mm\r\n", PAM2MM( cfg->beta2) );
            } 
            else if ( EQUALS( "q", argv[i] ) )
            {
               printf("Q: %f\r\n",  cfg->sqrt_q);
            }
            else if ( EQUALS( "x", argv[i] ) )
            {
               printf("Xrobot: %.2f mm\r\n", PAM2MM(cfg->Xrobot) );
            }
            else if ( EQUALS( "y", argv[i] ) )
            {
               printf("Yrobot: %.2f mm\r\n", PAM2MM(cfg->Yrobot) );
            }
            else if ( EQUALS( "phi", argv[i] ) )
            {
               printf("Phi: %d degrees\r\n", (integer) RAD_2_DEG(cfg->Phi) );
            }
            else if ( EQUALS( "close", argv[i] ) )
            {
               printf("close: %.2f mm\r\n", PAM2MM(cfg->close) );
            }
            else if ( EQUALS( "close2tar", argv[i] ) )
            {
               printf("close2tar: %.2f mm\r\n", PAM2MM(cfg->close2tar) );
            }
            else if ( EQUALS( "nrcic", argv[i] ) )
            {
               printf("nr cicles: %d\r\n", cfg->nr_cicles );
            }
            else if ( EQUALS( "cicacq", argv[i] ) )
            {
               printf("cicles acquisition: %d\r\n", cfg->cicles_acq );
            }
            else {
               printf("Unknown parameter( '%s' ). Try one of these instead: \r\n"
                      "   tobs, ttar, t2c, t2tar, b2, q, x, y, phi, close\r\n", argv[i]);
            }
         }
      } 
      else 
      /* THE END */
      if ( EQUALS( "exit",  argv[0]) )
      {
         printf( EXIT_CONFIRMATION );

         if ( GETLINE(buffer) )
         {
            answer = strtok(buffer, SEPS);

            if ( EQUALS( "yes", answer ) )
            {
               proceed = 0;
            }
         }
      } else 
      /* try again */
      {
         printf("%s: unknown command\r\n", argv[0] );
      }
   }
   printf( FAREWELL );
   bios_restart_system();
   exit(0);
}

/*
 *
 */
static struct history *obstar(integer Xtarget, integer Ytarget, struct config *cfg, struct history *h)
{
   natural   status;
   IRSENSOR *sensor;

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

   /* UNIT: DEG */
   natural phiMaster       = 0;

   /* UNIT: RAD */
   real    dphi            = 0.0;

   /* */
   natural sensBuf[ NIR ] = { 0, 0, 0, 0, 0, 0};
   natural indexBuf[NIR ] = { 0, 0, 0, 0, 0, 0};

   real    obsBuf[ NIR]   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   real    potBuf[ NIR]   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   real    disBuf[ NIR]   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   uint8   buffer[18]     = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

   if ( cfg->updated == 0)
      initial_computations (cfg);

   /*
    * There can be only one!!!!
    */
   if ( h )
      free(h);

   /*
      Cria um history com o numero suficiente de iteracoes
      (lembrar que existe um numero maximo de iteracoes definido na strcut config)
    */
   h = create_history( cfg->nr_cicles );

   /*tim_suspend_task(20000);*/
   time_at_init = tim_get_ticcount();

   /* Do obstacle avoidance 
    *
    * The value '30' to dt2 is an approximation, valid only for first cicle
    * problems concerning division by zero..
    */
    
   /*
   printf("Xtarget: %d Ytarget: %d\r\n", Xtarget, Ytarget);
   */

   dX              = Xtarget - (cfg->Xrobot);
   dY              = Ytarget - (cfg->Yrobot);
   dtarget         = sqrt( SQUARE( dX ) + SQUARE( dY ) );

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

/*
      aux_Phi         = true_v_cm < 0 ? : cfg->Phi;
*/
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
      printf("%d %d %f\r\n", dX, dY, Psi_Tar);
       */
            
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

      /*
      printf("%f %f %f ", dt_tic, BETA1( cfg->tauobs, dt_tic), obs_dynamics);
      */
      obs_dynamics   *= BETA1( cfg->tauobs, dt_tic);
      /*
      printf("%f\r\n0:%f 1:%f 2:%f 3:%f 4:%f 5:%f\r\n", obs_dynamics,
                                                       obsBuf[0], obsBuf[1], obsBuf[2],
                                                       obsBuf[3], obsBuf[4], obsBuf[5]);
      */

      /*
      printf("%f  %f\r\n",BETA1( cfg->tauobs, dt_tic), obs_dynamics );
      */

      /* integrate target following and obstacle avoidance */
      tot_dynamics    = obs_dynamics + tar_dynamics + noise;

      /* unit is Radians/tic, and it should be discrete. */
      W_times_r       = tot_dynamics * RADIUS;

      /* Compute right and left wheels' velocities from the dynamics values */
      /* FIXME */
      v_right_wheel   = (integer) rint( v_cm + W_times_r );
      v_left_wheel    = (integer) rint( v_cm - W_times_r );

      true_v_cm       = ( v_right_wheel + v_left_wheel ) / ( 2.0 );

      /* Tell the robot to follow the computed velocities */
      mot_new_speed_2m(v_right_wheel, v_left_wheel);

      /*
       * Determination of the cicle end and the start of the new one. 
       * dt2 is also computed, so that it can be used as an approximation of
       * the next cicle length.
       */
      time_cicle_end  = tim_get_ticcount();
      dt2             = time_cicle_end - time_cicle_init;

      update_history(h, cfg->Xrobot, cfg->Yrobot, cfg->Phi, sensBuf);

      /* release resources */
      tim_unlock();

      if ( !( j % cfg->cicles_acq ) && ( cfg->cicles_acq > 0 ) )
      {
      /* Send stuff over the radio */
         buffer[2] = ( cfg->Xrobot >> 0  ) & 0xFF; 
         buffer[3] = ( cfg->Xrobot >> 8  ) & 0xFF;
         buffer[4] = ( cfg->Xrobot >> 16 ) & 0xFF;
         buffer[5] = ( cfg->Xrobot >> 24 ) & 0xFF;

         buffer[6] = ( cfg->Yrobot >> 0  ) & 0xFF; 
         buffer[7] = ( cfg->Yrobot >> 8  ) & 0xFF;
         buffer[8] = ( cfg->Yrobot >> 16 ) & 0xFF;
         buffer[9] = ( cfg->Yrobot >> 24 ) & 0xFF;
   
         buffer[10]= (uint8) rint( true_v_cm * 4);         

         phiMaster = RAD_2_DEG( cfg->Phi );

         buffer[11]= phiMaster % 180; 
         buffer[12]= phiMaster / 180; 

         buffer[0] = -1;
         buffer[1] = 11;

         radio_sndBuffer( buffer );
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
   return h;
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
