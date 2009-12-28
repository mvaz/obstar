/*
 * structure containing all the values of the parameters that can 
 * be set using the shell.
 */ 
struct config {

   integer Xrobot;
   integer Yrobot;

   integer close;
   integer close2tar;

   real    Phi;

   /* b1 = 1/(tobs*dt)*/
   real    tauobs;
   real    beta2;

   real    tautar;

   real    sqrt_q;

   real    vtar;
   real    lambdav;
   real    t2c;
   real    t2tar;

   natural nr_cicles;
   natural cicles_acq;
   natural updated;
};


struct config *init()
{
   int i,j;
   struct config *cfg;

   cfg = (struct config *) calloc( 1, sizeof(struct config) );
   if ( !cfg )
   {
      printf("Error allocating memory in 'init'\r\n");
      exit(-1);
   }

   cfg->Xrobot  = 0;
   cfg->Yrobot  = 0;

   cfg->close   = MM2PAM( CLOSE );
   
   cfg->Phi     = PI / 2;

   cfg->tauobs  = TAUOBS;
   cfg->tautar  = TAUTAR;

   cfg->beta2   = MM2PAM( BETA2 );

   cfg->sqrt_q  = SQRT_Q;
   cfg->vtar    = V_TAR;

   cfg->close2tar = MM2PAM( CLOSE2TAR );
   cfg->lambdav = LAMBDA_V;

   cfg->t2c     = T2C;
   cfg->t2tar   = T2TAR;

   cfg->nr_cicles  = 500;

   cfg->cicles_acq = CICLES_ACQ;

   cfg->updated    = 0;

   /* other initializations */
   RADIUS       = MM2PAM( RADIUS_MM );
   FICT_RADIUS  = MM2PAM( FICT_RADIUS_MM );
   

   for ( i = 0 ; i < NIR ; i++ )
	{
      theta[i]        = DEG_2_RAD( theta[i] );
      theta_square[i] = SQUARE( theta[i] );
      delta_theta[i]  = DEG_2_RAD( delta_theta[i] );

      for ( j = 0 ; j < N_LOOKUP_VALUES ; j++ )
      {
         distances[i][j] = (real) MM2PAM( distances[i][j]   );
      }
   }

   return cfg;
}
