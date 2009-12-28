struct history {
   natural           iterations;
   struct iteration *log;
};

struct iteration {
   real    phi;
   integer xrobot;
   integer yrobot;
   uint8   distances[NIR];
};

struct history *create_history(int);
void            dump_history(struct history *);
void            update_history(struct history *, integer, integer, real, natural*);


/*
 * Creates a new history with room for a number 'its' of iterations
 */
struct history *create_history(int its)
{
   struct history *h;
   h      = (struct history*)   calloc (  1, sizeof(struct history) );
   assert(h);

   h->log = (struct iteration*) calloc (its, sizeof(struct iteration) );
   assert(h->log);

   return h;
}

/*
 * Printf's a 'struct history' (one iteration x line )
 */
void            dump_history(struct history *h)
{
   int i;
   assert(h);

   printf("%d\r\n", h->iterations);
   for ( i=0 ;  i < h->iterations ; i++ )
   {
      printf("%4.2f, %4.2f, %.3f, %10.2f, %10.2f, %10.2f, %10.2f, %10.2f, %10.2f\r\n", PAM2MM( (h->log)[i].xrobot ),
                                                       PAM2MM( (h->log)[i].yrobot ),
                                                       RAD_2_DEG( (h->log)[i].phi    ),
                                                       PAM2MM( distances[0][ (h->log)[i].distances[0] ]),
                                                       PAM2MM( distances[1][ (h->log)[i].distances[1] ]),
                                                       PAM2MM( distances[2][ (h->log)[i].distances[2] ]),
                                                       PAM2MM( distances[3][ (h->log)[i].distances[3] ]),
                                                       PAM2MM( distances[4][ (h->log)[i].distances[4] ]),
                                                       PAM2MM( distances[5][ (h->log)[i].distances[5] ]) );
      /*
      printf("%d, %d, %f, %d, %d, %d, %d, %d, %d\r\n", (h->log)[i].xrobot,
                                                       (h->log)[i].yrobot,
                                                       (h->log)[i].phi,
                                                       (h->log)[i].distances[0],
                                                       (h->log)[i].distances[1],
                                                       (h->log)[i].distances[2],
                                                       (h->log)[i].distances[3],
                                                       (h->log)[i].distances[4],
                                                       (h->log)[i].distances[5] );
      */
   }
}

/*

 */
void update_history(struct history* h, integer xr, integer yr, real phir, natural *d)
{
   ( h->log )[ h->iterations ].xrobot       = xr;
   ( h->log )[ h->iterations ].yrobot       = yr;
   ( h->log )[ h->iterations ].phi          = phir;
   ( h->log )[ h->iterations ].distances[0] = d[0];
   ( h->log )[ h->iterations ].distances[1] = d[1];
   ( h->log )[ h->iterations ].distances[2] = d[2];
   ( h->log )[ h->iterations ].distances[3] = d[3];
   ( h->log )[ h->iterations ].distances[4] = d[4];
   ( h->log )[ h->iterations ].distances[5] = d[5];
   ( h->iterations )++;
} 
