#include <sys/kos.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "led.h"
#include "sens.h"
#include "mot.h"
#include "obstar.h"



/* 
 * MAIN: launches tasks and performs all initial computations required
 */
int
main( void )
{
   
   uint32 status;

   com_reset  ();
   var_reset  ();
   sens_reset ();
   str_reset  ();
   mot_reset  ();

   status = install_task ( prName(LED), 800, proc(LED) );
   if (status == -1)
      exit (0);
   vIDProcess[LED] = (uint32) status; 

   status = install_task ( prName(SENS), 800, proc(SENS) );
   if (status == -1)
      exit (0);
   vIDProcess[SENS] = (uint32) status; 

   status = install_task ( prName(OBSTAR), 800, proc(OBSTAR) );
   if (status == -1)
      exit (0);
   vIDProcess[OBSTAR] = (uint32) status; 

   exit(0);
}
