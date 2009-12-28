#include <sys/kos.h>
#include <stdio.h>
#include <stdlib.h>

#include "sens.h"
/*
 * Process 1
 * ^^^^^^^^^
 *
 * This process publishes an association between "IR sens" and a Ptr
 * on the I.R. values; then it dies.
 *
 */

void
f(process_,SENS) ()
{
   int32 status;
   IRSENSOR *sensor;

   sensor = sens_get_pointer ();
   status = tim_define_association (IR_ASSOC_STRING, (uint32 *) sensor);
   if (status < 0)
   {
      printf (IR_ASSOC_GEN_ERROR_STRING);
      exit (IR_ASSOC_GEN_ERROR);
   }

   exit (0);
}
