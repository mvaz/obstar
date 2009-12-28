#include <sys/kos.h>
#include <sys/radio.h>
#include <math.h>
#include <stdlib.h>
#include "sender.h"
#include "units.h"

void
f(process_, SEND ) ()
{
   uint8 buffer[10] = { 2 , 8 };

   int x = MM2PAM(0);
   int y = MM2PAM(150);

   tim_suspend_task(666);

   buffer[5] = ( x >> 24 ) & 0xFF;
   buffer[4] = ( x >> 16 ) & 0xFF;
   buffer[3] = ( x >> 8 )  & 0xFF;
   buffer[2] =   x         & 0xFF; 

   buffer[9] = ( y >> 24 ) & 0xFF;
   buffer[8] = ( y >> 16 ) & 0xFF;
   buffer[7] = ( y >> 8 )  & 0xFF;
   buffer[6] =   y         & 0xFF; 

   radio_sndBuffer( buffer );

   tim_suspend_task(5000);
 
   x = MM2PAM(-100);
   y = MM2PAM(0);

   buffer[5] = ( x >> 24 ) & 0xFF;
   buffer[4] = ( x >> 16 ) & 0xFF;
   buffer[3] = ( x >> 8 )  & 0xFF;
   buffer[2] =   x         & 0xFF; 

   buffer[9] = ( y >> 24 ) & 0xFF;
   buffer[8] = ( y >> 16 ) & 0xFF;
   buffer[7] = ( y >> 8 )  & 0xFF;
   buffer[6] =   y         & 0xFF; 

   radio_sndBuffer( buffer );
 
   exit(0);
}
