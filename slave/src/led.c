#include <sys/kos.h>

#include "led.h"
/*
 * Process 0
 * ^^^^^^^^^
 *
 * This process changes the state of the LED 0
 * every 500 ms.
 *
 */

void
f(process_,LED) ()
{

   for (;;)
   {
      tim_suspend_task (900);
      var_change_led (0);
   }
}


