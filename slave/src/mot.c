#include <sys/kos.h>

#include "mot.h"

void
f(process_,MOT) ()
{
  mot_config_speed_1m (0, 3800, 800, 100);
  mot_config_speed_1m (1, 3800, 800, 100);
}
