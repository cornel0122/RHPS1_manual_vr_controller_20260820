#include "../../ANAAvatarController.h"
#include "../JoystickControl.h"

inline double deadzoned_input(double input, double dead)
{
  if(abs(input) < dead) { return 0.0; }
  if(input > 0) { return (input - dead) / (1.0 - dead); }
  return (input + dead) / (1.0 - dead);
}
