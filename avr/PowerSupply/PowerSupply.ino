#include "pm.h"
#include "i2c.h"
//
void setup()
{
  pm_init();
  i2c_init();
}

//
void loop()
{
  i2c_loop();
  pm_loop();
}

// EOF
