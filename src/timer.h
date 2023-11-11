#include "Arduino.h"

#ifndef __TIMER_H__
#define __TIMER_H__

typedef unsigned long int ulong;

boolean timerElapsed(ulong *timerPointer, ulong timer)
{
  boolean returnValue = false;

  if((millis() - *timerPointer) >= timer)
  {
    *timerPointer = millis();
    returnValue = true;
  }
  return returnValue;
}

#endif
