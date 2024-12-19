#ifndef DEBUG_H
#define DEBUG_H

#include "globals.h"

#ifdef DBG

void setupDEBUG();

template <typename T>
void printlnDEBUG(T symb)
{
  Serial.println(symb);
}

template <typename T>
void printDEBUG(T symb)
{
  Serial.print(symb);
}

#endif

#endif
