#ifndef CONT_H
#define CONT_H

#include "wifiaccesspoint.h"
#include "estimate.h"
#include "looptimer.h"
#include "estimate.h"
#include "pidcontroller.h"
#include "quaterniondigit.h"
#include "globals.h"
#include <vector>

void setupCONT();
void setCONT();

#ifdef DBG
void printCONTCal();
#endif

vector<float> getdifference(vector<float>& one, vector<float>& two);
void interpretrc();
float tolimit(float ans);

#endif
