/*
 * pidStruct.h
 *
 *  Created on: Mar 9, 2016
 *      Author: mbuvi
 */

#include <stdio.h>
#ifndef SOURCES_PIDSTRUCT_H_
#define SOURCES_PIDSTRUCT_H_

//double sampleTime=100;//update time in milliseconds
static double outmin,outmax;//values to set output limit for pid
/*working variables*/
struct pid{
    double kp,ki,kd;
    double output, Kgain,lastPv,dpv;
    double ITerm;
    double errsum;//Accumulated errors(integral on error
    double lasterr;//value of error from previous calculation
    double setpt;//desired value
    double pv;//process value
    double derr;//derivative on error
};


void pid_init(struct pid *pid);
double pid_out(struct pid *pid,double pv,double setpt);
#endif /* SOURCES_PIDSTRUCT_H_ */