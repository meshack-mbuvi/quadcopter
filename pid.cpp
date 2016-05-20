/*
 * pid.c
 *
 *  Created on: Mar 11, 2016
 *      Author: mbuvi
 */
#include "pid.h"
double sampleT=10;
void pid_init(struct pid *pid)
{
//initialize variables
    pid->ITerm=0;
    pid->kp=5;pid->ki=2;pid->kd=1;
    pid->dpv=0.0;pid->lastPv=0.0;
    //pid->Kgain=10;
    pid->derr=0.0;
    pid->errsum=0.0;
    pid->lasterr=0.0;
    pid->output=0.0;pid->pv=0.0;
    pid->setpt=0.0;

}
double pid_out(struct pid *pid,double pv,double setpt)
{
    outmin=0.0f;
    outmax=1.0f;
    double max=-1.0;
    double error=0.0f;static double output=0;
    bool greaterThanZero=0;
    error=setpt-pv;//error signal
    if(error>0)
    {
        greaterThanZero=1;
        }
        else{
            greaterThanZero=0;
            }
    pid->ITerm+=((pid->ki*error)/sampleT)*0.3;//Avoid bumps when ki  changes
    pid->dpv=(pv-pid->lastPv)/sampleT;//derivative on measurement
    pid->errsum+=error*sampleT;
    /*calculating offset for pwm signal*/
    switch(greaterThanZero){
        case 1:
          if((pid->ITerm)>outmax)
          {
              pid->ITerm=outmax;
              }
              else if(pid->ITerm<outmin)
              {
                  pid->ITerm=outmin;
                  }
                  output=((error+pid->ITerm-pid->kd*(pid->dpv))*0.1);//weird calculation for pwm
                  /** limiting output of the pid controller */
                  if(output>outmax)
                  {
                      output=outmax;
                      }
                      else if(output<outmin)
                      {
                          output=outmin;
                          }
        
        break;
        case 0:
                if((pid->ITerm)<max)
          {
              pid->ITerm=max;
              }
              else if(pid->ITerm>outmin)
              {
                  pid->ITerm=outmin;
                  }
                  output=((error+pid->ITerm-pid->kd*(pid->dpv))*0.1);//weird calculation for pwm
                  /** limiting output of the pid controller */
                  if(output<max)
                  {
                      output=max;
                      }
                      else if(output>outmin)
                      {
                          output=outmin;
                          }
           break;          
        
        }
    
    //Remember some variable for next calculation
    pid->lastPv=pv;

    return output;
}

