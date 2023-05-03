/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
    TODO: Initialize PID coefficients (and errors, if needed)
   **/
   this->Kp = Kpi;
   this->Ki = Kii;
   this->Kd = Kdi;
   this->output_lim_max = output_lim_maxi;
   this->output_lim_min = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
    TODO: Update PID errors based on cte.
   **/
   this->diff_cte = (cte - this->prev_cte)/this->dt;
   this->prev_cte = cte;
   this->int_cte += cte*this->dt;
   
}

double PID::TotalError() {
   /**
    TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    control = (prev_cte * Kp) + (int_cte * Ki) + (diff_cte * Kd);
    if (control > output_lim_max)
    {
      control = output_lim_max;
    }
    else if (control < output_lim_min)
    {
      control = output_lim_min;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
    TODO: Update the delta time with new value
   */
   dt = new_delta_time;
   return dt;
}