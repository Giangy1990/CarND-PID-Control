#include "PID.h"
#include <limits>
#include <iostream>

PID::PID() {
  p_error = 0;
  i_error = 0;
  first_run = true;
  observation_steps = 100000;
  steady_state_steps = 100;
  step_count = 0;
  total_error = 0;
  best_error = std::numeric_limits<double>::max();
  first_twiddle = true;
  p_idx = 0;
  twiddle_threshold = 0.001;
  twiddle_step = 0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  p = {Kp_, Ki_, Kd_};
  pd = {Kp_*.1, Ki_*.2, Kd_*.2};
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  if (first_run){
    // to set a correct d_error at startup
    p_error = cte;
    first_run = false;
  }
    
  i_error += cte;
  d_error = cte - p_error;
  p_error = cte;
  
  double sum = (pd[0] + pd[1] + pd[2]);
  if ( sum > twiddle_threshold){
    twiddle(cte);
  }
  else{
    std::cout << "PID value = " << p[0] << " , " << p[1] << " , " << p[2] << std::endl;
  }
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  
  return -p[0]*p_error - p[1]*i_error -p[2]*d_error;
}

void PID::twiddle(double cte) {
  /**
   * perform twiddle algrithm to ompimize the PID gains
   */
  
  double error = cte * cte;
  
  if (step_count < steady_state_steps){
    // waith a time to stabilize the control
    step_count++;
  }
  else if (step_count < (steady_state_steps + observation_steps)){
    // evaluate the error
    total_error += error;
    step_count++;
  }
  else{
    switch (twiddle_step){
      case 0:
        if(first_twiddle){
          // if first time, init best error
          best_error = total_error;
          first_twiddle = false;
          break;
        }
        // try to add pd to p
        p[p_idx] += pd[p_idx];
        
        // switch to next step
        twiddle_step = 1;
        break;
      case 1:
        // check the result of the addditon of pd to p
        if (total_error < best_error){
          // improvements
          best_error = total_error;
          pd[p_idx] *= 1.1;
          // set next p and restart the procedure
          p_idx = (p_idx + 1) % 3;
          twiddle_step = 0;
        }
        else{
          // if there isn't an improvement, try to subtract pd to p
          p[p_idx] -= 2*pd[p_idx];
        
          // switch to next step
          twiddle_step = 2;
        }
        break;
      case 2:
        // check the result of the subtraction of pd to p
        if (total_error < best_error){
          // improvements
          best_error = total_error;
          pd[p_idx] *= 1.1;
        }
        else{
          // if there isn't an improvement, restore p
          p[p_idx] += pd[p_idx];
          pd[p_idx] *= .9;
        }
        // set next p and restart the procedure
        p_idx = (p_idx + 1) % 3;
        twiddle_step = 0;
        break;
    }
    // reset the accumulator and the counter
    total_error = 0;
    step_count = 0;
  }
  
  std::cout << "p = " << p[0] << " , " << p[1] << " , " << p[2] << std::endl;
  std::cout << "dp = " << pd[0] << " , " << pd[1] << " , " << pd[2] << std::endl;
}