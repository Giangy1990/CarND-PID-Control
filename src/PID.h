#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  std::vector<double> p;
  std::vector<double> pd;
  
  bool first_run;
  bool first_twiddle;
  int observation_steps;
  int steady_state_steps;
  int step_count;
  double total_error;
  double best_error;
  int p_idx;
  double twiddle_threshold;
  int twiddle_step;
  
  void twiddle(double);
};

#endif  // PID_H