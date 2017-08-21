#ifndef PID_H
#define PID_H

#include <iostream>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Previous cte
   */

  double prev_cte;

  /*
   * twiddle parameters
   */

  int steps;
  int window_size;
  int max_steps;
  double err_tolerance;
  /*
   * probing value
   */
  double dp[3];


  /*
    * best_err
    */
   double err;
   double best_err;

/*
 * Flag that are used in Twiddle step. If err > best_err, then, prob value should be
 * decreased twice and parameter need to be recalculated. Therefore, three flags created
 * for all three parameters.
 * */
   bool kp_decrement = false;
   bool ki_decrement = false;
   bool kd_decrement = false;

  /*
  * Constructor
  */
  PID(int window_size, int max_steps, double err_tolerance);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Twiddle process.
  */
  void Twiddle(double cte);


  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
