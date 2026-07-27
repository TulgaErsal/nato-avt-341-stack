/**
* \class PidController
*
* A simple Proportional-Integral-Derivative (PID) controller.
* Controller is generic, but used for speed control in this application.
*
* \author Chris Goodin
* \author James R Baxter
*
* \date 8/31/2020
*/
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <fstream>

namespace avt_341 {
namespace control{

class AntiWindupMethod{
public:
  const static std::string ResetOnSetpoint;
  const static std::string IntegralClamping;
  const static std::string Disabled;
};


class PidController{
 public:
  PidController(const std::string & anti_windup_method, double output_min, double output_max);
  PidController();

  ~PidController();

  double GetControlVariable(double measured_value, double dt);

  void SetSetpoint(double setpoint){setpoint_ = setpoint;}

  void SetKp(double kp){kp_=kp;}

  void SetKi(double ki){ki_ = ki;}

  void SetKd(double kd){kd_ = kd;}

  void SetOvershootLimiter(bool osl){ overshoot_limiter_ = osl; }
  void Reset();

  void SetStayPositive(bool sp){ stay_positive_ = sp; }

  void SetUseFeedForward(bool uff){ use_feed_forward_ = uff; }

  void SetForwardModelParams(double a0, double a1, double a2){
    ff_a0_ = a0;
    ff_a1_ = a1;
    ff_a2_ = a2;
  }

  void SetIntegralAbsMax(double integral_abs_max){ integral_abs_max_ = integral_abs_max; }

 private:
  double kp_;
  double ki_;
  double kd_;
  double setpoint_;
  double previous_error_;
  double integral_;
  bool overshoot_limiter_;
  bool integral_clamping_;
  bool crossed_setpoint_;
  bool stay_positive_;

  // feed forward model parameters
  bool use_feed_forward_;
  double ff_a1_;
  double ff_a2_;
  double ff_a0_;
  std::ofstream fout_;
  double integral_abs_max_;
  double output_min_;
  double output_max_;
};

} // namespace control
} // namespace avt_341
#endif
