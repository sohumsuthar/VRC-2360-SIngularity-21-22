#ifndef AUTON_H
#define AUTON_H
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       auton.h                                                   */
/*    Author:       Sohum Suthar                                              */
/*    Created:      Oct 10 2021                                               */
/*    Description:  Competition Program 2360S                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <vex_competition.h>

using namespace vex;
double clip(double number, double min, double max){
  if(number < min){
    number = min;
  }
  else if(number > max){
    number = max;
  }
  return number;
}
void moveIn(double inches, int poder){
  double revs = inches/(4.000 * M_PI);
  MotorLB.setVelocity(poder, velocityUnits::pct);
  MotorLF.setVelocity(poder, velocityUnits::pct);
  MotorRB.setVelocity(poder, velocityUnits::pct);
  MotorRF.setVelocity(poder, velocityUnits::pct);

  MotorLB.spinFor(revs, rotationUnits::rev, false);
  MotorLF.spinFor(revs, rotationUnits::rev, false);
  MotorRB.spinFor(revs, rotationUnits::rev, false);
  MotorRF.spinFor(revs, rotationUnits::rev, true);
}

void move(double revs, double power) {
  MotorLB.setVelocity(power, velocityUnits::pct);
  MotorLF.setVelocity(power, velocityUnits::pct);
  MotorRB.setVelocity(power, velocityUnits::pct);
  MotorRF.setVelocity(power, velocityUnits::pct);

  MotorLB.spinFor(revs, rotationUnits::rev, false);
  MotorLF.spinFor(revs, rotationUnits::rev, false);
  MotorRB.spinFor(revs, rotationUnits::rev, false);
  MotorRF.spinFor(revs, rotationUnits::rev, true);
}

void gyroTurn(double referenceHeading, int veloc) {
  double imuHeading = imu.angle(rotationUnits::deg);
  // Set speeds of both Drive motors
  MotorLF.setVelocity(veloc, velocityUnits::pct);
  MotorLB.setVelocity(veloc, velocityUnits::pct);
  MotorRB.setVelocity(veloc, velocityUnits::pct);
  MotorRF.setVelocity(veloc, velocityUnits::pct);

  // Prints the referenceHeading for debugging puroses to ensure that it is going
  // for the right degree amount


  // While loop to do the spin
  if (referenceHeading < 0) {
    while (imuHeading <= referenceHeading) {
      MotorLF.spin(directionType::rev); 
      MotorRF.spin(directionType::fwd);
      MotorLB.spin(directionType::rev);
      MotorRB.spin(directionType::fwd);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }

  } else {
    while (imuHeading <= referenceHeading) {
      MotorLF.spin(directionType::fwd);
      MotorRF.spin(directionType::rev);
      MotorLB.spin(directionType::fwd);
      MotorRB.spin(directionType::rev);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }
  }

  // Stop motors after reached degree turn
  MotorRB.stop(hold);
  MotorLB.stop(hold);
  MotorLF.stop(hold);
  MotorRF.stop(hold);
  imu.setHeading(0, rotationUnits::deg);
}
void gyroTurnPID(double referenceHeading, int veloc, double kp, double ki, double kd) {
  double lastError = 0;
  float integralSum = 0;
  timer period = timer();
  period.reset();

  // Set speeds of both Drive motors

  // Prints the referenceHeading for debugging puroses to ensure that it is going
  // for the right degree amount

  // While loop to do the spin
  if (referenceHeading < 0) {
    while (imu.angle(rotationUnits::deg) <= referenceHeading) {
      period.reset();
      double error;
      double imuHeading = imu.angle(rotationUnits::deg);
      error = referenceHeading - imuHeading;
      double derivative = (error - lastError) / (double)period.value();
      integralSum = integralSum + (error * (double)period.value());
      double output = clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);

      MotorLF.setVelocity(100 * output, velocityUnits::pct);
      MotorLB.setVelocity(100 * output, velocityUnits::pct);
      MotorRB.setVelocity(100 * output, velocityUnits::pct);
      MotorRF.setVelocity(100 * output, velocityUnits::pct);

      MotorLF.spin(directionType::rev);
      MotorRF.spin(directionType::fwd);
      MotorLB.spin(directionType::rev);
      MotorRB.spin(directionType::fwd);
      lastError = error;
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }

  } else {
    while (imu.angle(rotationUnits::deg) <= referenceHeading) {
      period.reset();
      double error;
      double imuHeading = imu.angle(rotationUnits::deg);
      error = referenceHeading - imuHeading;
      double derivative = (error - lastError) / (double)period.value();
      integralSum = integralSum + (error * (double)period.value());
      double output = clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);

      MotorLF.setVelocity(100 * output, velocityUnits::pct);
      MotorLB.setVelocity(100 * output, velocityUnits::pct);
      MotorRB.setVelocity(100 * output, velocityUnits::pct);
      MotorRF.setVelocity(100 * output, velocityUnits::pct);

      MotorLF.spin(directionType::fwd);
      MotorRF.spin(directionType::rev);
      MotorLB.spin(directionType::fwd);
      MotorRB.spin(directionType::rev);
      lastError = error;
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }
  }

  // Stop motors after reached degree turn
  MotorRB.stop(hold);
  MotorLB.stop(hold);
  MotorLF.stop(hold);
  MotorRF.stop(hold);
  imu.setHeading(0, rotationUnits::deg);
}
#endif