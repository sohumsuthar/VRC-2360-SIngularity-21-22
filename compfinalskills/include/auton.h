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

void moveIn(float inches, int poder){
  float revs = inches/(4.000 * M_PI);
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

void gyroTurn(double DegreeAmount, int veloc, float decel) {
  int speed = veloc;
  // Set speeds of both Drive motors
  MotorLF.setVelocity(veloc, velocityUnits::pct);
  MotorLB.setVelocity(veloc, velocityUnits::pct);
  MotorRB.setVelocity(veloc, velocityUnits::pct);
  MotorRF.setVelocity(veloc, velocityUnits::pct);

  // Prints the DegreeAmount for debugging puroses to ensure that it is going
  // for the right degree amount


  // While loop to do the spin
  if (DegreeAmount < 0) {
    while (imu.angle(rotationUnits::deg) <= DegreeAmount) {
      MotorLF.spin(directionType::rev); 
      MotorRF.spin(directionType::fwd);
      MotorLB.spin(directionType::rev);
      MotorRB.spin(directionType::fwd);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }

  } else {
    while (imu.angle(rotationUnits::deg) <= DegreeAmount) {
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
#endif