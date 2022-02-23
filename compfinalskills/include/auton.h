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
void driveStop(){
  MotorRB.stop(hold);
  MotorLB.stop(hold);
  MotorLF.stop(hold);
  MotorRF.stop(hold);
}
double clip(double number, double min, double max) {
  if (number < min) {
    number = min;
  } else if (number > max) {
    number = max;
  }
  return number;
}
void balance(double power){
  while(imu.pitch(rotationUnits::deg) > 5 || imu.pitch(rotationUnits::deg) < -5){
    MotorLB.setVelocity(power, velocityUnits::pct);
    MotorLF.setVelocity(power, velocityUnits::pct);
    MotorRB.setVelocity(power, velocityUnits::pct);
    MotorRF.setVelocity(power, velocityUnits::pct);
    MotorLF.spin(directionType::fwd);
    MotorRF.spin(directionType::fwd);
    MotorLB.spin(directionType::fwd);
    MotorRB.spin(directionType::fwd);
  }
  driveStop();
}
void moveIn(double inches, int poder) {
  double revs = inches / (4.000 * M_PI);
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

  // Prints the referenceHeading for debugging puroses to ensure that it is
  // going for the right degree amount

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
  driveStop();
  imu.setHeading(0, rotationUnits::deg);
}
void gyroTurnPID(double referenceHeading, double kp, double ki,double kd) {
  double lastError = 0;
  float integralSum = 0;
  timer period = timer();

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
  driveStop();
}

void turnOnPID(double gyroRequestedValue, double MaxspeedinRPM) {
  float gyroSensorCurrentValue;
  float gyroError;
  float gyroDrive;
  float lastgyroError;
  float gyroP;
  float gyroD;

  const float gyro_Kp = 0.573;
  const float gyro_Ki = 0.4;
  const float gyro_Kd = 0.18;

  int TimeExit = 0;
  double Threshold = 1.5;
  while (1) {
    gyroSensorCurrentValue = imu.rotation(vex::rotationUnits::deg);
    Brain.Screen.setCursor(3, 1);

    gyroError = gyroRequestedValue - gyroSensorCurrentValue;

    if (gyroError < Threshold and gyroError > -Threshold) {
      break;
    } else if (TimeExit == 10000) {
      Brain.Screen.clearScreen();
      driveStop();
      break;
    } else {
      TimeExit = 0;
    }

    gyroP = (gyro_Kp * gyroError);
    static float gyroI = 0;
    gyroI += gyroError * gyro_Ki;
    if (gyroI > 1) {
      gyroI = 1;
    }
    if (gyroI < -1) {
      gyroI = -1;
    }
    gyroD = (gyroError - lastgyroError) * gyro_Kd;
    gyroDrive = gyroP + gyroI + gyroD;

    if (gyroDrive > MaxspeedinRPM) {
      gyroDrive = MaxspeedinRPM;
    }
    if (gyroDrive < -MaxspeedinRPM) {
      gyroDrive = -MaxspeedinRPM;
    }

    // Move Motors with PID
    int powerValue = gyroDrive;
    MotorRF.spin(vex::directionType::rev, (powerValue), vex::velocityUnits::rpm);
    MotorLF.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    MotorRB.spin(vex::directionType::rev, (powerValue), vex::velocityUnits::rpm);
    MotorLB.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);

    lastgyroError = gyroError;
    wait(10, vex::timeUnits::msec);
  }
  
  driveStop();
}
double angleCalc(double requestedAngle){
  double revs;
  revs = (((sqrt(574.65) * M_PI) / 360) * requestedAngle) / 4 * M_PI;
  return revs;
}
void turnNoIMU(double referenceHeading, double power, double kp, double ki, double kd){
  double lastError = 0;
  float integralSum = 0;
  timer period = timer();
  MotorLF.resetRotation();
  if (referenceHeading < 0) {
    while ((double)MotorLB.rotation(rotationUnits::rev) <= (double)angleCalc(referenceHeading)) {
      period.reset();
      double error;
      double imuHeading = MotorLB.rotation(rotationUnits::rev);
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
  }
   else {
    while ((double)MotorLB.rotation(rotationUnits::rev) >= (double)angleCalc(referenceHeading)){
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
  driveStop();
}

void turnAngle(double angle, double poder){
  if (angle > 0){
    MotorLF.rotateTo((double)angleCalc(angle), rotationUnits::rev, poder, velocityUnits::pct, false);
    MotorRF.rotateTo((double)angleCalc(angle) , rotationUnits::rev, -poder, velocityUnits::pct, false);
    MotorLB.rotateTo((double)angleCalc(angle), rotationUnits::rev, poder, velocityUnits::pct, false);
    MotorRB.rotateTo((double)angleCalc(angle) , rotationUnits::rev, -poder, velocityUnits::pct, true);
  }
  else if(angle < 0){
    MotorLF.rotateTo((double)angleCalc(angle), rotationUnits::rev, -poder, velocityUnits::pct, false);
    MotorRF.rotateTo((double)angleCalc(angle) , rotationUnits::rev, poder, velocityUnits::pct, false);
    MotorLB.rotateTo((double)angleCalc(angle), rotationUnits::rev, -poder, velocityUnits::pct, false);
    MotorRB.rotateTo((double)angleCalc(angle) , rotationUnits::rev, poder, velocityUnits::pct, true);
  }
  driveStop();
}

void moveInches(double inches, double poder){
  MotorLF.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, false);
  MotorRF.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, false);
  MotorLB.rotateTo((inches / (4 * M_PI)), rotationUnits::rev, poder, velocityUnits::pct, false);
  MotorRB.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, true);
}
#endif