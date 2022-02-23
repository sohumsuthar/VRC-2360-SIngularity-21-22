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
void driveStop(bool holding){
  if(holding){
    MotorRB.stop(hold);
    MotorLB.stop(hold);
    MotorLF.stop(hold);
    MotorRF.stop(hold);
  }
  else{
    MotorRB.stop();
    MotorLB.stop();
    MotorLF.stop();
    MotorRF.stop();
  }

}
double clip(double number, double min, double max) {
  if (number < min) {
    number = min;
  } else if (number > max) {
    number = max;
  }
  return number;
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
      driveStop(true);
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
  
  driveStop(true);
}

void driveOnPID(double distance, double MaxspeedinRPM) { 
  MotorRB.resetRotation();
  float degs = (distance / (4 * M_PI)) * 360;
  float encoderValue;
  float error;
  float Drive;
  float lastError;
  float P;
  float D;

  const float Kp = 0.35;
  const float Ki = 0.4;
  const float Kd = 0.18;

  int TimeExit = 0;
  double Threshold = 1.5;
  while (1) {
    encoderValue = MotorRB.rotation(vex::rotationUnits::deg);
    
    Brain.Screen.setCursor(3, 1);

    error = degs - encoderValue;

    if (error < Threshold and error > -Threshold) {
      break;
    } else if (TimeExit == 10000) {
      Brain.Screen.clearScreen();
      driveStop(true);
      break;
    } else {
      TimeExit = 0;
    }

    P = (Kp * error);
    static float I = 0;
    I += error * Ki;
    if (I > 1) {
      I = 1;
    }
    if (I < -1) {
      I = -1;
    }
    D = (error - lastError) * Kd;
    Drive = P + I + D;

    if (Drive > MaxspeedinRPM) {
      Drive = MaxspeedinRPM;
    }
    if (Drive < -MaxspeedinRPM) {
      Drive = -MaxspeedinRPM;
    }

    // Move Motors with PID
    int powerValue = Drive;
    MotorRF.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    MotorLF.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    MotorRB.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    MotorLB.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);

    lastError = error;
    wait(10, vex::timeUnits::msec);
  }
  
  driveStop(true);
}
#endif