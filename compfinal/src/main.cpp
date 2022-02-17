/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Sohum Suthar                                              */
/*    Created:      Oct 10 2021                                               */
/*    Description:  Competition Program 2360S                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <vex_competition.h>
#include <driver.h>

using namespace vex;
competition Competition;

void pre_auton(void) { //preauton for folding mechanisms (not in use)
  vexcodeInit();
}

void move(double revs, double power){
  MotorLB.setVelocity(power, velocityUnits::pct);
  MotorLF.setVelocity(power, velocityUnits::pct);
  MotorRB.setVelocity(power, velocityUnits::pct);
  MotorRF.setVelocity(power, velocityUnits::pct);

  MotorLB.spinFor(revs, rotationUnits::rev, false);
  MotorLF.spinFor(revs, rotationUnits::rev, false);
  MotorRB.spinFor(revs, rotationUnits::rev, false);
  MotorRF.spinFor(revs, rotationUnits::rev, true);

}
void autonomous(void) { //auton
/*MotorLB.spin(directionType::fwd, 20, velocityUnits::pct);
MotorRB.spin(directionType::fwd, 20, velocityUnits::pct);
MotorLF.spin(directionType::fwd, 20, velocityUnits::pct);
MotorRF.spin(directionType::fwd, 20, velocityUnits::pct);
vexDelay(2000);
MotorLB.stop(hold);
MotorRB.stop(hold);
MotorRF.stop(hold);
MotorLF.stop(hold); */
move(3.9, 80);
move(0.35, 20);
Claw.spinFor(-1.5, rotationUnits::rev, false);
vexDelay(1800);
Claw.stop();
move(-3, 50);
//0.15 50hold

}

void usercontrol(void) {
  Controller1.ButtonUp.pressed(nitroboost); //assigning all switchable modes
  Controller1.ButtonDown.pressed(snailmode);
  //Controller1.ButtonLeft.pressed(toggleonoff);
  timer Timer = timer(); //start timer for reminding the driver of time
  Timer.reset();
  float accel = 1;
  motor allMotors[] = {MotorLB, MotorLF, MotorRB, MotorRF, ArmL, ArmR, ArmB, Claw}; //check temps for overheating to fix an issue
  int i = 0;
  for (motor myMotor : allMotors) {
    if (myMotor.installed())
      continue;
    if (myMotor.temperature(fahrenheit) > 90) {
      Controller1.Screen.print("Motor %d overheating!", i);
      Controller1.rumble(".....");
      break;
    }
    i++;
  }
  while (1) { //drivercontrol functions
    if (Controller1.ButtonR1.pressing()) {
      ArmL.spin(directionType::fwd, 100 * maxSpeedPct, velocityUnits::pct);
      ArmR.spin(directionType::fwd, 100 * maxSpeedPct, velocityUnits::pct);
    } else if (Controller1.ButtonR2.pressing()) {
      ArmL.spin(directionType::rev, 100 * maxSpeedPct, velocityUnits::pct);
      ArmR.spin(directionType::rev, 100 * maxSpeedPct, velocityUnits::pct);
    } else if (Controller1.ButtonL2.pressing()) {
      Claw.spin(directionType::rev, 100 * maxSpeedPct, velocityUnits::pct);
    } else if (Controller1.ButtonL1.pressing()) {
      Claw.spin(directionType::fwd, 100 * maxSpeedPct, velocityUnits::pct);
    } else if (Controller1.ButtonX.pressing()) {
      ArmB.spin(directionType::rev, 100 * maxSpeedPct * accel, velocityUnits::pct);
      //accel -= 0.1;
    } else if (Controller1.ButtonB.pressing()) {
      ArmB.spin(directionType::fwd, 100 * maxSpeedPct, velocityUnits::pct);
    } else {
      ArmL.stop(hold);
      accel = 1;
      ArmR.stop(hold);
      ArmB.stop(hold);
      Claw.stop(hold);
    }

    // Drivetrain code for joysticks
    MotorLB.spin(directionType::fwd,Controller1.Axis3.position(percentUnits::pct) * maxSpeedPct,velocityUnits::pct);
    MotorRB.spin(directionType::fwd,Controller1.Axis2.position(percentUnits::pct) * maxSpeedPct,velocityUnits::pct);
    MotorLF.spin(directionType::fwd,Controller1.Axis3.position(percentUnits::pct) * maxSpeedPct,velocityUnits::pct);
    MotorRF.spin(directionType::fwd,Controller1.Axis2.position(percentUnits::pct) * maxSpeedPct, velocityUnits::pct);

    // Auxillerary systems for reminding the driver with vibrations
    if (Brain.Battery.capacity() < 15) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(0, 0);
      Controller1.Screen.print("CHARGE BATTERY:");
      Controller1.Screen.newLine();
      Controller1.Screen.print(Brain.Battery.capacity());
      Controller1.Screen.print("%");
      Controller1.rumble("---");
    }
    int timeRemaining = 105 - (int)Timer.value(); // 1 min 45 sec for the match 

    if ((timeRemaining <= 5 && timeRemaining >= 0) || timeRemaining == 10) { //vibrate the controller
      Controller1.rumble("-");
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(0, 0);
      Controller1.Screen.print("%d sec left", timeRemaining);
    }

    task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}
int main() {
  //set up all callbacks
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
