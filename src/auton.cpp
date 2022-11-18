#include "auton.h"

void long_roller_testing() {
  double WDesired = -M_PI/72;
  double WVelocity;
  PIDController WPID = PIDController(1, 0, 0);

  base.move_velocity(0, 200, 0);
  motor_move(INTAKE, -127);
  delay(1000);
  motor_brake(INTAKE);
  base.brake();

  delay(1000);

  while(!(mathy_within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  flywheel.set_mode(flywheelMode::Auton);

  flywheel.spin_velocity(2050);

  delay(3000);

  motor_move(PUNCHER, 127);

  delay(500);

  motor_brake(PUNCHER);

  delay(3000);

  motor_move(PUNCHER, 127);

  delay(500);

  flywheel.spin_velocity(0);

  motor_brake(PUNCHER);
}

void testing() {
  flywheel.set_mode(flywheelMode::Auton);

  flywheel.spin_velocity(2050);

  delay(5000);

  motor_move(PUNCHER, 127);

  delay(500);

  motor_brake(PUNCHER);

  delay(3000);

  motor_move(PUNCHER, 127);

  delay(500);

  // flywheel.spin_velocity(0);

  motor_brake(PUNCHER);
}

void testing_pid_movement() {
    PIDController TranslationPID = PIDController(1, 0, 0);

    double TranslationVelocity;
    double XDesired = 1 * 24, YDesired = 1 * 24;
    double TranslationalError;

    while(!(mathy_within(odometry.getPosition().y, YDesired, 0.5) && mathy_within(odometry.getPosition().x, XDesired, 0.5))) {
        Vector difference = Vector();
        difference.x = XDesired - odometry.getPosition().x;
        difference.y = YDesired - odometry.getPosition().y;
        //rotate the vector by heading to make turn resistant
        //disable if breaks
        difference.rotate(odometry.getHeading());

        double TranslationalPower = TranslationPID.calculate(difference.magnitude());
        //double RotationalPower = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));
        printf("angle: %f, power: %f\n", difference.angle(), TranslationalPower);
        base.move_vector(difference.angle(), TranslationalPower, 0, false);
        pros::delay(20);
    }

  base.brake();
}