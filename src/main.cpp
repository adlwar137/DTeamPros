#include "main.h"
#include "config.h"
#include "chassis.h"
#include "flywheel.h"
#include "mathy.h"
#include "PIDController.h"
#include "auton.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "constants.h"

using namespace pros::c;

// declare subsystems here
bool isForward = true;

//no idea if works or not
static bool within(double x, double y, double tolerance) {
  return fabs(y - x) < tolerance;
}

void flywheel_run(void* param) {
  while(1) {
    flywheel.update();
    pros::delay(20);
  }
}

void odometry_run(void* param) {
  while(1) {
    odometry.updatePosition(mathy_angle_wrap(mathy_to_radians<double>(inertial.get_heading())));
    pros::delay(20);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  printf("Initializing");

  inertial.reset();
  
  while(inertial.is_calibrating()) {
    delay(200);
  }

  //delay(4000);

  // Initialize motors

  // Set motor gearing
  motor_set_brake_mode(PUNCHER, MOTOR_BRAKE_BRAKE);
  
  // Set specific motor directions
  motor_set_reversed(INTAKE, false);
  motor_set_reversed(PUNCHER, true);

  // Set brake modes
  base.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

  flywheel.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);

  gps_initialize_full(GPS_LEFT, 0, 0, 0, 0, 0);
  gps_initialize_full(GPS_RIGHT, 0, 0, 180, 0, 0);

  // Reset encoder tick positions
  left_encoder.reset();
  right_encoder.reset();
  strafe_encoder.reset();

  //create the odometry task to run in the background
  //That sweet sweet absolute position
  pros::Task odometry_task = pros::Task(odometry_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sir William Rowan Hamilton");

  pros::Task fly_wheel_pid = pros::Task(flywheel_run, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheel burr");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  printf("disabled");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() { 
  printf("comp init"); 
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {



// long one roller start

  double WDesired = -M_PI/72;
  double WVelocity;
  PIDController WPID = PIDController(1, 0, 0);

  base.move_velocity(0, 200, 0);
  motor_move(INTAKE, -127);
  delay(500);
  motor_brake(INTAKE);
  base.brake();

  delay(1000);

  while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  flywheel.set_flywheel_mode(flywheelMode::Auton);

  flywheel.spin_velocity(410);

  delay(3000);

  motor_move(PUNCHER, 127);

  delay(500);

  motor_brake(PUNCHER);

  delay(3000);

  motor_move(PUNCHER, 127);

  delay(500);

  flywheel.spin_velocity(0);

  motor_brake(PUNCHER);


// long one roller end

/*
  //short one roller start
  base.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);

  double TranslationalVelocity, WVelocity;
  double XDesired = -1 * 24, YDesired = 0, WDesired = 0;
  double TranslationalError, WError;

  PIDController TranslationPID = PIDController(0.05, 0, 0);
  PIDController WPID = PIDController(1, 0, 0);

  while(!(within(odometry.getPosition().y, YDesired, 0.5) && within(odometry.getPosition().x, XDesired, 0.5))) {
    Vector difference = Vector();
    difference.x = XDesired - odometry.getPosition().x;
    difference.y = YDesired - odometry.getPosition().y;
    //rotate the vector by heading to make turn resistant
    //disable if breaks
    //difference.rotate(odometry.getHeading());

    double TranslationalPower = TranslationPID.calculate(difference.magnitude());
    double RotationalPower = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));
    printf("angle: %f, power: %f\n", difference.angle(), TranslationalPower);
    base.move_vector(difference.angle(), TranslationalPower, 0, false);
    pros::delay(20);
  }

  base.brake();

   while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  base.move_velocity(0, 100, 0);
  motor_move(INTAKE, -127);
  delay(1500);
  motor_brake(INTAKE);
  base.brake();

  //short one roller end
*/

/*
  //long one roller start
  base.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);

  double TranslationalVelocity, WVelocity;
  double XDesired = 1 * 24, YDesired = 0, WDesired = 0;
  double TranslationalError, WError;

  PIDController TranslationPID = PIDController(0.05, 0, 0);
  PIDController WPID = PIDController(1, 0, 0);

  while(!(within(odometry.getPosition().y, YDesired, 0.5) && within(odometry.getPosition().x, XDesired, 0.5))) {
    Vector difference = Vector();
    difference.x = XDesired - odometry.getPosition().x;
    difference.y = YDesired - odometry.getPosition().y;
    //rotate the vector by heading to make turn resistant
    //disable if breaks
    //difference.rotate(odometry.getHeading());

    double TranslationalPower = TranslationPID.calculate(difference.magnitude());
    double RotationalPower = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));
    printf("angle: %f, power: %f\n", difference.angle(), TranslationalPower);
    base.move_vector(difference.angle(), TranslationalPower, 0, false);
    pros::delay(20);
  }

  base.brake();

   while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  base.move_velocity(0, 100, 0);
  motor_move(INTAKE, -127);
  delay(1500);
  motor_brake(INTAKE);
  base.brake();

  //long one roller end
*/

/*
  //two roller start
  base.set_brake_mode(MOTOR_BRAKE_BRAKE);

  base.move_velocity(0, 200, 0);
  motor_move(INTAKE, -127);
  delay(500);
  motor_brake(INTAKE);
  base.brake();

  double TranslationalVelocity, WVelocity;
  double XDesired = -24*3.5, YDesired = -24*3.5, WDesired = -M_PI/2;
  double TranslationalError, WError;

  PIDController TranslationPID = PIDController(0.05, 0, 0);
  PIDController WPID = PIDController(1, 0, 0);

  while(!(within(odometry.getPosition().y, YDesired, 0.5) && within(odometry.getPosition().x, XDesired, 0.5))) {
    Vector difference = Vector();
    difference.x = XDesired - odometry.getPosition().x;
    difference.y = YDesired - odometry.getPosition().y;
    //rotate the vector by heading to make turn resistant
    //disable if breaks
    //difference.rotate(odometry.getHeading());

    double TranslationalPower = TranslationPID.calculate(difference.magnitude());
    double RotationalPower = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));
    printf("angle: %f, power: %f\n", difference.angle(), TranslationalPower);
    base.move_vector(difference.angle(), TranslationalPower, 0, false);
    pros::delay(20);
  }

  while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  delay(1000);

  base.move_vector(0.75* M_PI, 1, 0, false);
  motor_move(INTAKE, -127);
  delay(1000);
  motor_brake(INTAKE);
  base.brake();
  //two roller end
*/


  //skills start
/*
  base.set_brake_mode(MOTOR_BRAKE_BRAKE);

  base.move_velocity(0, 200, 0);
  pros::delay(500);
  base.brake();
  motor_move_relative(INTAKE, 1350, -255);
  motor_brake(INTAKE);

  double WDesired = -M_PI/72;
  double WVelocity;
  PIDController WPID = PIDController(1, 0, 0);

  delay(1000);

  while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  flywheel.set_flywheel_mode(flywheelMode::Auton);

  flywheel.spin_velocity(410);

  delay(3000);

  motor_move(PUNCHER, 127);

  delay(500);

  motor_brake(PUNCHER);

  delay(3000);

  motor_move(PUNCHER, 127);

  delay(500);

  flywheel.spin_velocity(0);

  motor_brake(PUNCHER);

  double XDesired = 12, YDesired = -24;
  WDesired = M_PI/2;

  PIDController TranslationPID = PIDController(0.04, 0, 0);
  WPID = PIDController(1, 0, 0);

  while(!(within(odometry.getPosition().y, YDesired, 0.5) && within(odometry.getPosition().x, XDesired, 0.5))) {
    Vector difference = Vector();
    difference.x = XDesired - odometry.getPosition().x;
    difference.y = YDesired - odometry.getPosition().y;
    //rotate the vector by heading to make turn resistant
    //disable if breaks
    //difference.rotate(odometry.getHeading());

    double TranslationalPower = TranslationPID.calculate(difference.magnitude());
    double RotationalPower = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));
    printf("angle: %f, power: %f\n", difference.angle(), TranslationalPower);
    base.move_vector(difference.angle(), TranslationalPower, 0, false);
    pros::delay(20);
  }

  while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  pros::delay(500);

  base.move_velocity(0, 200, 0);
  pros::delay(500);
  base.brake();
  motor_move_relative(INTAKE, 1350, -255);
  motor_brake(INTAKE);

  XDesired = -4*24;
  YDesired = -3.8*24;
  WDesired = 0;
  
  pros::delay(1000);

  while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();
  pros::delay(1000);

  while(!(within(odometry.getPosition().y, YDesired, 0.5) && within(odometry.getPosition().x, XDesired, 0.5))) {
    Vector difference = Vector();
    difference.x = XDesired - odometry.getPosition().x;
    difference.y = YDesired - odometry.getPosition().y;
    //rotate the vector by heading to make turn resistant
    //disable if breaks
    //difference.rotate(odometry.getHeading());

    double TranslationalPower = TranslationPID.calculate(difference.magnitude());
    //double RotationalPower = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));
    printf("angle: %f, power: %f\n", difference.angle(), TranslationalPower);
    base.move_vector(difference.angle() + odometry.getHeading(), TranslationalPower, 0, false);
    pros::delay(20);
  }

  base.brake();
  pros::delay(500);

  WDesired = -M_PI/2;

  while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }
  
  base.brake();

  base.move_velocity(0, 25, 0);
  pros::delay(3000);
  base.brake();
  motor_move_relative(INTAKE, 1350, -255);
  motor_brake(INTAKE);

  WDesired = 0;

    while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  XDesired = -3.25*24;
  YDesired = -3.25*24;

  while(!(within(odometry.getPosition().y, YDesired, 0.5) && within(odometry.getPosition().x, XDesired, 0.5))) {
    Vector difference = Vector();
    difference.x = XDesired - odometry.getPosition().x;
    difference.y = YDesired - odometry.getPosition().y;
    //rotate the vector by heading to make turn resistant
    //disable if breaks
    //difference.rotate(odometry.getHeading());

    double TranslationalPower = TranslationPID.calculate(difference.magnitude());
    //double RotationalPower = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));
    printf("angle: %f, power: %f\n", difference.angle(), TranslationalPower);
    base.move_vector(difference.angle() + odometry.getHeading(), TranslationalPower, 0, false);
    pros::delay(20);
  }

  base.brake();
  pros::delay(500);

  WDesired = M_PI;

  while(!(within(odometry.getHeading(), WDesired, 0.05))) {

    WVelocity = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));

    base.move_vector(0, 0, WVelocity, false);

    pros::delay(20);
  }

  base.brake();

  base.move_velocity(0, 50, 0);
  pros::delay(3000);
  base.brake();
  motor_move_relative(INTAKE, 1350, -255);
  motor_brake(INTAKE);

  piston.set_value(true);

  XDesired = 3.25*24;
  YDesired = 3.25*24;

  while(!(within(odometry.getPosition().y, YDesired, 0.5) && within(odometry.getPosition().x, XDesired, 0.5))) {
    Vector difference = Vector();
    difference.x = XDesired - odometry.getPosition().x;
    difference.y = YDesired - odometry.getPosition().y;
    //rotate the vector by heading to make turn resistant
    //disable if breaks
    //difference.rotate(odometry.getHeading());

    double TranslationalPower = TranslationPID.calculate(difference.magnitude());
    //double RotationalPower = WPID.calculate(mathy_angle_wrap(WDesired - odometry.getHeading()));
    printf("angle: %f, power: %f\n", difference.angle(), TranslationalPower);
    base.move_vector(difference.angle() + odometry.getHeading(), TranslationalPower, 0, false);
    pros::delay(20);
  }
  //skills end
*/
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  bool pistonState = true;

  printf("opcontrol");

  int opcontrol_time = pros::millis();

  left_encoder.reset();
  right_encoder.reset();
  strafe_encoder.reset();

  odometry.reset();

  PIDController WPID = PIDController(64, 0, 0);
  PIDController rollercontroller = PIDController(0.01, 0, 0);

  bool intakeLimitState = false;
  bool lastIntakeLimitState = false;

  flywheel.set_flywheel_mode(flywheelMode::Driver);


  while (1) {

    printf("range: %d\n", rangeFinder.get_value());
    if(rangeFinder.get_value() < 100) {
      rollerLED.set_value(0);
    } else {
      rollerLED.set_value(1);
    }


/*
      int area = rollerSensor.get_by_sig(0, 1).width * rollerSensor.get_by_sig(0, 1).height;
      
      //rollerSensor.get_by_sig(0, 1);

      int desired = 30000;
      int error = desired - area;
      
      printf("area: %f, error: %d\n", area, error);

      motor_move(INTAKE, -rollercontroller.calculate(error));
*/

      if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_R1)) {
        motor_move(INTAKE, 127);
      }
      if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_R2)) {
        motor_move(INTAKE, 0);
      }
      if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_A)) {
        motor_move(INTAKE, -127);
      }
  
    if(intakeLimitState == false && lastIntakeLimitState == true) {
      motor_brake(INTAKE);
    }
    
    lastIntakeLimitState = intakeLimitState;
    
    piston.set_value(pistonState);

    if(controller_get_digital(MASTER_CONTROLLER, DIGITAL_B) == 1) {
      motor_move(PUNCHER, 127);
    } else {
      motor_brake(PUNCHER);
    }

    if((pros::millis() - opcontrol_time) > 95000) {
        if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_LEFT)) {
          pistonState = !pistonState;
        }
    }


    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_DOWN) == 1) {
      isForward = !isForward;
    }

    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_L1)) {
      flywheel.spin(127);
    }
    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_L2)) {
      flywheel.brake();
    }

    //printf("flywheelASpeed: %f, flywheelATemp: %f, flywheelBSpeed: %f, flywheelBTemp: %f\n", motor_get_actual_velocity(FLYWHEELA), motor_get_temperature(FLYWHEELA), motor_get_actual_velocity(FLYWHEELB), motor_get_temperature(FLYWHEELB));
    int desired_controller_x = controller_get_analog(MASTER_CONTROLLER, ANALOG_LEFT_X);
    int desired_controller_y = controller_get_analog(MASTER_CONTROLLER, ANALOG_LEFT_Y);
    int desired_controller_w = controller_get_analog(MASTER_CONTROLLER, ANALOG_RIGHT_X);

    //flywheel.spin_velocity(((double)controller_get_analog(MASTER_CONTROLLER, ANALOG_RIGHT_Y) / 127) * 600);
    //printf("controller: %f\n",((double)controller_get_analog(MASTER_CONTROLLER, ANALOG_RIGHT_Y) / 127) * 200);
    double field_based_controller_x = desired_controller_x * cos(odometry.getHeading()) - desired_controller_y * sin(odometry.getHeading());
    double field_based_controller_y = desired_controller_x * sin(odometry.getHeading()) + desired_controller_y * cos(odometry.getHeading());

    //field centric
    //base.move_vector(atan2(field_based_controller_y, field_based_controller_x), hypot(field_based_controller_x, field_based_controller_y) / (double)127, desired_controller_w/(double)127, false);

    if(isForward) {
      base.move_vector(atan2(desired_controller_y, desired_controller_x), hypot(desired_controller_x, desired_controller_y) / (double)127, desired_controller_w/(double)127, false);
    } else {
      base.move_vector(atan2(desired_controller_y, desired_controller_x), hypot(desired_controller_x, desired_controller_y) / (double)127, desired_controller_w/(double)127/2, true);
    }

/*
    if(isForward) {
      base.move_velocity(
      mathy_remap(desired_controller_x, -127, 127, -200, 200),
      mathy_remap(desired_controller_y, -127, 127, -200, 200),
      mathy_remap(desired_controller_w, -127, 127, -200, 200));
    } else {
      

      double redGoal = atan2(48 - odometry.getPosition().x, 48 - odometry.getPosition().y);
      double blueGoal = atan2(-48, -48);

      double WDesired = mathy_angle_wrap(redGoal);

      //printf("heading: %f\n", WDesired);
      printf("atan test: %f\n", mathy_to_degrees(atan2(48 - odometry.getPosition().x,48 - odometry.getPosition().y) - odometry.getStatus().w));

      double WError = mathy_angle_wrap(atan2(48 - odometry.getPosition().x,48 - odometry.getPosition().y) - odometry.getStatus().w);

      double WVelocity = WPID.calculate(WError);

      WVelocity = mathy_clamp(WVelocity, -127, 127);


      base.move_velocity(
      mathy_remap(-desired_controller_x, -127, 127, -200, 200),
      mathy_remap(-desired_controller_y, -127, 127, -200, 200),
      mathy_remap(WVelocity, -127, 127, -200, 200));
    }
*/
/*
    base_move_velocity(base,
      mathy_remap(field_based_controller_x, -127, 127, -200, 200),
      mathy_remap(field_based_controller_y, -127, 127, -200, 200),
      mathy_remap(desired_controller_w, -127, 127, -200, 200));
*/

    //printf("x: %f, Y: %f, W: %f\n", odometry.getStatus().x, odometry.getStatus().y, odometry.getStatus().heading);
    //printf("left: %d, right: %d, strafe: %d\n", adi_encoder_get(left_encoder), adi_encoder_get(right_encoder), adi_encoder_get(strafe_encoder));

    delay(20);
  }
}
