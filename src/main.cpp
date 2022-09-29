#include "main.h"
#include "chassis.h"
#include "flywheel.h"
#include "odometry.h"
#include "pros/misc.h"
#include "pros/motors.h"

using namespace pros::c;



const int32_t MOTOR_MAX_VOLTAGE = 127;
const int32_t MOTOR_MIN_VOLTAGE = -127;

const uint8_t FRONTRIGHT = 1;
const uint8_t BACKRIGHT = 2;
const uint8_t BACKLEFT = 3;
const uint8_t FRONTLEFT = 4;

const uint8_t FLYWHEELA = 5;
const uint8_t FLYWHEELB = 6;

const uint8_t INTAKE = 11;

const uint8_t PISTON = 8;

const uint8_t ADI_ENCODER_RIGHT_TOP = 1;
const uint8_t ADI_ENCODER_RIGHT_BOTTOM = 2;

const uint8_t ADI_ENCODER_LEFT_TOP = 3;
const uint8_t ADI_ENCODER_LEFT_BOTTOM = 4;

const uint8_t ADI_ENCODER_STRAFE_TOP = 5;
const uint8_t ADI_ENCODER_STRAFE_BOTTOM = 6;

const pros::controller_id_e_t MASTER_CONTROLLER = pros::controller_id_e_t::E_CONTROLLER_MASTER;

adi_encoder_t right_encoder;
adi_encoder_t left_encoder;
adi_encoder_t strafe_encoder;

// declare subsystems here
chassis_t base;
flywheel discShooter;


// odometry stuff
tracking_params_t params;
pose_t pose;

static int32_t remap(int32_t value, int32_t from1, int32_t to1, int32_t from2, int32_t to2) {
  return (int)((double)(value - from1) / (to1 - from1) * (to2 - from2) + from2);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  printf("Initializing");

  // Initialize motors
  base.frontLeftMotor = FRONTLEFT;
  base.frontRightMotor = FRONTRIGHT;
  base.backLeftMotor = BACKLEFT;
  base.backRightMotor = BACKRIGHT;

  discShooter.motorA = FLYWHEELA;
  discShooter.motorB = FLYWHEELB;

  // Set motor gearing
  base_set_gearing(base, pros::E_MOTOR_GEARSET_18);
  flywheel_set_gearing(discShooter, pros::E_MOTOR_GEARSET_06);
  
  // Set specific motor directions
  motor_set_reversed(base.frontLeftMotor, true);
  motor_set_reversed(base.backLeftMotor, true);
  motor_set_reversed(discShooter.motorA, true);
  motor_set_reversed(INTAKE, false);

  motor_set_brake_mode(base.frontLeftMotor, MOTOR_BRAKE_COAST);
  motor_set_brake_mode(base.frontRightMotor, MOTOR_BRAKE_COAST);
  motor_set_brake_mode(base.backLeftMotor, MOTOR_BRAKE_COAST);
  motor_set_brake_mode(base.backRightMotor, MOTOR_BRAKE_COAST);

  motor_set_brake_mode(discShooter.motorA, MOTOR_BRAKE_COAST);
  motor_set_brake_mode(discShooter.motorB, MOTOR_BRAKE_COAST);

  left_encoder = adi_encoder_init(ADI_ENCODER_LEFT_TOP, ADI_ENCODER_LEFT_BOTTOM, false);
  right_encoder = adi_encoder_init(ADI_ENCODER_RIGHT_TOP, ADI_ENCODER_RIGHT_BOTTOM, false);
  strafe_encoder = adi_encoder_init(ADI_ENCODER_STRAFE_TOP, ADI_ENCODER_STRAFE_BOTTOM, false);

  adi_pin_mode(PISTON,OUTPUT);

  adi_encoder_reset(left_encoder);
  adi_encoder_reset(right_encoder);
  adi_encoder_reset(strafe_encoder);

  if (usd_is_installed()) {
    printf("SD card installed :(\n");
  } else {
    printf("SD card installed :)\n");
  }

  params.left_encoder = left_encoder;
  params.right_encoder = right_encoder;
  params.strafe_encoder = strafe_encoder;

  pose.x = 0;
  pose.y = 0;
  pose.w = 0;

  params.pose = pose;

  pros::task_t odometry = task_create(odometry_track, (void*)(&params), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Jeffrey Hamil The 3rd");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() { printf("disabled"); }

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() { printf("comp init"); }

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
  flywheel_spin(discShooter, 127);

  adi_digital_write(PISTON, true);

  delay(2000);

  adi_digital_write(PISTON, false);
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
  printf("opcontrol");

  while (1) {
/*
    printf("Right Encoder: %d, Left Encoder: %d, Strafe Encoder: %d\n", 
    ext_adi_encoder_get(right_encoder), 
    ext_adi_encoder_get(left_encoder), 
    ext_adi_encoder_get(strafe_encoder));
*/
    //printf("%d\n", controller_get_digital(MASTER_CONTROLLER, DIGITAL_B));
  
    if(controller_get_digital(MASTER_CONTROLLER, DIGITAL_B) == 1) {
      adi_digital_write(PISTON, true);
    } else {
      adi_digital_write(PISTON, false);
    }

    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_L1)) {
      flywheel_spin(discShooter, 127);
    }
    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_L2)) {
      flywheel_spin(discShooter, 0);
    }

    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_R1)) {
      motor_move(INTAKE, 127);
    }
    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_R2)) {
      motor_move(INTAKE, 0);
    }
    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_A)) {
      motor_move(INTAKE, -127);
    }

    double desired_controller_x = controller_get_analog(MASTER_CONTROLLER, ANALOG_LEFT_X);
    double desired_controller_y = controller_get_analog(MASTER_CONTROLLER, ANALOG_LEFT_Y);
    double desired_controller_w = controller_get_analog(MASTER_CONTROLLER, ANALOG_RIGHT_X);

    double controller_x;
    double controller_y;
    double controller_w;

    double translation_gradient = 0.1;
    double rotation_gradient = 0.2;

    //might cause issues with exact values due to imprecise doubles
    controller_x += (desired_controller_x - controller_x) * translation_gradient;
    controller_y += (desired_controller_y - controller_y) * translation_gradient;
    controller_w += (desired_controller_w - controller_w) * rotation_gradient;

    if(desired_controller_x < 0.000001 && desired_controller_x > -0.000001) {
      controller_x = 0;
    }
    if(desired_controller_y < 0.000001 && desired_controller_y > -0.000001) {
      controller_y = 0;
    }

    double field_based_controller_x = controller_x * cos(pose.w) - controller_y * sin(pose.w);
    double field_based_controller_y = controller_x * sin(pose.w) + controller_y * cos(pose.w);

    base_move_velocity(base,
    remap(field_based_controller_x, -127, 127, -200, 200),
    remap(field_based_controller_y, -127, 127, -200, 200),
    remap(controller_w, -127, 127, -200, 200));

    

    //flywheel_spin(discShooter, controller_get_analog(MASTER_CONTROLLER, ANALOG_RIGHT_Y));
    /*
    // Logging logic
    if (usd_is_installed()) {
      FILE *file = fopen("/usd/file.csv", "a");
      if (file != NULL) {
        const int firstSpeed = motor_get_actual_velocity(FLYWHEELA);
        const int secondSpeed = motor_get_actual_velocity(FLYWHEELB);
        const double firstTorque = motor_get_torque(FLYWHEELA);
        const double secondTorque = motor_get_torque(FLYWHEELB);
        const double firstTemperature = motor_get_temperature(FLYWHEELA);
        const double secondTemperature = motor_get_temperature(FLYWHEELB);
        fprintf(file, "%d, %f, %f, %d, %f, %f\n", firstSpeed, firstTorque,
                firstTemperature, secondSpeed, secondTorque, secondTemperature);
        fclose(file);

        printf("%d, %f, %f, %d, %f, %f\n", firstSpeed, firstTorque,
               firstTemperature, secondSpeed, secondTorque, secondTemperature);
      } else {
        printf("File not found\n");
      }
    }
    */
    delay(20);
  }
}
