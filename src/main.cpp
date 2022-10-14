#include "main.h"
#include "constants.h"
#include "chassis.h"
#include "flywheel.h"
#include "odometry.h"
#include "mathy.h"
#include "PIDController.h"
#include "auton.h"
#include "locationSensor.h"
#include "pros/misc.h"
#include "pros/motors.h"

using namespace pros::c;

adi_encoder_t right_encoder;
adi_encoder_t left_encoder;
adi_encoder_t strafe_encoder;

chassis_t base;
flywheel discShooter;

tracking_params_t params;
vector3d pose;

// declare subsystems here
bool isForward = true;

//no idea if works or not
static bool within(double x, double y, double tolerance) {
  return fabs(y - x) < tolerance;
}

//roller swap
void autonomous_roller_swap() {
  base_move_velocity(base, 0, 200, 0);
  motor_move(INTAKE, -127);
  delay(500);
  motor_brake(INTAKE);
  base_brake(base);
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
  motor_set_brake_mode(PUNCHER, MOTOR_BRAKE_BRAKE);
  
  // Set specific motor directions
  motor_set_reversed(base.backRightMotor, true);
  motor_set_reversed(base.frontRightMotor, true);
  motor_set_reversed(discShooter.motorA, true);
  motor_set_reversed(INTAKE, false);
  motor_set_reversed(PUNCHER, true);

  // Set brake modes
  base_set_brake_mode(base, MOTOR_BRAKE_COAST);

  motor_set_brake_mode(discShooter.motorA, MOTOR_BRAKE_BRAKE);
  motor_set_brake_mode(discShooter.motorB, MOTOR_BRAKE_BRAKE);

  // Initialize encoders
  left_encoder = adi_encoder_init(ADI_ENCODER_LEFT_TOP, ADI_ENCODER_LEFT_BOTTOM, false);
  right_encoder = adi_encoder_init(ADI_ENCODER_RIGHT_TOP, ADI_ENCODER_RIGHT_BOTTOM, false);
  strafe_encoder = adi_encoder_init(ADI_ENCODER_STRAFE_TOP, ADI_ENCODER_STRAFE_BOTTOM, false);

  gps_initialize_full(GPS_LEFT, 0, 0, 0, 0, 0);
  gps_initialize_full(GPS_RIGHT, 0, 0, 180, 0, 0);

  //set the piston adi port to output
  adi_pin_mode(LED,OUTPUT);

  // Reset encoder tick positions
  adi_encoder_reset(left_encoder);
  adi_encoder_reset(right_encoder);
  adi_encoder_reset(strafe_encoder);

  //sd card inserted readout
  if (usd_is_installed()) {
    printf("SD card installed :(\n");
  } else {
    printf("SD card installed :)\n");
  }

  //set the parameters of the odometry task
  params.left_encoder = left_encoder;
  params.right_encoder = right_encoder;
  params.strafe_encoder = strafe_encoder;
  params.pose = &pose;

  //reset absolute pose
  pose.x = 0;
  pose.y = 0;
  pose.w = 0;

  //create the odometry task to run in the background
  //That sweet sweet absolute position
  pros::task_t odometry = task_create(odometry_track, (void*)(&params), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sir William Rowan Hamilton");
/*
  lv_obj_t * scr1;
  scr1 = lv_obj_create(NULL, NULL);

  lv_obj_t * background;
  background = lv_obj_create(NULL, NULL);
  lv_obj_set_size(background, 480, 272);
  lv_obj_set_style(background, &lv_style_scr);
  lv_obj_align(background, NULL, LV_ALIGN_CENTER, 0, 0);
  

  lv_obj_t * scr2;
  scr2 = lv_obj_create(NULL, NULL);
  lv_obj_set_size(scr2, 480, 272);
  lv_obj_set_style(scr2, &lv_style_pretty);
  lv_obj_align(scr2, NULL, LV_ALIGN_CENTER, 0, 0);

  lv_scr_load(scr1);

  lv_obj_t * obj1;
  obj1 = lv_obj_create(lv_scr_act(), NULL);
  lv_obj_set_size(obj1, 100, 100);
  lv_obj_set_style(obj1, &lv_style_plain_color);
  lv_obj_align(obj1, NULL, LV_ALIGN_CENTER, 0, -60);

  lv_obj_t * robLabel;
  robLabel = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_long_mode(robLabel, LV_LABEL_LONG_BREAK);
  lv_label_set_recolor(robLabel, true);
  lv_label_set_align(robLabel, LV_LABEL_ALIGN_CENTER);
  lv_label_set_text(robLabel, "#000000 RobOS");

  lv_obj_set_width(robLabel, 100);
  lv_obj_align(robLabel, NULL, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t * bar1 = lv_bar_create(lv_scr_act(), NULL);
  lv_obj_set_size(bar1, 200, 30);
  lv_obj_align(bar1, NULL, LV_ALIGN_CENTER, 0, 30);
  lv_bar_set_style(bar1, LV_BAR_STYLE_INDIC, &lv_style_pretty_color);
  lv_bar_set_value_anim(bar1, 100, 2000);
  */
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

  auton_roller_swap();

  base_move_velocity(base, -200, -200, 0);

  delay(3500);

  base_brake(base);

   //score first roller
   /*
  base_move_velocity(base, 0, 200, 0);
  motor_move(INTAKE, -127);
  delay(500);
  motor_brake(INTAKE);
  base_brake(base);
  */
  /*
  base_set_brake_mode(base, MOTOR_BRAKE_BRAKE);

  double TranslationalVelocity, WVelocity;
  double XDesired = 0, YDesired = 0, WDesired = -M_PI/2;
  double TranslationalError, WError;

  PIDController_t TranslationPID = PIDController_create(8, 0, 0);
  PIDController_t WPID = PIDController_create(64, 0, 0);

  delay(2000);

 

  pose.x = 0;
  pose.y = 0;
  pose.w = 0;

 
  base_brake(base);

  double field_based_controller_x = -200 * cos(pose.w) - (-200) * sin(pose.w);
  double field_based_controller_y = -200 * sin(pose.w) + (-200) * cos(pose.w);

  base_move_velocity(base, field_based_controller_x, field_based_controller_y, 0);

  delay(3500);

  base_brake(base);

  delay(300);

   while(!(within(pose.w, WDesired, 0.05))) {


    base_move_velocity(base, 0, 0, WVelocity);

    delay(20);
  }

  base_brake(base);

  delay(300);

  base_move_velocity(base, 0, 200, 0);
  motor_move(INTAKE, -127);
  delay(2000);
  motor_brake(INTAKE);
  base_brake(base);

  base_set_brake_mode(base, MOTOR_BRAKE_COAST);



*/


  //move to second roller

  //move in between tiles to the left
/*
  XDesired = (-24 * 3) - 12;
  while(!(within(pose.x, XDesired, 0.5) && within(pose.y, YDesired, 0.5) && within(pose.w, WDesired, 0.1))) {
    
    TranslationalError = mathy_distance_between_points(XDesired, pose.x, YDesired, pose.y);
    WError = mathy_angle_wrap(WDesired - pose.w); //wrap angle to signed smallest distance
    
    TranslationalVelocity = PIDController_calculate(TranslationPID, TranslationalError);
    WVelocity = PIDController_calculate(WPID, WError);

    //get the angle in radians of the remaining distance vector
    double distanceVectorAngle = atan2(YDesired - pose.y, XDesired - pose.x);

    //add the heading to rotate the vector to the global space
    distanceVectorAngle += pose.w;

    motor_move_velocity(FRONTLEFT, TranslationalVelocity * cos(distanceVectorAngle - (M_PI/2)) + WVelocity);
    motor_move_velocity(BACKLEFT, TranslationalVelocity * cos((3*M_PI/4) - distanceVectorAngle) + WVelocity);
    motor_move_velocity(BACKRIGHT, TranslationalVelocity * cos(distanceVectorAngle - (M_PI/2)) - WVelocity);
    motor_move_velocity(FRONTRIGHT, TranslationalVelocity * cos((3*M_PI/4) - distanceVectorAngle) - WVelocity);
  
    delay(20);
  }
  base_brake(base);

  YDesired = 1.5 * 24;
  while(!(within(pose.x, XDesired, 0.5) && within(pose.y, YDesired, 0.5) && within(pose.w, WDesired, 0.1))) {
    
    TranslationalError = mathy_distance_between_points(XDesired, pose.x, YDesired, pose.y);
    WError = mathy_angle_wrap(WDesired - pose.w); //wrap angle to signed smallest distance
    
    TranslationalVelocity = PIDController_calculate(TranslationPID, TranslationalError);
    WVelocity = PIDController_calculate(WPID, WError);

    //get the angle in radians of the remaining distance vector
    double distanceVectorAngle = atan2(YDesired - pose.y, XDesired - pose.x);

    //add the heading to rotate the vector to the global space
    distanceVectorAngle += pose.w;

    motor_move_velocity(FRONTLEFT, TranslationalVelocity * cos(distanceVectorAngle - (M_PI/2)) + WVelocity);
    motor_move_velocity(BACKLEFT, TranslationalVelocity * cos((3*M_PI/4) - distanceVectorAngle) + WVelocity);
    motor_move_velocity(BACKRIGHT, TranslationalVelocity * cos(distanceVectorAngle - (M_PI/2)) - WVelocity);
    motor_move_velocity(FRONTRIGHT, TranslationalVelocity * cos((3*M_PI/4) - distanceVectorAngle) - WVelocity);
  
    delay(20);
  }
  base_brake(base);
*/
/*
  double TranslationalVelocity, WVelocity;
  double XDesired = 0, YDesired = 0, WDesired = 0;
  double TranslationalError, WError;

  PIDController_t TranslationPID = PIDController_create(8, 0, 0);
  PIDController_t WPID = PIDController_create(0, 0, 0);

  delay(2000);

  pose.x = 0;
  pose.y = 0;
  pose.w = 0;

  XDesired= 24*3.5;
  YDesired= -24*3.5;

  while(!(within(pose.x, XDesired, 0.5) && within(pose.y, YDesired, 0.5) && within(pose.w, WDesired, 0.1))) {
    
    TranslationalError = mathy_distance_between_points(XDesired, pose.x, YDesired, pose.y);
    WError = mathy_angle_wrap(WDesired - pose.w); //wrap angle to signed smallest distance
    
    TranslationalVelocity = PIDController_calculate(TranslationPID, TranslationalError);
    WVelocity = PIDController_calculate(WPID, WError);

    //get the angle in radians of the remaining distance vector
    double distanceVectorAngle = atan2(YDesired - pose.y, XDesired - pose.x);

    //add the heading to rotate the vector to the global space
    distanceVectorAngle += pose.w;

    if((TranslationalVelocity + WVelocity) > 200 || (TranslationalVelocity + WVelocity) < -200) {
      double multi = (double)200/(TranslationalVelocity+WVelocity);
      TranslationalVelocity = multi*TranslationalVelocity;
      WVelocity = multi*WVelocity;
    }

    motor_move_velocity(FRONTLEFT, TranslationalVelocity * cos(distanceVectorAngle - (M_PI/2)) + WVelocity);
    motor_move_velocity(BACKLEFT, TranslationalVelocity * cos((3*M_PI/4) - distanceVectorAngle) + WVelocity);
    motor_move_velocity(BACKRIGHT, TranslationalVelocity * cos(distanceVectorAngle - (M_PI/2)) - WVelocity);
    motor_move_velocity(FRONTRIGHT, TranslationalVelocity * cos((3*M_PI/4) - distanceVectorAngle) - WVelocity);
  
    delay(20);
  }
  base_brake(base);
  
  delay(2000);
*/
/*
  WDesired = M_PI/2;

  while(!(within(pose.x, XDesired, 0.5) && within(pose.y, YDesired, 0.5) && within(pose.w, WDesired, 0.1))) {
    
    TranslationalError = mathy_distance_between_points(XDesired, pose.x, YDesired, pose.y);
    WError = mathy_angle_wrap(WDesired - pose.w); //wrap angle to signed smallest distance
    
    TranslationalVelocity = PIDController_calculate(TranslationPID, TranslationalError);
    WVelocity = PIDController_calculate(WPID, WError);

    //get the angle in radians of the remaining distance vector
    double distanceVectorAngle = atan2(YDesired - pose.y, XDesired - pose.x);

    //add the heading to rotate the vector to the global space
    distanceVectorAngle += pose.w;

    if((TranslationalVelocity + WVelocity) > 200 || (TranslationalVelocity + WVelocity) < -200) {
      double multi = (double)200/(TranslationalVelocity+WVelocity);
      TranslationalVelocity = multi*TranslationalVelocity;
      WVelocity = multi*WVelocity;
    }

    motor_move_velocity(FRONTLEFT, TranslationalVelocity * cos(distanceVectorAngle - (M_PI/2)) + WVelocity);
    motor_move_velocity(BACKLEFT, TranslationalVelocity * cos((3*M_PI/4) - distanceVectorAngle) + WVelocity);
    motor_move_velocity(BACKRIGHT, TranslationalVelocity * cos(distanceVectorAngle - (M_PI/2)) - WVelocity);
    motor_move_velocity(FRONTRIGHT, TranslationalVelocity * cos((3*M_PI/4) - distanceVectorAngle) - WVelocity);
  
    delay(20);
  
  base_brake(base);
  }
*/

/*
  //score second roller
  base_move_velocity(base, 0, 200, 0);
  motor_move(INTAKE, -127);
  delay(500);
  motor_brake(INTAKE);
  base_brake(base);
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
  printf("opcontrol");

  PIDController_t WPID = PIDController_create(128, 0, 0);

  while (1) {
    printf("OdometryHeading: %f, LocationSensorHeading: %f\n", mathy_to_degrees(pose.w), 0);

    if(controller_get_digital(MASTER_CONTROLLER, DIGITAL_B) == 1) {
      motor_move(PUNCHER, 127);
    } else {
      motor_brake(PUNCHER);
    }

    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_DOWN) == 1) {
      isForward = !isForward;
    }

    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_L1)) {
      flywheel_spin(discShooter, 127);
    }
    if(controller_get_digital_new_press(MASTER_CONTROLLER, DIGITAL_L2)) {
      flywheel_spin(discShooter, 0);
    }

    //printf("flywheelASpeed: %f, flywheelATemp: %f, flywheelBSpeed: %f, flywheelBTemp: %f\n", motor_get_actual_velocity(FLYWHEELA), motor_get_temperature(FLYWHEELA), motor_get_actual_velocity(FLYWHEELB), motor_get_temperature(FLYWHEELB));

    if(((motor_get_actual_velocity(FLYWHEELA) + motor_get_actual_velocity(FLYWHEELB)) / 2) > 450 ) {
      adi_digital_write(LED, false);
      controller_rumble(MASTER_CONTROLLER, "-");
    } else {
      adi_digital_write(LED, true);
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

    double field_based_controller_x = desired_controller_x * cos(pose.w) - desired_controller_y * sin(pose.w);
    double field_based_controller_y = desired_controller_x * sin(pose.w) + desired_controller_y * cos(pose.w);

    if(isForward) {
      base_move_velocity(base,
      mathy_remap(desired_controller_x, -127, 127, -200, 200),
      mathy_remap(desired_controller_y, -127, 127, -200, 200),
      mathy_remap(desired_controller_w, -127, 127, -200, 200));
    } else {

      double redGoal = atan2(0 - locationSensor_get_pose().x, 0 - locationSensor_get_pose().y);
      double blueGoal = atan2(-48, -48);

      double WDesired = mathy_angle_wrap(redGoal);

      //printf("heading: %f\n", WDesired);
      printf("atan test: %f\n", mathy_to_degrees(atan2(48 - locationSensor_get_pose().x,48 - locationSensor_get_pose().y) - mathy_to_radians(gps_get_heading_raw(GPS_LEFT) + 90)));

      double WError = mathy_angle_wrap(atan2(48 - locationSensor_get_pose().x,48 - locationSensor_get_pose().y) - mathy_to_radians(gps_get_heading_raw(GPS_LEFT) + 90));

      double WVelocity = PIDController_calculate(WPID, WError);

      WVelocity = mathy_clamp(WVelocity, -127, 127);

      base_move_velocity(base,
      mathy_remap(-desired_controller_x, -127, 127, -200, 200),
      mathy_remap(-desired_controller_y, -127, 127, -200, 200),
      mathy_remap(WVelocity, -127, 127, -200, 200));
    }

/*
    base_move_velocity(base,
      mathy_remap(field_based_controller_x, -127, 127, -200, 200),
      mathy_remap(field_based_controller_y, -127, 127, -200, 200),
      mathy_remap(desired_controller_w, -127, 127, -200, 200));
*/

    //printf("x: %f, Y: %f, W: %f\n", pose.x, pose.y, pose.w);
    //printf("left: %d, right: %d, strafe: %d\n", adi_encoder_get(left_encoder), adi_encoder_get(right_encoder), adi_encoder_get(strafe_encoder));

    delay(20);
  }
}
