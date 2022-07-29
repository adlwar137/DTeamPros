#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "chassis.h"
#include "flywheel.h"

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

const pros::controller_id_e_t MASTER_CONTROLLER = pros::controller_id_e_t::E_CONTROLLER_MASTER;

//needs to not be global
FILE *file = fopen("file.csv", "a");

//declare subsystems here
chassis base;
flywheel discShooter;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	printf("Initializing");

	//Initialize motors
	base.frontLeftMotor = FRONTLEFT;
	base.frontRightMotor = FRONTRIGHT;
	base.backLeftMotor = BACKLEFT;
	base.backRightMotor = BACKRIGHT;

	discShooter.motorA = FLYWHEELA;
	discShooter.motorB = FLYWHEELB;

	//Set motor gearing
	base_set_gearing(base, pros::E_MOTOR_GEARSET_18);
	flywheel_set_gearing(discShooter, pros::E_MOTOR_GEARSET_06);

	//Set specific motor directions
	motor_set_reversed(base.frontLeftMotor, true);
	motor_set_reversed(base.backLeftMotor, true);
	motor_set_reversed(discShooter.motorA, true);
	motor_set_reversed(INTAKE, true);

	if(file==NULL){
		printf("file error");
	} else {
		printf("file working :)");
	}

	lv_obj_t *label1 = lv_label_create(lv_scr_act(), NULL);

	lv_label_set_text(label1, "Hello World!");

	lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}

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

	bool pistonState = false;

	while(1) {

		if(controller_get_digital_new_press(CONTROLLER_MASTER, DIGITAL_B)) {
			pistonState = !pistonState;
			adi_digital_write(PISTON, pistonState);
		}

		if(controller_get_digital(MASTER_CONTROLLER, DIGITAL_R1)) {
			motor_move(INTAKE, MOTOR_MAX_VOLTAGE);
		} else if (controller_get_digital(MASTER_CONTROLLER, DIGITAL_R2)) {
			motor_move(INTAKE, MOTOR_MIN_VOLTAGE);
		} else {
			motor_brake(INTAKE);
		}

		base_move(base, controller_get_analog(MASTER_CONTROLLER, ANALOG_LEFT_X), controller_get_analog(MASTER_CONTROLLER, ANALOG_LEFT_Y));
		base_turn(base, controller_get_analog(MASTER_CONTROLLER, ANALOG_RIGHT_X));

		flywheel_spin(discShooter, controller_get_analog(MASTER_CONTROLLER, ANALOG_RIGHT_Y));

		if(file != NULL) {
			const int firstSpeed = motor_get_actual_velocity(FLYWHEELA);
			const int secondSpeed = motor_get_actual_velocity(FLYWHEELB);
			const double firstTorque = motor_get_torque(FLYWHEELA);
			const double secondTorque = motor_get_torque(FLYWHEELB);
			const double firstTemperature = motor_get_temperature(FLYWHEELA);
			const double secondTemperature = motor_get_temperature(FLYWHEELB);
			fprintf(file, "%d, %f, %f, %d, %f, %f\n", firstSpeed, firstTorque, firstTemperature, secondSpeed, secondTorque, secondTemperature);	
			fclose(file);

			printf("%d, %f, %f, %d, %f, %f\n", firstSpeed, firstTorque, firstTemperature, secondSpeed, secondTorque, secondTemperature);	
		}

		delay(20);
	}
}
