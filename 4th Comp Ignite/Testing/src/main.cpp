#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "main.h"

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup left_drive({-16, -8, -7}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({15, 4, 3}, pros::MotorGearset::blue);

pros::MotorGroup intake({6, -5}, pros::MotorGearset::blue);

/* --------------------------------- Sensors -------------------------------- */
pros::IMU inertial(2);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut descore('D');
pros::adi::DigitalOut lock('E');
pros::adi::DigitalOut matchloader('F');
pros::adi::DigitalOut trapdoor('G');

/* ---------------------------- Global Variables ---------------------------- */
bool matchloader_down = false;
bool descore_up = true;

void initialize() {
	// Motor Stopping
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Setup
	pros::lcd::initialize();

	// Brain Screen
	// pros::Task screen_task([&]() {
	// 	while (true) {
	// 		pros::lcd::print(2, "X: %f", chassis.getPose().x);
	// 		pros::lcd::print(3, "Y: %f", chassis.getPose().y);
	// 		pros::lcd::print(4, "Theta: %f", chassis.getPose().theta);

	// 		pros::delay(50);
	// 	}
    // });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;

	int intake_speed = 0;

	descore.set_value(true);

	// loop forever
    while (true) {
        /* --------------------------- Drivetrain Control --------------------------- */
        // Get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Deadzone
		if (abs(leftY) < dead_zone) {
            leftY = 0;
        }
        if (abs(rightX) < dead_zone) {
            rightX = 0;
        }

        // Move motors
        left_drive.move(leftY + rightX);
        right_drive.move(leftY - rightX);

		/* ----------------------------- Intake Control ---------------------------- */
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake_speed = 127;

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			// Outtake
			intake_speed = -127;

		} else {
            intake_speed = 0;
        }

        intake.move(intake_speed);

		/* --------------------------------- Pistons -------------------------------- */
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            lock.set_value(true);

        } else {
            lock.set_value(false);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            trapdoor.set_value(true);
            
        } else {
            trapdoor.set_value(false);
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			matchloader_down = !matchloader_down;
			matchloader.set_value(matchloader_down);

		} else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			descore_up = !descore_up;
			descore.set_value(descore_up);
		}

		pros::delay(20);
	}
}