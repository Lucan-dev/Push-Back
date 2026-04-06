#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "main.h"

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup left_drive({-4, -16, -17}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({7, 14, 15}, pros::MotorGearset::blue);

pros::MotorGroup intake({3, -8}, pros::MotorGearset::blue);

/* --------------------------------- Sensors -------------------------------- */
pros::IMU inertial(21);
pros::Rotation vert_tracker(5);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut mid_descore('A');
pros::adi::DigitalOut matchloader('C');
pros::adi::DigitalOut descore('D');
pros::adi::DigitalOut lock_bottom('E');
pros::adi::DigitalOut lock_top('F');

/* ----------------------------- Tracking Wheels ---------------------------- */
lemlib::TrackingWheel vert_wheel(&vert_tracker, lemlib::Omniwheel::NEW_2, 0.25);

/* ---------------------------- Drivetrain Setup ---------------------------- */
lemlib::Drivetrain drivetrain(
	&left_drive,
    &right_drive,
    10.875,
    lemlib::Omniwheel::NEW_325,
    450,
    2
);

lemlib::OdomSensors sensors(
	&vert_wheel,
    nullptr,
    nullptr,
    nullptr,
    &inertial
);

/* ------------------------------- PID Values ------------------------------- */
lemlib::ControllerSettings lateral_controller(
    6,
    0,
    10,
    3,
    1,
    100,
    3,
    500,
    0
);

lemlib::ControllerSettings angular_controller(
    2.3,
    0,
    20,
    3,
    1,
    100,
    3,
    500,
    0
);

/* ----------------------------- Create Chassis ----------------------------- */
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

/* ---------------------------- Global Variables ---------------------------- */
bool matchloader_down = false;
bool descore_up = false;
bool mid_descore_down = false;
int lock_state = 0; // 0 = locked, 1 = long goal, 2 = mid goal

/* ------------------------------- Functions -------------------------------- */
void initialize() {
	// Motor Stopping
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	// Setup
	pros::lcd::initialize();
	chassis.calibrate();

	// Brain Screen
	pros::Task screen_task([&]() {
		while (true) {
			pros::lcd::print(2, "X: %f", chassis.getPose().x);
			pros::lcd::print(3, "Y: %f", chassis.getPose().y);
			pros::lcd::print(4, "Theta: %f", chassis.getPose().theta);

			pros::delay(50);
		}
    });
}

void disabled() {}

void competition_initialize() {}

/* ----------------------------- Custom Function ---------------------------- */
void piston_locked() {
    lock_bottom.set_value(true);
    lock_top.set_value(false);
}

void piston_long() {
    lock_bottom.set_value(false);
    lock_top.set_value(false);
}

void piston_middle() {
    lock_bottom.set_value(true);
    lock_top.set_value(true);
}

void intake_for(int velocity, int mseconds) {
    intake.move(velocity);
    pros::delay(mseconds);
    intake.brake();
}

/* -------------------------- Competition Functions ------------------------- */
void autonomous() {
    piston_locked();    
	intake.move(127);

    // Matchloader
    chassis.moveToPoint(0, 28, 1000);
    matchloader.set_value(true);

    chassis.turnToPoint(10, 29.5, 1000);
    chassis.moveToPoint(10, 29.5, 800);

    chassis.waitUntilDone();
    pros::delay(100);

    // Score
    chassis.moveToPoint(-25, 30.7, 1000, {.forwards = false});
    pros::delay(800);
    piston_long();

    chassis.turnToHeading(90, 300, {.minSpeed = 30});
    chassis.moveToPoint(-26, 30.7, 300, {.minSpeed = 100});
    pros::delay(600);

    chassis.setPose({0, 0, chassis.getPose().theta});
    pros::delay(50);

    intake.brake();
    matchloader.set_value(false);

    // 6 Blocks
    chassis.turnToHeading(196, 800, {.minSpeed = 100, .earlyExitRange = 15});
    chassis.turnToHeading(196, 400, {.minSpeed = 5});

    intake.move(127);
    piston_locked();

    chassis.moveToPoint(2.5, -14, 1000, {.minSpeed = 50, .earlyExitRange = 8});
    chassis.moveToPoint(3.9, -70.7, 1500);

    chassis.waitUntil(50);
    matchloader.set_value(true);

    // Go to Score
    chassis.turnToPoint(30.3, -93.25, 800);
    chassis.moveToPoint(30.3, -93.25, 800);
    chassis.turnToHeading(90, 800);

    chassis.moveToPoint(9.5, -94.4, 800, {.forwards = false});
    pros::delay(600);
    piston_long();

    chassis.turnToHeading(90, 300, {.minSpeed = 30});
    chassis.moveToPoint(9.5, -94.4, 300, {.minSpeed = 100});
    pros::delay(700);

    chassis.setPose({0, 0, chassis.getPose().theta});
    pros::delay(50);

    piston_locked();

    // 2nd Matchloader
    chassis.moveToPoint(29.9, 0.8, 1000);
    chassis.waitUntilDone();

    pros::delay(300);

    // Middle Goal
    chassis.moveToPoint(20.4, 0.8, 800, {.forwards = false});
    chassis.turnToHeading(135, 800);

    chassis.turnToPoint(-24.6, 41.2, 800, {.forwards = false});

    chassis.moveToPoint(-24.6, 41.2, 1000, {.forwards = false, .earlyExitRange = 40});
    chassis.moveToPoint(-24.6, 41.2, 1000, {.forwards = false, .maxSpeed = 30});

    chassis.waitUntilDone();
    intake.move(75);
    piston_middle();
    pros::delay(10000);

    // Ending
    chassis.waitUntilDone();
    pros::delay(500);
    intake.brake();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;
	int intake_speed = 0;

    /* ----------------------------- Motor Stopping ----------------------------- */
    left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

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
            if (lock_state == 2) {
                intake_speed = 127;
            } else {
                intake_speed = 127;
            }


        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			// Outtake
			intake_speed = -127;

		} else {
            intake_speed = 0;
        }

        intake.move(intake_speed);

        /* --------------------------------- Pistons -------------------------------- */
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (lock_state == 2) {
                lock_state = 0;
            } else {
                lock_state = 2;
            }

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            lock_state = 1;

        } else if (lock_state != 2) {
            lock_state = 0;
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			matchloader_down = !matchloader_down;
			matchloader.set_value(matchloader_down);

		} else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			descore_up = !descore_up;
			descore.set_value(descore_up);

		} else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            mid_descore_down = !mid_descore_down;
            mid_descore.set_value(mid_descore_down);
        }

        if (lock_state == 0) {
            piston_locked();

        } else if (lock_state == 1) {
            piston_long();

        } else {
            piston_middle();
        }

		pros::delay(20);
	}
}