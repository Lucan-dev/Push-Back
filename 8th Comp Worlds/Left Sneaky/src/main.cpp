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
pros::adi::DigitalOut odom_lift('A');
pros::adi::DigitalOut mid_descore('B');
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
    odom_lift.set_value(true);

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
    // Robot starts 15 degrees to the left
    piston_locked();
    odom_lift.set_value(true);
    descore.set_value(true);

    // Group of 3 blocks
	intake.move(127);
    chassis.moveToPoint(0, 29, 2000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.waitUntil(15);
    matchloader.set_value(true);

    // Matchloader
    chassis.turnToPoint(-31, 8, 800, {.minSpeed = 10, .earlyExitRange = 4});
    chassis.moveToPoint(-31, 8, 1500, {.minSpeed = 10, .earlyExitRange = 2});

    chassis.turnToPoint(-36.5, -4, 600, {.minSpeed = 10, .earlyExitRange = 4});
    chassis.moveToPoint(-36.5, -4, 700, {.maxSpeed = 100, .minSpeed = 20});

    chassis.waitUntilDone();
    pros::delay(200);

    // Middle Goal
    chassis.moveToPoint(-33.1, 6.4, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2});

    chassis.turnToPoint(17, 35, 800, {.forwards = false, .minSpeed = 10, .earlyExitRange = 4});
    chassis.moveToPoint(17, 35, 1000, {.forwards = false, .minSpeed = 10});

    chassis.waitUntil(32);
    chassis.cancelAllMotions();
    chassis.moveToPoint(17, 35, 1200, {.forwards = false, .maxSpeed = 50});

    pros::delay(300);
    intake.move(-50);
    pros::delay(100);
    piston_middle();

    intake.move(100);
    pros::delay(500);
    piston_locked();
    pros::delay(200);

    // Sneaky Descore
    chassis.moveToPoint(-13.5, 16.5, 1000,{.minSpeed = 10, .earlyExitRange = 2});
    matchloader.set_value(false);

    chassis.turnToPoint(-11.5, 49, 800, {.minSpeed = 10, .earlyExitRange = 4});
    chassis.moveToPoint(-11.5, 49, 1000, {.minSpeed = 10, .earlyExitRange = 2});

    chassis.turnToHeading(17.5, 300, {.minSpeed = 20, .earlyExitRange = 4});
    descore.set_value(false);

    pros::delay(100);
    chassis.moveToPoint(-22, 16, 1500, {.forwards = false, .maxSpeed = 60});

    chassis.waitUntil(6);
    chassis.cancelAllMotions();

    chassis.moveToPoint(-20, 19, 1500, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2});

    // Score
    chassis.turnToPoint(-28, 18, 800, {.minSpeed = 15, .earlyExitRange = 4});
    intake.brake();
    chassis.moveToPoint(-28, 18, 800, {.minSpeed = 25, .earlyExitRange = 2});

    chassis.turnToPoint(-26.5, 29, 800, {.forwards = false, .minSpeed = 15, .earlyExitRange = 4});
    chassis.moveToPoint(-26.5, 29, 800, {.forwards = false, .minSpeed = 20});
    pros::delay(200);
    intake.move(127);
    pros::delay(200);
    piston_long();

    chassis.turnToHeading(-165, 300, {.minSpeed = 30});
    pros::delay(300);
    chassis.moveToPoint(-26, 32, 300, {.forwards = false, .minSpeed = 100});
    pros::delay(300);
    chassis.setPose(0, 0, chassis.getPose().theta - 196);

    pros::delay(50);

    // 2nd Descore
    // chassis.swingToHeading(50, lemlib::DriveSide::RIGHT, 900, {.minSpeed = 50, .earlyExitRange = 5});
    // chassis.moveToPoint(10.5, 13, 800, {.minSpeed = 15, .earlyExitRange = 2});
    chassis.moveToPoint(0, 6, 1600, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.moveToPoint(15.5, 14, 1600, {.minSpeed = 30, .earlyExitRange = 2});

    intake.brake();
    chassis.turnToHeading(5, 800, {.minSpeed = 15, .earlyExitRange = 4});
    // chassis.moveToPoint(7.5, -16, 15000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20});
    chassis.moveToPoint(11, -18, 15000, {.forwards = false, .minSpeed = 40});

    // // chassis.swingToHeading(48, lemlib::DriveSide::RIGHT, 800, {.minSpeed = 60, .earlyExitRange = 5});
    // chassis.moveToPoint(0, 5, 1600, {.minSpeed = 10, .earlyExitRange = 2});
    // chassis.moveToPoint(14, 14, 1600, {.minSpeed = 30, .earlyExitRange = 2});

    // intake.brake();
    // chassis.turnToHeading(8, 800, {.minSpeed = 15, .earlyExitRange = 4});

    // left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    // right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    // chassis.moveToPoint(7, -16, 15000, {.forwards = false, .minSpeed = 40});
    
    // Hold position
    left_drive.brake();
    right_drive.brake();
}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;
	int intake_speed = 0;

    /* ----------------------------- Motor Stopping ----------------------------- */
    left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    
    odom_lift.set_value(false);

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
            lock_state = 2;

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            lock_state = 1;

        } else {
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