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
    // Setup
    left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    piston_locked();
    intake.move(127);
    odom_lift.set_value(false);

    // Intake 6 blocks from park zone
    left_drive.move(60);
    right_drive.move(60);
    pros::delay(1000);

    left_drive.move(-40);
    right_drive.move(-40);
    pros::delay(100);

    left_drive.move(60);
    right_drive.move(60);
    pros::delay(600);

    // Reset odom
    left_drive.move(-55);
    right_drive.move(-55);
    pros::delay(750);
    odom_lift.set_value(true);

    left_drive.move(20);
    right_drive.move(20);
    pros::delay(1000);

    left_drive.brake();
    right_drive.brake();
    pros::delay(50);
    chassis.setPose({0, 0, chassis.getPose().theta});
    pros::delay(50);

    // Get 1 blue block
    chassis.swingToPoint(10.5, -29, lemlib::DriveSide::RIGHT, 800, {.forwards = false, .minSpeed = 10});
    chassis.moveToPoint(10.5, -29, 1200, {.forwards = false, .maxSpeed = 80});
    chassis.turnToPoint(19.2, -31.2, 1200, {.maxSpeed = 60});

    chassis.moveToPoint(19.2, -31.2, 1200, {.maxSpeed = 60});

    // Score on mid goal
    chassis.turnToPoint(10, -41, 800, {.forwards = false});
    chassis.moveToPoint(10, -41, 700, {.forwards = false});

    pros::delay(600);

    intake.brake();
    piston_middle();
    pros::delay(100);

    intake.move(85);
    chassis.turnToHeading(45, 300, {.minSpeed = 5}, false);
    pros::delay(400);

    intake.move(70);
    pros::delay(850);

    intake.move(60);
    pros::delay(1200);
    chassis.moveToPoint(50, -1, 100, {.maxSpeed = 40});
    piston_locked();

    // Get 3 block group
    chassis.moveToPoint(50, -1, 2000, {.maxSpeed = 100});
    chassis.waitUntil(4);
    intake.move(127);

    // Long goal
    chassis.turnToPoint(50.5, -21, 800, {.forwards = false});
    chassis.moveToPoint(50.5, -21, 1000, {.forwards = false, .minSpeed = 10});

    pros::delay(600);
    piston_long();

    chassis.turnToHeading(0, 300, {.minSpeed = 30});
    chassis.moveToPoint(50.5, -21, 300, {.forwards = false, .minSpeed = 100});
    pros::delay(500);

    // 1st matchloader
    chassis.setPose({0, 0, chassis.getPose().theta - 2.5});
    pros::delay(50);

    matchloader.set_value(true);
    chassis.moveToPoint(-1, 32, 1200, {.maxSpeed = 60, .minSpeed = 10});
    chassis.waitUntil(5);
    piston_locked();

    chassis.waitUntilDone();
    pros::delay(1300);

    // Got to score
    chassis.moveToPoint(-1.1, 19, 1000, {.forwards = false});
    chassis.turnToPoint(-12.5, 6.6, 800, {.forwards = false});
    chassis.moveToPoint(-12.5, 6.6, 800, {.forwards = false});

    chassis.turnToPoint(-11.5, -73, 800, {.forwards = false});
    chassis.moveToPoint(-11.5, -73, 2000, {.forwards = false, .maxSpeed = 100});

    chassis.turnToPoint(2, -74.5, 800, {.forwards = false});
    chassis.moveToPoint(2, -74.5, 800, {.forwards = false});

    chassis.turnToPoint(2.5, -58, 800, {.forwards = false});
    chassis.moveToPoint(2.5, -58, 800, {.forwards = false, .minSpeed = 10});

    pros::delay(600);
    piston_long();

    chassis.turnToHeading(180, 400, {.minSpeed = 30});
    chassis.moveToPoint(2.5, -58, 400, {.forwards = false, .minSpeed = 100});

    pros::delay(1500);
    chassis.setPose({0, 0, chassis.getPose().theta});
    pros::delay(50);

    // 2nd matchloader
    chassis.moveToPoint(-1.5, -33, 1200, {.maxSpeed = 60, .minSpeed = 10});
    piston_locked();

    chassis.waitUntilDone();
    pros::delay(1300);

    // Score 2nd load
    chassis.moveToPoint(0, 2, 1000, {.forwards = false, .minSpeed = 10});

    pros::delay(800);
    piston_long();

    chassis.turnToHeading(180, 400, {.minSpeed = 30});
    chassis.moveToPoint(0, 2, 400, {.forwards = false, .minSpeed = 100});

    pros::delay(1500);
    chassis.setPose({0, 0, chassis.getPose().theta});
    pros::delay(50);

    // Cross field
    chassis.moveToPoint(0, -12.5, 800);
    matchloader.set_value(false);
    piston_locked();

    chassis.turnToPoint(-100, -14, 800);
    chassis.moveToPoint(-100, -14, 3000, {.maxSpeed = 100});

    // 3rd matchloader
    chassis.turnToPoint(-101, -33, 800);
    chassis.waitUntilDone();

    matchloader.set_value(true);
    chassis.moveToPoint(-101, -33, 1000, {.maxSpeed = 60, .minSpeed = 10});

    chassis.waitUntilDone();
    pros::delay(1300);

    // Score 3rd load
    chassis.moveToPoint(-101, -22.5, 1000, {.forwards = false});
    chassis.turnToPoint(-88, -13, 800, {.forwards = false});
    chassis.moveToPoint(-88, -13, 800, {.forwards = false});
    
    chassis.turnToPoint(-87, 74, 800, {.forwards = false});
    chassis.moveToPoint(-87, 74, 2000, {.forwards = false, .maxSpeed = 100});

    chassis.turnToPoint(-101, 74, 800);
    chassis.moveToPoint(-101, 74, 800);

    chassis.turnToPoint(-100.5, 58, 800, {.forwards = false});
    chassis.moveToPoint(-100.5, 58, 800, {.forwards = false, .minSpeed = 20});

    pros::delay(600);
    piston_long();

    chassis.turnToHeading(0, 400, {.minSpeed = 30});
    chassis.moveToPoint(-100.5, 58, 400, {.forwards = false, .minSpeed = 100});

    pros::delay(1500);
    chassis.setPose({0, 0, chassis.getPose().theta});
    pros::delay(50);

    // 4th matchloader
    chassis.moveToPoint(0.5, 32, 1200, {.maxSpeed = 60, .minSpeed = 10});
    piston_locked();

    chassis.waitUntilDone();
    pros::delay(1300);

    // Score 4th load
    chassis.moveToPoint(0, -2, 1000, {.forwards = false, .minSpeed = 10});

    pros::delay(800);
    piston_long();

    chassis.turnToHeading(0, 400, {.minSpeed = 30});
    chassis.moveToPoint(0, -2, 400, {.forwards = false, .minSpeed = 100});

    pros::delay(1200);
    chassis.setPose({0, 0, chassis.getPose().theta});
    pros::delay(50);

    // Go to park
    matchloader.set_value(false);
    chassis.swingToPoint(16, 33, lemlib::DriveSide::RIGHT, 800, {.minSpeed = 20});
    chassis.moveToPoint(16, 33, 2000, {.minSpeed = 10});
    chassis.turnToHeading(85, 1000, {.minSpeed = 20});

    chassis.waitUntilDone();
	
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
                intake_speed = 67;
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