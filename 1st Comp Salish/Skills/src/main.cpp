#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "main.h"

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup left_drive({-1, 2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({8, -9, 10}, pros::MotorGearset::blue);

pros::Motor intake_bottom(-12, pros::MotorGearset::blue);
pros::Motor intake_top(19, pros::MotorGearset::blue);

/* --------------------------------- Sensors -------------------------------- */
pros::Rotation hor(-20);
pros::Rotation vert(11);
pros::IMU inertial(13);
pros::Optical color_sensor(7);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut matchloader('B');
pros::adi::DigitalOut trapdoor('C');

/* ----------------------------- Tracking Wheels ---------------------------- */
lemlib::TrackingWheel vert_wheel(&vert, lemlib::Omniwheel::NEW_2, -0.9375);
lemlib::TrackingWheel hor_wheel(&hor, lemlib::Omniwheel::NEW_2, -0.1875);

/* ---------------------------- Drivetrain Setup ---------------------------- */
lemlib::Drivetrain drivetrain(
	&left_drive,
    &right_drive,
    10.95,
    lemlib::Omniwheel::NEW_4,
    342.857,
    2
);

lemlib::OdomSensors sensors(
	&vert_wheel,
    nullptr,
    &hor_wheel,
    nullptr,
    &inertial
);

/* ------------------------------- PID Values ------------------------------- */
lemlib::ControllerSettings lateral_controller(
    6.1,
    0,
    10,
    3,
    1,
    100,
    3,
    500,
    15
);

lemlib::ControllerSettings angular_controller(
    2.15,
    0,
    22,
    3,
    1,
    100,
    3,
    500,
    7.6
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
bool trapdoor_down = false;

/* -------------------------------- Functions ------------------------------- */
void intake_hold() {
	intake_bottom.move(127);
	intake_top.move(-30);
}

void intake_score() {
	intake_bottom.move(127);
	intake_top.move(127);
}

void intake_stop() {
	intake_bottom.brake();
	intake_top.brake();
}

void wait_until_blue() {
	int current_hue = color_sensor.get_hue();

	while (current_hue < 80) {
		current_hue = color_sensor.get_hue();
		pros::delay(50);
	}
}

void initialize() {
	// Motor Stopping
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	intake_top.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

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

void autonomous() {
	// 1st Matchloader
	chassis.moveToPoint(0, 39.5, 1500, {.maxSpeed = 100});
	chassis.turnToPoint(7.5, 43.5, 800, {.maxSpeed = 80});

	chassis.moveToPoint(7.5, 43.5, 800, {.maxSpeed = 60, .minSpeed = 35});
	matchloader.set_value(true);
	intake_hold();

	// Score 1st Load
	chassis.waitUntilDone();
	pros::delay(1200);
	chassis.moveToPoint(-25, 43.5, 1000, {.forwards = false, .maxSpeed = 100});

	chassis.waitUntilDone();
	intake_score();

	// 2nd Matchloader
	chassis.waitUntilDone();
	pros::delay(2000);
	matchloader.set_value(false);
	intake_stop();

	chassis.moveToPoint(-11, 43.5, 800);
	chassis.turnToPoint(0, -51.5, 800);
	chassis.moveToPoint(0, -51.5, 2500, {.maxSpeed = 80});

	chassis.turnToPoint(12, -55.5, 800, {.maxSpeed = 80});

	chassis.moveToPoint(12, -55.5, 800, {.maxSpeed = 60, .minSpeed = 35});
	matchloader.set_value(true);
	intake_hold();

	// Score 2nd Load
	chassis.waitUntilDone();
	pros::delay(1200);
	chassis.moveToPoint(-21, -57.5, 1000, {.forwards = false, .maxSpeed = 100});

	chassis.waitUntilDone();
	intake_score();
	pros::delay(1500);

	// Ending
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	intake_stop();
	matchloader.set_value(false);
}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;

	int intake_bottom_speed = 0;
	int intake_top_speed = 0;

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
			if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
				// Intake to score
				intake_bottom_speed = 127;
				intake_top_speed = 127;
			} else {
				// Intake to hold balls
				intake_bottom_speed = 127;
				intake_top_speed = -20;
			}

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			// Outtake
			intake_bottom_speed = -127;
			intake_top_speed = -127;

		} else {
            intake_bottom_speed = 0;
			intake_top_speed = 0;
        }

        intake_bottom.move(intake_bottom_speed);
		intake_top.move(intake_top_speed);

		/* --------------------------------- Pistons -------------------------------- */
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			trapdoor_down = !trapdoor_down;
			trapdoor.set_value(trapdoor_down);

		} else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			matchloader_down = !matchloader_down;
			matchloader.set_value(matchloader_down);
		}

		pros::delay(20);
	}
}