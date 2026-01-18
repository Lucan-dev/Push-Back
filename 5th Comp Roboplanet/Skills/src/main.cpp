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
pros::Rotation vert_tracker(9);
pros::Optical intake_color(21);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut descore('D');
pros::adi::DigitalOut lock('E');
pros::adi::DigitalOut trapdoor('F');
pros::adi::DigitalOut matchloader('G');

/* ----------------------------- Tracking Wheels ---------------------------- */
lemlib::TrackingWheel vert_wheel(&vert_tracker, lemlib::Omniwheel::NEW_2, -0.5);

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
    6.5,
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
bool trapdoor_down = false;

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

void wait_until_red() {
    int current_hue = intake_color.get_hue();

	while (current_hue > 15) {
		current_hue = intake_color.get_hue();
		pros::delay(50);
	}
}

void wait_until_blue() {
    intake_color.set_led_pwm(100);
    pros::delay(100);
    int current_hue = intake_color.get_hue();

	while (current_hue < 100) {
		current_hue = intake_color.get_hue();
		pros::delay(50);
	}

    intake_color.set_led_pwm(0);
}

void intake_hold() {
    
}

void outake() {

}

void intake_stop() {}

void intake_score() {}

void autonomous() {
    // 1st matchloader
    chassis.moveToPoint(0, 39, 1500, {.maxSpeed = 80});
    chassis.turnToPoint(14, 40, 1000);

    chassis.waitUntilDone();
    matchloader.set_value(true);
    intake.move(127);

    pros::delay(200);
    chassis.moveToPoint(14, 40, 1500, {.maxSpeed = 60, .minSpeed = 10});

    chassis.waitUntilDone();
    pros::delay(1500);

    // Score 1st load
    chassis.moveToPoint(1, 40, 1000, {.forwards = false});
    chassis.turnToPoint(-16, 52, 800, {.forwards = false});
    chassis.moveToPoint(-19, 52, 1000, {.forwards = false, .maxSpeed = 80});

    chassis.turnToPoint(-91, 50, 800, {.forwards = false});
    intake.brake();
    chassis.moveToPoint(-91, 50, 2500, {.forwards = false, .maxSpeed = 80});

    chassis.waitUntilDone();
    matchloader.set_value(false);
    pros::delay(200);

    chassis.turnToPoint(-91.5, 38, 800, {.forwards = false});
    chassis.moveToPoint(-91.5, 38, 800, {.forwards = false});

    chassis.turnToPoint(-79, 37.5, 800, {.forwards = false});
    chassis.moveToPoint(-79, 37.5, 1000, {.forwards = false,.maxSpeed = 80});
    matchloader.set_value(true);

    chassis.waitUntilDone();
    intake.move(-127);
    pros::delay(150);

    lock.set_value(true);
    intake.move(127);

    pros::delay(3000);
    lock.set_value(false);

    // 2nd matchloader
    chassis.moveToPoint(-113, 36.5, 1500, {.maxSpeed = 60, .minSpeed = 10});
    chassis.waitUntilDone();
    pros::delay(1500);

    // Score 2nd load
    chassis.moveToPoint(-79, 37.5, 1000, {.forwards = false, .maxSpeed = 80});

    chassis.waitUntilDone();
    intake.move(-127);
    pros::delay(150);

    lock.set_value(true);
    intake.move(127);
    pros::delay(3000);
    lock.set_value(false);

    // 3rd matchloader
    chassis.moveToPoint(-93, 37.5, 800);
    intake.brake();
    matchloader.set_value(false);

    chassis.turnToPoint(-94, -62, 800);
    chassis.moveToPoint(-94, -62, 2500, {.maxSpeed = 80});

    chassis.turnToPoint(-114, -62, 800);
    chassis.waitUntilDone();
    matchloader.set_value(true);
    intake.move(127);

    pros::delay(200);
    chassis.moveToPoint(-114, -62, 1500, {.maxSpeed = 60, .minSpeed = 10});

    chassis.waitUntilDone();
    pros::delay(1500);

    // Score 3rd load
    chassis.moveToPoint(-101, -63.5, 800, {.forwards = false});
    chassis.turnToPoint(-81, -75.5,800, {.forwards = false});
    chassis.moveToPoint(-81, -75.5, 1500, {.forwards = false, .maxSpeed = 100});

    chassis.turnToPoint(-11, -75.5, 800, {.forwards = false});
    intake.brake();
    chassis.moveToPoint(-11, -75.5, 2500, {.forwards = false, .maxSpeed = 80});

    chassis.waitUntilDone();
    matchloader.set_value(false);
    pros::delay(200);

    chassis.turnToPoint(-11, -63, 800, {.forwards = false});
    chassis.moveToPoint(-11, -63, 800, {.forwards = false});

    chassis.turnToPoint(-23, -63, 800, {.forwards = false});
    chassis.moveToPoint(-23, -63, 1000, {.forwards = false,.maxSpeed = 80});
    matchloader.set_value(true);

    chassis.waitUntilDone();
    intake.move(-127);
    pros::delay(150);

    lock.set_value(true);
    intake.move(127);

    pros::delay(3000);
    lock.set_value(false);

    // 4th matchloader
    chassis.moveToPoint(11, -63, 1500, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(1500);

    // Score 4th load
    chassis.moveToPoint(-23, -63, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intake.move(-127);
    pros::delay(150);

    lock.set_value(true);
    intake.move(127);
    pros::delay(3000);
    lock.set_value(false);

    // Park
    matchloader.set_value(false);
    intake.brake();
    chassis.moveToPoint(-11.5, -63.5, 800);
    
    chassis.turnToPoint(-8.6, -14.5, 800);
    chassis.moveToPoint(-8.5, -14.5, 2000, {.maxSpeed = 80});

    chassis.turnToPoint(1, -14, 1000);
    chassis.moveToPoint(1, -14, 1500, {.maxSpeed = 80});
    chassis.waitUntilDone();
    descore.set_value(true);

    left_drive.move(80);
    right_drive.move(80);

    pros::delay(1500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    left_drive.brake();
    right_drive.brake();

    // Ending
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.brake();
}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;
	int intake_speed = 0;

	descore.set_value(true);
    descore_up = true;

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
            if (trapdoor_down) {
                intake_speed = 80;
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
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            lock.set_value(true);

        } else {
            lock.set_value(false);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            trapdoor_down = !trapdoor_down;
            trapdoor.set_value(trapdoor_down);
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