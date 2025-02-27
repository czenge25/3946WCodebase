#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

// making new controllers
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller side(pros::E_CONTROLLER_PARTNER);

// motor groups
pros::MotorGroup leftMotors({-5, -10, -16}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({8, 6, 2}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor intake(18, pros::v5::MotorGears::green);
pros::Motor hooks(1, pros::v5::MotorGears::blue);
pros::Motor wall(21, pros::v5::MotorGears::blue);

pros::adi::DigitalOut latch('G');
pros::adi::DigitalOut doinker('D');

pros::Imu imu(13);
pros::Optical colorSensor(17);

const int redMax = 100;
const int greenMax = 200;
const int blueMax = 600;

bool latchState;
bool doinkerState;
int vHooks;
int vIntake;
char desiredColor;

void toggle(bool &state, pros::adi::DigitalOut &output) {
	state = !state;
	output.set_value(state);
	pros::delay(300); // Delay to debounce button press
	
}

char readColorSensor() {
    int hue = colorSensor.get_hue();
    int proximity = colorSensor.get_proximity();

    pros::c::optical_raw_s_t raw_values = colorSensor.get_raw();

    if (proximity > 200) {
        if ((raw_values.red - 150) > raw_values.blue) {
            return 'r';
        } else if ((raw_values.blue + 25) > raw_values.red) {
            return 'b';
        }
    }
    return 'n'; // No valid object detected

    // if (proximity < 20) {
    //     if (hue >= 0 && hue <= 25) {
    //         return 'r'; // Red
    //     } else if (hue >= 170 && hue <= 240) {
    //         return 'b'; // Blue
    //     }
    // }
    // return 'n'; // No valid object detected
}

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 10 inch track width 7.875"
                              lemlib::Omniwheel::OLD_325, // using 3.25" omnis
                              360, // drivetrain rpm is 360
                              4 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

	vHooks = blueMax;
	vIntake = -greenMax;

    colorSensor.set_led_pwm(85); // brightness of color sensor
    wall.tare_position(); // reset walls position
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    chassis.calibrate();

    for (auto motor : {-5, -10, -16}) {
        pros::Motor(motor).set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    for (auto motor : {8, 6, 2}) {
        pros::Motor(motor).set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    chassis.setPose(0, 0, 0);
    


}

/**
 * Runs in driver control
 */
void opcontrol() {

    desiredColor = 'n';

    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // get left joystick y position
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // get right joystick y position
        // move the chassis with curvature drive
        chassis.tank(leftY, rightY);
        // delay to save resources
        pros::delay(10);

		if (master.get_digital(DIGITAL_L1) || side.get_digital(DIGITAL_L1)) {
			intake.move_velocity(vIntake); // move intake out 
			hooks.move_velocity(vHooks); // move hooks out
		} else if (master.get_digital(DIGITAL_L2) || side.get_digital(DIGITAL_L2)) {
			intake.move_velocity(-vIntake);  // move hooks in
            hooks.move_velocity(-vHooks); // move hooks out
            if ((desiredColor != readColorSensor()) && (desiredColor != 'n' && readColorSensor() != 'n')) {
                int initialTicks = hooks.get_position();
                while (fabs(hooks.get_position() - initialTicks) < 320) {
                    pros::delay(10);
                }
                hooks.move_velocity(vHooks);
                pros::delay(100);
            }
            hooks.move_velocity(-vHooks);
		} else {
			intake.move_velocity(0);
			hooks.move_velocity(0);
		}
        // else if (side.get_digital(DIGITAL_L1)) {
		// 	intake.move_velocity(vIntake);
		// } else if (side.get_digital(DIGITAL_A)) {
		// 	intake.move_velocity(-vIntake);
		// } else if (side.get_digital(DIGITAL_R1)) {
		// 	hooks.move_velocity(vHooks);
		// } else if (side.get_digital(DIGITAL_R2)) {
		// 	hooks.move_velocity(-vHooks);
		// } 
            

		if (master.get_digital(DIGITAL_R1) || side.get_digital(DIGITAL_B)) { 
			toggle(latchState,latch);
		}

		if (master.get_digital(DIGITAL_R2)) { 
			toggle(doinkerState, doinker);
		}
		
		if (master.get_digital(DIGITAL_LEFT)) {
			wall.move_absolute(0, -greenMax/2);
		} else if (master.get_digital(DIGITAL_UP) || side.get_digital(DIGITAL_UP)) {
			wall.move_absolute(230, greenMax);
		} else if (master.get_digital(DIGITAL_RIGHT)) {
			wall.move_absolute(1025,greenMax);
		} else if (master.get_digital(DIGITAL_DOWN)) {
			wall.move_absolute(1125, greenMax);
		} else {
			wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}

        if (master.get_digital(DIGITAL_A)) {
            desiredColor = 'r';
        } else if (master.get_digital(DIGITAL_B)) {
            desiredColor = 'b';
        } else if (master.get_digital(DIGITAL_X)) {
            desiredColor = 'n';
        }
    }
}