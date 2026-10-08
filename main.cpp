/*
  _____    _______  _____ ______      
 / __  \  /  ___  \|\   _ \  _   \    
|\/_|\  \/__/|_/  /\ \  \\\__\ \  \   
\|/ \ \  \__|//  / /\ \  \\|__| \  \  
     \ \  \  /  /_/__\ \  \    \ \  \ 
      \ \__\|\________\ \__\    \ \__\
       \|__| \|_______|\|__|     \|__|

*/
#include "main.h"

ez::Drive chassis(
    {-1, -2},
    {3, 4},
    7,
    2.75,
    480
);

pros::MotorGroup cascades({19, -20});
pros::Motor wrist(11);
pros::MotorGroup intake_and_claw({5, 7});

bool intake_on = false;
bool intake_out = false;

const double WRIST_MAX_POS = 900.0;
const double WRIST_MIN_SPEED = 0.1;


// Red Back
void red_back() {
    chassis.pid_drive_set(5_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-8_in, 100);
    chassis.pid_wait();

    chassis.pid_drive_set(8_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-8_in, 100);
    chassis.pid_wait();

    chassis.pid_drive_set(8_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-3.2_in, 100);
    chassis.pid_wait();

    cascades.move(127);
    pros::delay(200);
    cascades.move(0);

    chassis.pid_turn_set(-70_deg, 100);
    chassis.pid_wait();

    pros::delay(1000);

    wrist.move_relative(500, 100);
    pros::delay(500);

    chassis.pid_drive_set(27.56_in, 20);
    chassis.pid_wait();

    pros::delay(1500);

    intake_and_claw.move(-127);
}


// Blue Back
void blue_back() {
    chassis.pid_drive_set(5_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-8_in, 100);
    chassis.pid_wait();

    chassis.pid_drive_set(8_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-3.2_in, 100);
    chassis.pid_wait();

    cascades.move(127);
    pros::delay(200);
    cascades.move(0);

    chassis.pid_turn_set(-70_deg, 100);
    chassis.pid_wait();

    pros::delay(1000);

    wrist.move_relative(500, 100);
    pros::delay(500);

    chassis.pid_drive_set(27.56_in, 20);
    chassis.pid_wait();

    pros::delay(1500);

    intake_and_claw.move(-127);
}


// Red Side
void red_side() {
    chassis.pid_drive_set(5_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-8_in, 100);
    chassis.pid_wait();

    chassis.pid_drive_set(8_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-8_in, 100);
    chassis.pid_wait();

    chassis.pid_drive_set(8_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-3.2_in, 100);
    chassis.pid_wait();

    cascades.move(127);
    pros::delay(200);
    cascades.move(0);

    chassis.pid_turn_set(70_deg, 100);
    chassis.pid_wait();

    pros::delay(1000);

    wrist.move_relative(500, 100);
    pros::delay(500);

    chassis.pid_drive_set(27.56_in, 20);
    chassis.pid_wait();

    pros::delay(1500);

    intake_and_claw.move(-127);
}


// Blue Side
void blue_side() {
    chassis.pid_drive_set(5_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-8_in, 100);
    chassis.pid_wait();

    chassis.pid_drive_set(8_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-8_in, 100);
    chassis.pid_wait();

    chassis.pid_drive_set(8_in, 100);
    chassis.pid_wait();

    pros::delay(750);

    chassis.pid_drive_set(-3.2_in, 100);
    chassis.pid_wait();

    cascades.move(127);
    pros::delay(200);
    cascades.move(0);

    chassis.pid_turn_set(70_deg, 100);
    chassis.pid_wait();

    pros::delay(1000);

    wrist.move_relative(500, 100);
    pros::delay(500);

    chassis.pid_drive_set(27.56_in, 20);
    chassis.pid_wait();

    pros::delay(1500);

    intake_and_claw.move(-127);
}


void initialize() {
    ez::ez_template_print();

    pros::delay(500);

    chassis.opcontrol_curve_buttons_toggle(true);
    chassis.opcontrol_drive_activebrake_set(0.0);
    chassis.opcontrol_curve_default_set(0.0, 0.0);

    ez::as::auton_selector.autons_add({
        {"Red Back", red_back},
        {"Blue Back", blue_back},
        {"Red Side", red_side},
        {"Blue Side", blue_side},
    });

    chassis.initialize();
    ez::as::initialize();

    cascades.set_brake_mode(MOTOR_BRAKE_HOLD);
    wrist.set_brake_mode(MOTOR_BRAKE_HOLD);

    wrist.set_encoder_units(MOTOR_ENCODER_DEGREES);
    wrist.tare_position();

    master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}


void disabled() {
}


void competition_initialize() {
}


void autonomous() {
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

    ez::as::auton_selector.selected_auton_call();
}


void ez_template_extras() {
    if (!pros::competition::is_connected()) {

        if (master.get_digital_new_press(DIGITAL_X)) {
            chassis.pid_tuner_toggle();
        }

        if (master.get_digital(DIGITAL_B) &&
            master.get_digital(DIGITAL_DOWN)) {

            pros::motor_brake_mode_e_t preference =
                chassis.drive_brake_get();

            autonomous();

            chassis.drive_brake_set(preference);
        }

        chassis.pid_tuner_iterate();
    }
    else {
        if (chassis.pid_tuner_enabled()) {
            chassis.pid_tuner_disable();
        }
    }
}


void opcontrol() {
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);

    while (true) {
        ez_template_extras();

        // Drive
        chassis.opcontrol_arcade_standard(ez::SPLIT);

        // Wrist
        double wrist_position = wrist.get_position();
        double t = wrist_position / WRIST_MAX_POS;

        if (t < 0.0) {
            t = 0.0;
        }

        if (t > 1.0) {
            t = 1.0;
        }

        double wrist_speed =
            0.5 - t * (0.5 - WRIST_MIN_SPEED);

        if (master.get_digital(DIGITAL_DOWN)) {
            wrist.move(127 * wrist_speed);
        }
        else if (master.get_digital(DIGITAL_B)) {
            wrist.move(-80 * wrist_speed);
        }
        else {
            wrist.brake();
        }

        // Cascade
        if (master.get_digital(DIGITAL_L1)) {
            cascades.move(-127);
        }
        else if (master.get_digital(DIGITAL_L2)) {
            cascades.move(100);
        }
        else {
            cascades.move(0);
        }

        // Intake
        if (master.get_digital_new_press(DIGITAL_R1)) {
            intake_on = !intake_on;
            intake_out = false;
        }

        if (master.get_digital_new_press(DIGITAL_R2)) {
            intake_out = !intake_out;
            intake_on = false;
        }

        if (intake_on) {
            intake_and_claw.move(127);
        }
        else if (intake_out) {
            intake_and_claw.move(-127);
        }
        else {
            intake_and_claw.move(0);
        }

        pros::delay(ez::util::DELAY_TIME);
    }
}