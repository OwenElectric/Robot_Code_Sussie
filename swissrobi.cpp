#include "mbed.h"

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"


#include "pm2_drivers/DCMotor.h"

#include "pm2_drivers/UltrasonicSensor.h"

#include "eigen/Dense.h"

#include "pm2_drivers/SensorBar.h"

#include "pm2_drivers/LineFollower.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button
                                     // falling and rising edge
void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                     // button gets pressed, definition below

int main()
{
    // attach button fall function address to user button object, button has a pull-up resistor
    user_button.fall(&toggle_do_execute_main_fcn);

    enum RobotState {
        FOLLOW,
        WALL,
        LINE_BREAK,
        EMERGENCY
        // add states here
    } robot_state = RobotState::FOLLOW;


    const int main_task_period_ms = 20;
    Timer main_task_timer;      

    const float bar_dist = 0.118f; // distance from wheel axis to leds on sensor bar / array in meters

    // PIN DECLERATIONS     
                                      
    DigitalOut user_led(USER_LED);
    DigitalOut led1(PB_9);
    UltrasonicSensor us_sensor(PB_D3);
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    SensorBar sensorBar(PB_9, PB_8, bar_dist);

    // BEGIN MOTOR DEFINITIONS

    const float r1_wheel = 0.059f / 2.0; // right wheel radius in meters
    const float r2_wheel = 0.059f / 2.0; // left  wheel radius in meters
    const float b_wheel = 0.1618f;  // wheelbase, distance from wheel to wheel in meters

    float us_distance_cm = 20.0f;

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    const float gear_ratio = 100.0f; // gear ratio
    const float kn = 140.0f / 12.0f;  // motor constant [rpm/V]

    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);


    // Setting Motor 1 to max speed off the rip
    // motor_M1.enableMotionPlanner();
    // motor_M1.setMaxVelocity(motor_M1.getMaxVelocity() * 1.0f);
    // motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 1.0f);

    // // Setting Motor 2 to max speed off the rip
    // motor_M2.enableMotionPlanner();
    // motor_M2.setMaxVelocity(motor_M2.getMaxVelocity() * 1.0f);
    // motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 1.0f);

    // BEGIN LINE SENSOR DEFINITIONS

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller
    const float Kp{17.0f};

    // velocity controller data
    const float wheel_vel_max = 2.0f * M_PI * motor_M2.getMaxPhysicalVelocity();

    const float d_wheel = 0.059f;  // wheel diameter in meters

    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r1_wheel / 2.0f   ,  r2_wheel / 2.0f   ,
                    r1_wheel / b_wheel, -r2_wheel / b_wheel;

    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity());

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            enable_motors = 1;

            // only update sensor bar angle if a led is triggered
            if (sensorBar.isAnyLedActive()) {
                angle = sensorBar.getAvgAngleRad();
            }

             // state machine
            switch (robot_state) {
                case RobotState::FOLLOW:
                                                // control algorithm in robot velocities
                        Eigen::Vector2f robot_coord = {1.0f * wheel_vel_max * r1_wheel, // half of the max. forward velocity
                                                    Kp * angle};                     // proportional angle controller

                        // map robot velocities to wheel velocities in rad/sec
                        Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

                                                // setpoints for the dc-motors in rps
                        motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M2
                        
                    break;
                // case RobotState::TURN:
                //     if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                //         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                //         motor_M1.setRotationRelative(wheel_angle_turn(0) / (2.0f * M_PI));
                //         motor_M2.setRotationRelative(wheel_angle_turn(1) / (2.0f * M_PI));
                //         robot_state = RobotState::FORWARD_OR_RESET;
                //     }
                //     break;
                // case RobotState::FORWARD_OR_RESET:
                //     if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                //         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                //         turn_cntr++;
                //         if (turn_cntr == 4) {
                //             robot_state = RobotState::RESET;
                //         } else {
                //             robot_state = RobotState::FORWARD;
                //         }
                //     }
                //     break;
                // case RobotState::EMERGENCY:

                //     break;
                // default:
                //    break;
            }

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                led1 = 0;
                enable_motors = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        printf("The value of angle is: %.2f\n", angle);
        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}