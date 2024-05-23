#include "mbed.h"

#include <iostream>

#include <cmath>

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"


#include "pm2_drivers/DCMotor.h"

#include "pm2_drivers/UltrasonicSensor.h"

#include "eigen/Dense.h"

#include "pm2_drivers/SensorBar.h"

#include "pm2_drivers/LineFollower.h"



//#include "stdint.h"


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
        WALL_BACKUP,
        WALL_TURN,
        LINE_BREAK,
        FOLLOW_TRANS,
        WALL_FORWARD,
        WALL_FORWARD_2,
        WALL_TURN_2,
        WALL_TURN_90,
        JUNCTION,
        JUNCTION_DECISION,
        BACK,
        TURN_T,
        ROTATE,
        FORWARD_3
        //EMERGENCY
    } robot_state = RobotState::FOLLOW;

    string state_tracker = "FOLLOW";


    const int main_task_period_ms = 40;
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
    float turns1 = 0.0;
    float turns2 = 0.0;

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

    bool allSensorsLit = true;
    int sensorValues[8];
    int density;

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

    // variables to track rotation completion
    bool rotation_started = false;
    Timer rotation_timer;
    const float angle_threshold = 0.01f;
    int turn_cntr = 0;

    float garbage = 0.0f;
    float garbage2 = 0.0f;

    uint8_t rawVal = 0;

    //Setup for going aroudn wall
    /*
    float forward_rotations = 1.0f;
    float wall_turn_1 = M_PI/4;
    float wall_turn_2 = -M_PI/4;
    Eigen::Vector2f robot_coord_forward = {forward_rotations, 0.0f};
    Eigen::Vector2f robot_coord_turn_1 = {0.0, wall_turn_1};
    Eigen::Vector2f robot_coord_turn_2 = {0.0, wall_turn_2};
    Eigen::Vector2f wheel_angle_forward = Cwheel2robot.inverse()*robot_coord_forward;
    Eigen::Vector2f wheel_angle_turn_1 =  Cwheel2robot.inverse()*robot_coord_turn_1;
    Eigen::Vector2f wheel_angle_turn_2 =  Cwheel2robot.inverse()*robot_coord_turn_2;
*/


    // this loop will run forever
    while (true) {
        main_task_timer.reset();

            if (sensorBar.isAnyLedActive()) {
                angle = sensorBar.getAvgAngleRad();
            }

        if (do_execute_main_task) {

            rawVal = sensorBar.getRaw();

            
            
            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            enable_motors = 1;

            Eigen::Vector2f robot_coord;
            Eigen::Vector2f wheel_speed;
            Eigen::Vector2f wheel_angle_forward{1.0f, 1.0f}; // wheel angles for forward motion
            Eigen::Vector2f wheel_angle_turn{0.25f, -0.25f}; // wheel angles for 45 degrees turn



             // state machine
            switch (robot_state) {
                case RobotState::FOLLOW:

                        // map robot velocities to wheel velocities in rad/sec
                        robot_coord = {1.0f * wheel_vel_max * r1_wheel, // half of the max. forward velocity
                                        Kp * angle};                     // proportional angle controller
                        wheel_speed = Cwheel2robot.inverse() * robot_coord;

                                                // setpoints for the dc-motors in rps
                        motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M2

                        if (mechanical_button.read()) {
                            state_tracker = "WALL_BACKUP";
                            //motor_M1.setRotationRelative(-wheel_angle_forward(0)/ (2.0f*M_PI));
                            //motor_M2.setRotationRelative(-wheel_angle_forward(1)/ (2.0f*M_PI));
                            //motor_M1.setRotationRelative(-1.0f);
                            //motor_M2.setRotationRelative(-1.0f);
                            robot_state = RobotState::WALL_BACKUP;
                        }

                        if (rawVal == 255) {
                            state_tracker = "JUNCTION";
                            //motor_M1.setRotationRelative(-wheel_angle_forward(0)/ (2.0f*M_PI));
                            //motor_M2.setRotationRelative(-wheel_angle_forward(1)/ (2.0f*M_PI));
                            // motor_M1.setRotationRelative(-0.5f);
                            // motor_M2.setRotationRelative(-1.0f);
                            robot_state = RobotState::JUNCTION;
                        }

                        //Will Added


                        // if (angle <= 0.001 && angle >= -0.001){
                        //     turns1 = motor_M1.getRotation() + 3.0f;
                        //     turns2 = motor_M2.getRotation() + 3.0f;
                        //     robot_state = RobotState::LINE_BREAK;
                        //     state_tracker = "LINE_BREAK";
                        // }

                    
            
                        
                    break;
                case RobotState::WALL_BACKUP:
                    // if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                    //      (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        //motor_M1.setRotationRelative(-0.5f);
                        //motor_M2.setRotationRelative(-0.5f);
                        garbage = motor_M1.getRotation() - 1.0f;

                        motor_M1.setRotation(garbage);
                        motor_M2.setRotation(garbage);

                        robot_state = RobotState::WALL_TURN;
                        state_tracker = "WALL_TURN";
                    //}
                    break;

                case RobotState::WALL_TURN:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        garbage = motor_M1.getRotation() + 0.5f;
                        garbage2 = motor_M2.getRotation() + 0.5f;
                        motor_M1.setRotation(garbage); 
                        motor_M2.setRotation(garbage2 - 1.0);
                        robot_state = RobotState::WALL_FORWARD;
                        state_tracker = "WALL_FORWARD";
                    }
                    break;

                case RobotState::WALL_FORWARD:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        garbage = motor_M1.getRotation() + 1.0f;
                        garbage2 = motor_M2.getRotation() + 1.0f;
                        motor_M1.setRotation(garbage); 
                        motor_M2.setRotation(garbage2);
                        robot_state = RobotState::WALL_TURN_90;
                        state_tracker = "FOLLOW_TURN_90";
                    }
                    break;

                case RobotState::WALL_TURN_90:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        garbage = motor_M1.getRotation() + 1.0f;
                        garbage2 = motor_M2.getRotation() + 1.0f;
                        motor_M1.setRotation(garbage); 
                        motor_M2.setRotation(garbage2 - 2.0);
                        robot_state = RobotState::WALL_FORWARD_2;
                        state_tracker = "WALL_FORWARD_2";
                    }
                    break;

                case RobotState::WALL_FORWARD_2:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        garbage = motor_M1.getRotation() + 2.0f;
                        garbage2 = motor_M2.getRotation() + 2.0f;
                        motor_M1.setRotation(garbage); 
                        motor_M2.setRotation(garbage2);
                        robot_state = RobotState::WALL_TURN_2;
                        state_tracker = "WALL_TURN_2";
                    }
                    break;


                case RobotState::WALL_TURN_2:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        garbage = motor_M1.getRotation() + 0.5f;
                        garbage2 = motor_M2.getRotation() + 0.5f;
                        motor_M1.setRotation(garbage - 1.0); 
                        motor_M2.setRotation(garbage2);
                        robot_state = RobotState::FOLLOW_TRANS;
                        state_tracker = "FOLLOW_TRANS";
                    }
                    break;
                case RobotState::FOLLOW_TRANS:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        robot_state = RobotState::FOLLOW;
                        state_tracker = "FOLLOW";
                    }
                    break;

                case RobotState::JUNCTION:
                    garbage = motor_M1.getRotation() + 0.2f;
                    garbage2 = motor_M2.getRotation() + 0.2f;
                    motor_M1.setRotation(garbage); 
                    motor_M2.setRotation(garbage2);
                    state_tracker = "JUNCTION_DECISION";
                    
                    robot_state = RobotState::JUNCTION_DECISION;
                    break;

                case RobotState::JUNCTION_DECISION:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                             //case 1 


                            if (rawVal == 0){           // T junction
                            //     //backtrack 0.5 rot, rotate 90 degrees, line follow
                                robot_state = RobotState::BACK;
                                state_tracker = "BACK";
                            }

                            if (rawVal <= 256 && rawVal >= 254 ){         // Dead end
                                // rotate 180 degrees, 0.5 rot, line follow
                                state_tracker = "ROTATE";
                                robot_state = RobotState::ROTATE;
                                
                            }

                            // if (rawVal != 0 || rawVal != 255){              //nutin
                            //     robot_state = RobotState::FOLLOW;
                            //     state_tracker = "FOLLOW";
                            // }

                         }

                    break;

                case RobotState::BACK:
                    garbage = motor_M1.getRotation() - 0.5f;
                    garbage2 = motor_M2.getRotation() - 0.5f;
                    motor_M1.setRotation(garbage); 
                    motor_M2.setRotation(garbage2);

                    robot_state = RobotState::TURN_T;
                    state_tracker = "TURN_T";
                    break;

                case RobotState::TURN_T:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {

                    garbage = motor_M1.getRotation() + 0.9f;
                    garbage2 = motor_M2.getRotation() - 0.9f;
                    motor_M1.setRotation(garbage); 
                    motor_M2.setRotation(garbage2);
                    robot_state = RobotState::FOLLOW_TRANS;
                    state_tracker = "FOLLOW_TRANS";
                    }
                    break;

                case RobotState::ROTATE:

                    garbage = motor_M1.getRotation() - 2.0f;
                    garbage2 = motor_M2.getRotation() + 2.0f;
                    motor_M1.setRotation(garbage); 
                    motor_M2.setRotation(garbage2);
                    robot_state = RobotState::FORWARD_3;
                    state_tracker = "FORWARD_3";
                
                    break;

                case RobotState::FORWARD_3:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                         (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {

                    garbage = motor_M1.getRotation() - 1.0f;
                    garbage2 = motor_M2.getRotation() + 1.0f;
                    motor_M1.setRotation(garbage); 
                    motor_M2.setRotation(garbage2);
                    robot_state = RobotState::FOLLOW_TRANS;
                    state_tracker = "FOLLOW_TRANS";
                    }
                    break;

                // LINE BREAK CODE IF NEEDED - ADJUST ROTATION TIMER VALUE

                // case RobotState::LINE_BREAK:
                //     if (!rotation_started) {
                //         // Enable motion planner and set rotations
                //         motor_M1.enableMotionPlanner(true);
                //         motor_M2.enableMotionPlanner(true);
                //         motor_M1.setRotation(turns1);
                //         motor_M2.setRotation(turns2);

                //         // Start rotation timer
                //         rotation_timer.reset();
                //         rotation_timer.start();
                //         rotation_started = true;
                //     }

                //     // Check if rotation time has passed (assuming 3 seconds for 3 rotations)
                //     if (rotation_timer.read() >= 3.0f) {
                //         motor_M1.enableMotionPlanner(false);
                //         motor_M2.enableMotionPlanner(false);
                //         robot_state = RobotState::FOLLOW;
                //         state_tracker = "FOLLOW";
                //         rotation_started = false;
                //     }
                //     break;
                default:
                    return 0;
                    break;

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

        //printf("The value of angle is: %.2f\n", angle);
        
        //printf(" Rotations2: %.2f", motor_M2.getRotation());
        std::cout << " " << " Robot State: " << state_tracker << " \n";

        //printf("Target: %f", wheel_angle_forward(0));
        //rawVal = sensorBar.getRaw();
        
        printf(" getRaw Output: %u", rawVal);

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