/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;

/**
 * This class contains parameters and preferences related to all robot operations.
 */
public class RobotParams
{
    //
    // Robot preferences.
    //
    public static class Preferences
    {
        public static final boolean useSubsystems               = true;
        public static final boolean useExternalOdometry         = false;
        public static final boolean useXboxController           = false;
        public static final boolean useButtonPanels             = true;
        public static final boolean usePdp                      = true;
        public static final boolean useTraceLog                 = true;
        public static final boolean useNavX                     = true;
        public static final boolean useGyroAssist               = false;
        public static final boolean useVision                   = false;
        public static final boolean useStreamCamera             = true;
        public static final boolean doAutoUpdates               = true;
        public static final boolean timDrive                    = true;
        public static final boolean showSubsystemStatus         = true;

        public static final boolean debugPowerConsumption       = false;
        public static final boolean debugPidDrive               = false;
        public static final boolean debugDriveBase              = false;
        public static final boolean debugVision                 = false;
        public static final boolean debugLoopTime               = false;
    }   //class Preferences

    public static final String GAME_NAME                        = "RapidReact";
    public static final String TEAM_FOLDER                      = "/home/lvuser/trc492";
    public static final double DASHBOARD_UPDATE_INTERVAL        = 0.1;

    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54.0*12.0;
    public static final double FIELD_WIDTH                      = 27.0*12.0;

    //
    // Robot dimensions in inches.
    //
    public static final double ROBOT_WIDTH                      = 34.5;     // Frame dimensions, including bumpers.
    public static final double ROBOT_LENGTH                     = 36.5;     // Frame dimensions, including bumpers.

    public static final double ROBOT_DRIVE_WIDTH                = 23.25;    // Required by swerve drive base.
    public static final double ROBOT_DRIVE_LENGTH               = 25.625;   // Required by swerve drive base.

    //
    // Autonomous constants.
    //
    public static final TrcPose2D RED_START_POS_2_BALL = new TrcPose2D(84.6, -43.0, 134.0);
    public static final TrcPose2D BLUE_START_POS_2_BALL = new TrcPose2D(-85.7, 32.6, 250.0);

    //5 ball auto start point
    public static final TrcPose2D RED_START_POS_5_BALL = new TrcPose2D(26, 89.3, 358.0);
    public static final TrcPose2D BLUE_START_POS_5_BALL = new TrcPose2D(-25.3, -88.0, 177.0 );

    //path points for each different auto path
    //first point: robot picking up the ball around the tarmacs
    //second point: robot moving into shooter position 
    //2 BALL AUTO PATH 
    public static final TrcPose2D[] BLUE_2_BALL_PATH = new TrcPose2D[]
    {
        new TrcPose2D(-119, 73.7, 47),
        new TrcPose2D(-119, 73.7, 238)
    };

    public static final TrcPose2D[] RED_2_BALL_PATH = new TrcPose2D[]
    {
        new TrcPose2D(128.8, -79.8, 225),
        new TrcPose2D(109, -43.2, 68)
    };

    //5 BALL AUTO PATH
    //point 1: ball near tarmac and center of field y axis
    //point 2: other ball near tarmac and first ball
    //point 3: ball near human player, also where it will wait for human player ball input
    public static final TrcPose2D[] BLUE_5_BALL_PATH = new TrcPose2D[]
    {
        new TrcPose2D(-25.9, -151.4, 177.0),
        new TrcPose2D(-124.88, -88.3, 302),
        new TrcPose2D(-282.0, -117.7, 259.5),
    };

    public static final TrcPose2D[] RED_5_BALL_PATH = new TrcPose2D[]
    {
        new TrcPose2D(26.5, 108.5, 358.0),
        new TrcPose2D(125.75, 88.3, 302.0),
        new TrcPose2D(283.5, 117, 259.5),
    };
    //
    // Joystick ports.
    //
    public static final int JSPORT_DRIVER_LEFTSTICK             = 0;
    public static final int JSPORT_DRIVER_RIGHTSTICK            = 1;
    public static final int JSPORT_OPERATORSTICK                = 2;
    public static final int JSPORT_BUTTON_PANEL                 = 3;
    public static final int JSPORT_SWITCH_PANEL                 = 4;
    public static final int XBOX_DRIVERCONTROLLER               = 5;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONT_DRIVE               = 3;    // Orange: 40A
    public static final int CANID_RIGHTFRONT_DRIVE              = 4;    // Yellow: 40A
    public static final int CANID_LEFTBACK_DRIVE                = 5;    // Green: 40A
    public static final int CANID_RIGHTBACK_DRIVE               = 6;    // Blue: 40A

    public static final int CANID_LEFTFRONT_STEER_ENCODER       = 7;    // Orange
    public static final int CANID_RIGHTFRONT_STEER_ENCODER      = 8;    // Yellow
    public static final int CANID_LEFTBACK_STEER_ENCODER        = 9;    // Green
    public static final int CANID_RIGHTBACK_STEER_ENCODER       = 10;   // Blue

    public static final int CANID_LEFTFRONT_STEER               = 13;   // Orange: 40A
    public static final int CANID_RIGHTFRONT_STEER              = 14;   // Yellow: 40A
    public static final int CANID_LEFTBACK_STEER                = 15;   // Green: 40A
    public static final int CANID_RIGHTBACK_STEER               = 16;   // Blue: 40A

    public static final int CANID_CONVEYOR                      = 23;   // Orange: 40A
    public static final int CANID_SHOOTER_TILTER                = 24;   // Yellow: ??
    public static final int CANID_CLIMBER                       = 25;   // Green: 40A
    public static final int CANID_INTAKE                        = 27;   // Purple: 40A
    public static final int CANID_SHOOTER_LOWER_FLYWHEEL        = 28;   // Gray: 40A
    public static final int CANID_SHOOTER_UPPER_FLYWHEEL        = 29;   // White: 40A

    public static final int CANID_PCM                           = 30;
    public static final int CANID_PDP                           = 31;

    //
    // PDP Channels. (TODO: Need to remap these.)
    //
    public static final int PDP_CHANNEL_LEFT_FRONT_DRIVE        = 11;   // Orange: 40A`
    public static final int PDP_CHANNEL_RIGHT_FRONT_DRIVE       = 5;    // Yellow: 40A
    public static final int PDP_CHANNEL_LEFT_BACK_DRIVE         = 13;   // Green: 40A
    public static final int PDP_CHANNEL_RIGHT_BACK_DRIVE        = 3;    // Blue: 40A
    public static final int PDP_CHANNEL_LEFT_FRONT_STEER        = 10;   // Orange: 40A
    public static final int PDP_CHANNEL_RIGHT_FRONT_STEER       = 6;    // Yellow: 40A
    public static final int PDP_CHANNEL_LEFT_BACK_STEER         = 12;   // Green: 40A
    public static final int PDP_CHANNEL_RIGHT_BACK_STEER        = 4;    // Blue: 40A
    public static final int PDP_CHANNEL_SHOOTER_LOWER_FLYWHEEL  = 14;   // Gray: 40A
    public static final int PDP_CHANNEL_SHOOTER_UPPER_FLYWHEEL  = 1;    // White: 40A
    public static final int PDP_CHANNEL_SHOOTER_TILTER          = 16;   // Yellow: 40A
    public static final int PDP_CHANNEL_CLIMBER                 = 0;    // Green: 40A
    public static final int PDP_CHANNEL_INTAKE                  = 2;    // Purple: 40A
    public static final int PDP_CHANNEL_CONVEYOR                = 15;   // Orange: 40A
    public static final int PDP_CHANNEL_PCM                     = 9;    // 20A

    public static final int PDP_CHANNEL_MISC_LOW_CURRENT        = 18;   // 10A
    public static final int PDP_CHANNEL_VRM                     = 19;   // 10A

    public static final double BATTERY_NOMINAL_VOLTAGE          = 12.0;
    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;

    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 0;

    //
    // Digital Input/Output ports.
    //
    public static final int DIO_CONVEYOR_ENTRANCE_SENSOR        = 6;
    public static final int DIO_CONVEYOR_EXIT_SENSOR            = 7;
    public static final int DIO_CLIMBER_LOWER_LIMIT_SWITCH      = 8;
    public static final int DIO_TILTER_LOWER_LIMIT_SWITCH       = 9;

    //
    // PWM channels.
    //
    public static final int PWM_CHANNEL_LED                     = 0;
    public static final int NUM_LEDS                            = 60;   // TODO: needs updating

    //
    // Relay channels.
    //

    //
    // Solenoid channels.
    //
    public static final int PNEUMATIC_INTAKE_RETRACT            = 0;
    public static final int PNEUMATIC_INTAKE_EXTEND             = 1;
    public static final int PNEUMATIC_CLIMBER_RETRACT           = 2;
    public static final int PNEUMATIC_CLIMBER_EXTEND            = 3;

    //
    // Vision subsystem.
    // TODO: Needs updating
    //
    public static final double CAMERA_Y_OFFSET                  = 12;   // in from pivot of arm + is forward
    public static final double CAMERA_Y_OFFSET_TO_PIVOT         = 26;
    public static final double CAMERA_X_OFFSET                  = 0;    //Inches from pivot of arm to center of camera, + = right
    public static final double CAMERA_DATA_TIMEOUT              = 0.5;  //500ms
    public static final double CAMERA_CENTERED_THRESHOLD        = 2;    // +- 2 inches in x axis
    public static final double VISION_HIGH_TARGET_HEIGHT        = 89.75;//TrcUtil.average(81.25, HIGH_TARGET_HEIGHT);
    public static final double HIGH_TARGET_HEIGHT               = 98.25;
    public static final double HIGH_VISION_TARGET_HEIGHT        = 89.75;//TrcUtil.average(81.25, HIGH_TARGET_HEIGHT);
    public static final double PIVOT_HEIGHT                     = 23.5; // in from ground
    public static final double SHOOTER_BARREL_LENGTH            = 30; // inches

    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    // public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // DriveBase subsystem.
    //
    public static final double WCD_INCHES_PER_COUNT             = 2.2421;
    public static final double WCD_KP                           = 0.011;
    public static final double WCD_KI                           = 0.0;
    public static final double WCD_KD                           = 0.0013;
    public static final double WCD_KF                           = 0.0;
    public static final double WCD_TOLERANCE                    = 2.0;

    public static final double MECANUM_X_INCHES_PER_COUNT       = 2.2421;
    public static final double MECANUM_X_KP                     = 0.011;
    public static final double MECANUM_X_KI                     = 0.0;
    public static final double MECANUM_X_KD                     = 0.0013;
    public static final double MECANUM_X_KF                     = 0.0;
    public static final double MECANUM_X_TOLERANCE              = 2.0;

    public static final double MECANUM_Y_INCHES_PER_COUNT       = 2.2421;
    public static final double MECANUM_Y_KP                     = 0.011;
    public static final double MECANUM_Y_KI                     = 0.0;
    public static final double MECANUM_Y_KD                     = 0.0013;
    public static final double MECANUM_Y_KF                     = 0.0;
    public static final double MECANUM_Y_TOLERANCE              = 2.0;

    // 2022-03-10: Scale=9.250709670962152e-4, PID=0.02, 0.0, 0.005
    public static final double SWERVE_INCHES_PER_COUNT          = 9.250709670962152e-4;
    public static final double SWERVE_KP                        = 0.02;
    public static final double SWERVE_KI                        = 0.0;
    public static final double SWERVE_KD                        = 0.005;
    public static final double SWERVE_KF                        = 0.0;
    public static final double SWERVE_TOLERANCE                 = 2.0;

    public static final double GYRO_TURN_KP                     = 0.007;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.0007;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;
    public static final double GYRO_ASSIST_TURN_GAIN            = 0.1;

    public static final double ROBOT_MAX_VELOCITY               = 180.0;
    public static final double ROBOT_MAX_ACCELERATION           = 2100.0;
    public static final double ROBOT_MAX_TURN_RATE              = 1000.0;
    public static final double ROBOT_VEL_KP                     = 0.0;
    public static final double ROBOT_VEL_KI                     = 0.0;
    public static final double ROBOT_VEL_KD                     = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final double ROBOT_VEL_KF                     = 1.0 / ROBOT_MAX_VELOCITY;

    public static final double DRIVE_SLOW_SCALE                 = 0.5;
    public static final double TURN_SLOW_SCALE                  = 0.3;
    public static final double DRIVE_MEDIUM_SCALE               = 0.75;
    public static final double TURN_MEDIUM_SCALE                = 0.6;
    public static final double DRIVE_FAST_SCALE                 = 1.0;
    public static final double TURN_FAST_SCALE                  = 0.8;

    public static final double DRIVE_MAX_XPID_POWER             = 0.5;
    public static final double DRIVE_MAX_YPID_POWER             = 0.6;
    public static final double DRIVE_MAX_TURNPID_POWER          = 1.0;

    public static final double DRIVE_MAX_XPID_RAMP_RATE         = 0.5;
    public static final double DRIVE_MAX_YPID_RAMP_RATE         = 0.6;
    public static final double DRIVE_MAX_TURNPID_RAMP_RATE      = 1.0;

    public static final double DRIVE_RAMP_RATE                  = 0.2;

    // Applicable only for Swerve Drive.
    public static final int STEER_ENCODER_PPR                   = 4096;
    public static final double STEER_DEGREES_PER_TICK           = 360.0 / STEER_ENCODER_PPR;
    public static final double STEER_MAX_REQ_VEL                = 1000.0;   // deg/sec. max commanded velocity, not necessarily max vel
    public static final double STEER_MAX_ACCEL                  = 5000.0;   // deg/sec^2
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    public static final double STEER_MAX_VEL                    = ((18700.0 * 0.81 / 56.67) / 60.0) * 360.0;        // deg/sec
    public static final double STEER_MAX_VEL_TICKS_PER_100MS    = (STEER_MAX_VEL / STEER_DEGREES_PER_TICK) / 10.0;  // ticks/100ms

    // order is lf, rf, lr, rr
    // steerzeros.txt: 3974, 3748, 1192, 3487
    public static final int[] STEER_ZEROS                       = new int[]{ 3842, 3748, 1185, 3476 };  // this is a backup if the zeros file isn't found

    public static final TrcPidController.PidCoefficients magicSteerCoeff =
        new TrcPidController.PidCoefficients(2.0, 0.01, 0.0, 1023.0 / STEER_MAX_VEL_TICKS_PER_100MS, 5.0 / STEER_DEGREES_PER_TICK);
    public static final double STEER_KP                         = 0.9;
    public static final double STEER_KI                         = 0.0;
    public static final double STEER_KD                         = 32.0;
    public static final double STEER_KF                         = 0.0;
    public static final double STEER_CAL_POWER                  = 0.1;
    public static final TrcPidController.PidCoefficients steerCoeffs =
        new TrcPidController.PidCoefficients(STEER_KP, STEER_KI, STEER_KD, STEER_KF);
    //current kp and kd constants are tuned off of the falcon's integrated sensor, NOT off cancoder (yet)
    public static final double PPD_FOLLOWING_DISTANCE           = 10.0;
    public static final double PPD_POS_TOLERANCE                = 2.0;
    public static final double PPD_TURN_TOLERANCE               = 2.0;
    public static final double PPD_MOVE_OUTPUT_LIMIT            = 0.6;

    //
    // Other subsystems.
    //

    // Shooter subsystem.
    public static final double FLYWHEEL_KP                      = 0.05;
    public static final double FLYWHEEL_KI                      = 1e-4;
    public static final double FLYWHEEL_KD                      = 5.0;
    public static final double FLYWHEEL_KF                      = 0.0479;
    public static final double FLYWHEEL_IZONE                   = 2000.0;
    public static final double FLYWHEEL_TOLERANCE               = 100.0;    // in RPM
    public static final double FLYWHEEL_SETTLING_TIME           = 1.0;      // in seconds
    public static final double FLYWHEEL_UPPER2LOWER_VALUE_RATIO = 1.0;
    public static final int FLYWHEEL_MAX_RPM                    = 6400;
    public static final double FLYWHEEL_ENCODER_PPR             = 2048;     //Falcon integrated encoder: 2048 CPR
    public static final double FLYWHEEL_GEAR_RATIO              = 1.0;
    public static final double FLYWHEEL_MAX_VEL                 =
        FLYWHEEL_MAX_RPM / 60.0 * FLYWHEEL_GEAR_RATIO * FLYWHEEL_ENCODER_PPR; //Tested to be about 220000 SU/s
    public static final TrcPidController.PidCoefficients SHOOTER_COEFFS =
        new TrcPidController.PidCoefficients(0.05, 1e-4, 5, 0.0479, 2000);

    public static final double TILTER_KP                        = 1.0;
    public static final double TILTER_KI                        = 0.0;
    public static final double TILTER_KD                        = 0.0;
    public static final double TILTER_TOLERANCE                 = 0.1;
    public static final int TILTER_ENCODER_PPR                  = 4096;
    public static final double TILTER_OFFSET                    = 16.0; // Abs Enc: 1384
    public static final int TILTER_ZERO                         = 1384;
    // Tilter Upper Limit: Enc=1070, Deg=45
    public static final boolean TILTER_MOTOR_INVERTED           = false;
    public static final double TILTER_MIN_POS                   = TILTER_OFFSET;
    public static final double TILTER_MAX_POS                   = 40.0;
    public static final double TILTER_DEG_PER_COUNT             = Math.abs((45.0 - TILTER_OFFSET)/(1070 - TILTER_ZERO));
    public static final double TILTER_CAL_POWER                 = 0.6;
    // public static final double TILTER_STALL_MIN_POWER           = 0.2;
    // public static final double TILTER_STALL_TOLERANCE           = 0.2;
    // public static final double TILTER_STALL_TIMEOUT             = 0.5;
    // public static final double TILTER_RESET_TIMEOUT             = 0.5;

    // Intake subsystem.
    public static final double INTAKE_PICKUP_POWER              = 0.6;
    public static final double INTAKE_SPITOUT_POWER             = -0.6;
    public static final double INTAKE_PICKUP_DELAY              = 0.5;  // in seconds.

    // Conveyor subsystem.
    public static final boolean CONVEYOR_MOTOR_INVERTED         = false;
    public static final boolean CONVEYOR_ENTRANCE_SENSOR_INVERTED=false;
    public static final boolean CONVEYOR_EXIT_SENSOR_INVERTED   = false;
    public static final double CONVEYOR_MOVE_POWER              = 0.5;

    // Climber subsystem.
    public static final double CLIMBER_KP                       = 0.06;
    public static final double CLIMBER_KI                       = 0.0;
    public static final double CLIMBER_KD                       = 0.005;
    public static final double CLIMBER_TOLERANCE                = 1.0;
    public static final int CLIMBER_ENCODER_PPR                 = 4096;
    public static final double CLIMBER_INCHES_PER_COUNT         = 1.392027924751009e-4;
    public static final double CLIMBER_OFFSET                   = 29.6785;
    public static final double CLIMBER_CAL_POWER                = 0.3;
    // public static final double CLIMBER_STALL_MIN_POWER          = CLIMBER_CAL_POWER - 0.1;
    // public static final double CLIMBER_STALL_TOLERANCE          = 0.1;
    // public static final double CLIMBER_STALL_TIMEOUT            = 0.1;
    // public static final double CLIMBER_RESET_TIMEOUT            = 0.5;
    public static final boolean CLIMBER_MOTOR_INVERTED          = true;
    public static final double CLIMBER_MIN_POS                  = 20.0;
    public static final double CLIMBER_MAX_POS                  = 65.0;

}   //class RobotParams