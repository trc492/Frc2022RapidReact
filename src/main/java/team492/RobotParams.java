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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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
        // Inputs
        public static final boolean useXboxController           = true;
        public static final boolean useButtonPanels             = true;
        public static final boolean doOneStickDrive             = true;
        public static final boolean useGyroAssist               = false;
        // Sensors
        public static final boolean useNavX                     = true;
        public static final boolean usePdp                      = false;
        public static final boolean useExternalOdometry         = false;
        public static final boolean useVision                   = false;
        public static final boolean usePhotonVision             = true;
        public static final boolean useStreamCamera             = true;
        public static final boolean useWallAlignSensor          = false;
        public static final boolean useCANCoder                 = true;     //set to false to use Analog Encoder.
        // Subsystems
        public static final boolean useSubsystems               = true;
        public static final boolean useBalanceDrive             = true;
        public static final boolean useVacuum                   = false;
        // Miscellaneous
        public static final boolean useTraceLog                 = true;
        public static final boolean doAutoUpdates               = true;
        public static final boolean showSubsystemStatus         = true;
        public static final boolean showVisionStatus            = true;
        // Debug
        public static final boolean debugPowerConsumption       = false;
        public static final boolean debugDriveBase              = true;
        public static final boolean debugPurePursuitDrive       = false;
        public static final boolean debugPidDrive               = false;
        public static final boolean debugSubsystems             = true;
        public static final boolean debugVision                 = false;
        public static final boolean debugLoopTime               = false;
        public static final boolean debugShooter                = true;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "RapidReact_Robot";
    public static final String TEAM_FOLDER                      = "/home/lvuser/trc492";
    public static final double DASHBOARD_UPDATE_INTERVAL        = 0.5;

    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54.0*12.0;
    public static final double FIELD_WIDTH                      = 27.0*12.0;

    //
    // Robot dimensions in inches.
    //
    public static final double ROBOT_WIDTH                      = 34.5;     // Frame dimensions, including bumpers.
    public static final double ROBOT_LENGTH                     = 37.0;     // Frame dimensions, including bumpers.

    public static final double ROBOT_DRIVE_WIDTH                = 23.25;    // Required by swerve drive base.
    public static final double ROBOT_DRIVE_LENGTH               = 25.625;   // Required by swerve drive base.

    public static final double INTAKE_OFFSET                    = ROBOT_LENGTH / 2.0 + 8.0;

    //
    // Robot starting positions.
    //
    public static final TrcPose2D STARTPOS_1 = new TrcPose2D(  86.534,  -14.869,  -80.250);
    public static final TrcPose2D STARTPOS_2 = new TrcPose2D(  62.800,  -62.800,  -45.000);
    public static final TrcPose2D STARTPOS_3 = new TrcPose2D( -46.861,  -74.281,   32.250);
    public static final TrcPose2D[] startPos = {STARTPOS_1, STARTPOS_2, STARTPOS_3};

    public static final TrcPose2D STARTPOS_AUTO_5BALL = new TrcPose2D(  89.810,  -24.987,   88.5);

    //
    // Cargo positions.
    //
    // The following info is precisely measured from the Field CAD file. Do NOT modify.
    public static final TrcPose2D BALLPOS_1 = new TrcPose2D( 150.790,  -25.910, 0.0);
    public static final TrcPose2D BALLPOS_2 = new TrcPose2D(  88.303, -124.946, 0.0);
    public static final TrcPose2D BALLPOS_3 = new TrcPose2D( -81.643, -129.396, 0.0);
    public static final TrcPose2D BALLPOS_4 = new TrcPose2D(-149.227,  -33.767, 0.0);
    public static final TrcPose2D BALLPOS_5 = new TrcPose2D( -33.767,  149.227, 0.0);
    public static final TrcPose2D BALLPOS_6 = new TrcPose2D( 124.946,   88.303, 0.0);
    public static final TrcPose2D BALLPOS_7 = new TrcPose2D( 117.725, -282.080, 0.0);

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

    public static final int CANID_LEFT_LIDAR                    = 11;
    public static final int CANID_RIGHT_LIDAR                   = 12;

    public static final int CANID_LEFTFRONT_STEER               = 13;   // Orange: 40A
    public static final int CANID_RIGHTFRONT_STEER              = 14;   // Yellow: 40A
    public static final int CANID_LEFTBACK_STEER                = 15;   // Green: 40A
    public static final int CANID_RIGHTBACK_STEER               = 16;   // Blue: 40A

    public static final int CANID_SPARE                         = 23;   // Orange: 40A
    public static final int CANID_CONVEYOR                      = 24;   // Yellow: 40A
    public static final int CANID_CLIMBER                       = 25;   // Green: 40A
    public static final int CANID_INTAKE                        = 27;   // Purple: 40A
    public static final int CANID_SHOOTER_LOWER_FLYWHEEL        = 28;   // Gray: 40A
    public static final int CANID_SHOOTER_UPPER_FLYWHEEL        = 29;   // White: 40A

    public static final int CANID_PCM                           = 30;
    public static final int CANID_PDP                           = 31;

    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_LEFT_FRONT_DRIVE        = 11;   // Orange: 40A
    public static final int PDP_CHANNEL_RIGHT_FRONT_DRIVE       = 5;    // Yellow: 40A
    public static final int PDP_CHANNEL_LEFT_BACK_DRIVE         = 13;   // Green: 40A
    public static final int PDP_CHANNEL_RIGHT_BACK_DRIVE        = 3;    // Blue: 40A
    public static final int PDP_CHANNEL_LEFT_FRONT_STEER        = 10;   // Orange: 40A
    public static final int PDP_CHANNEL_RIGHT_FRONT_STEER       = 6;    // Yellow: 40A
    public static final int PDP_CHANNEL_LEFT_BACK_STEER         = 12;   // Green: 40A
    public static final int PDP_CHANNEL_RIGHT_BACK_STEER        = 4;    // Blue: 40A
    public static final int PDP_CHANNEL_SHOOTER_LOWER_FLYWHEEL  = 14;   // Gray: 40A
    public static final int PDP_CHANNEL_SHOOTER_UPPER_FLYWHEEL  = 1;    // White: 40A
    public static final int PDP_CHANNEL_CONVEYOR                = 16;   // Yellow: 40A
    public static final int PDP_CHANNEL_CLIMBER                 = 0;    // Green: 40A
    public static final int PDP_CHANNEL_INTAKE                  = 2;    // Purple: 40A
    public static final int PDP_CHANNEL_SPARE                   = 15;   // Orange: 40A
    public static final int PDP_CHANNEL_PCM                     = 9;    // 20A

    public static final int PDP_CHANNEL_MISC_LOW_CURRENT        = 18;   // 10A
    public static final int PDP_CHANNEL_VRM                     = 19;   // 10A

    public static final double BATTERY_NOMINAL_VOLTAGE          = 12.0;
    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;

    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 3;
    public static final int AIN_LEFTFRONT_STEER_ENCODER         = 0;
    public static final int AIN_RIGHTFRONT_STEER_ENCODER        = 1;
    public static final int AIN_LEFTBACK_STEER_ENCODER          = 2;
    public static final int AIN_RIGHTBACK_STEER_ENCODER         = 3;

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
    public static final int NUM_LEDS                            = 144;

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
    public static final int PNEUMATIC_TILTER_RETRACT            = 4;
    public static final int PNEUMATIC_TILTER_EXTEND             = 5;

    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    // public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // Vision subsystem.
    //
    public static final double CAMERA_Y_OFFSET                  = 2.5;  // Inches from the center of the robot
    public static final double CAMERA_X_OFFSET                  = 0.0;  // Inches from the center of the robot
    public static final double CAMERA_HEIGHT                    = 22.0; // Inches from the floor
    public static final double CAMERA_ANGLE                     = 36.0; // Degrees from horizontal
    public static final double CAMERA_DATA_TIMEOUT              = 0.5;  // 500ms
    public static final double VISION_HIGH_TARGET_HEIGHT        = 104.0;// Inches from the floor
    public static final double VISION_TARGET_RADIUS             = 53.375/2.0;// Inches
    public static final double VISION_DISTANCE_FUDGE_FACTOR     = 0.9;  // Compensate unknown discrepancy.
    public static final Transform3d CAMERA_TRANSFORM3D          = new Transform3d(
        new Translation3d(CAMERA_X_OFFSET, CAMERA_Y_OFFSET, CAMERA_HEIGHT),
        new Rotation3d(0.0, CAMERA_ANGLE, 0.0));

    //
    // Wall Alignment Sensor.
    //
    public static final double LIDAR_INTER_SENSOR_DIST          = 16.625; // Inches
    public static final double LIDAR_SENSOR_Y_OFFSET            = -3.5; // Inches

    public static final double WALL_ALIGN_LOW_THRESHOLD         = -0.5;
    public static final double WALL_ALIGN_HIGH_THRESHOLD        = 0.0;

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

    // 2022-03-10: Scale=9.250709670962152e-4, PID=0.02, 0.0, 0.005, Tol=2.0
    // 2022-03-30: Scale=9.072106867127145344367826764411e-4, PID=0.02, 0.0, 0.0
    public static final double SWERVE_INCHES_PER_COUNT          = 9.072106867127145344367826764411e-4;
    public static final double SWERVE_KP                        = 0.02;
    public static final double SWERVE_KI                        = 0.0;
    public static final double SWERVE_KD                        = 0.0;
    public static final double SWERVE_KF                        = 0.0;
    public static final double SWERVE_TOLERANCE                 = 2.0;

    // 2022-03-01: PID=0.007, 0.0, 0.0007, Tol=2.0
    // 2022-03-30: PID=0.012, 0.0, 0.0, Tol=2.0
    public static final double GYRO_TURN_KP                     = 0.012;//0.0012;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.0;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;
    // TODO: MAKE NEW GYRO_ALIGN VALUES FOR AUTO AND TELEOP
                                                                            //these work for 5 ball auto
    public static final double GYRO_ALIGN_KP                    = 0.015;//0.015;    //0.01;
    public static final double GYRO_ALIGN_KI                    = 0.0;
    public static final double GYRO_ALIGN_KD                    = 0.0012;   //0.0012;
    public static final double GYRO_ALIGN_TOLERANCE             = 1.5;      //2.0;
    public static final double GYRO_ALIGN_SETTLING_TIME         = 0.2;
    public static final double GYRO_ALIGN_STEADY_STATE_ERROR    = 2.5 ;     //2.5;
    public static final double GYRO_ALIGN_ERRRATE_THRESHOLD     = 1.5;

    public static final double GYRO_PITCH_KP                    = 0.0095;
    public static final double GYRO_PITCH_KI                    = 0.0;
    public static final double GYRO_PITCH_KD                    = 0.001;
    public static final double GYRO_PITCH_KF                    = 0.0;
    public static final double GYRO_PITCH_TOLERANCE             = 2.0;
    public static final double GYRO_PITCH_SETTLING_TIME         = 0.2;
    public static final double GYRO_PITCH_MAX_PID_POWER         = 0.2;
    public static final double GYRO_PITCH_PID_RAMP_RATE         = 0.2;

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
    public static final double CANCODER_CPR                     = 4096.0;
    public static final double FALCON_CPR                       = 2048.0;
    public static final double STEER_ENCODER_SCALE              = FALCON_CPR / CANCODER_CPR;
    public static final double STEER_GEAR_RATIO                 = (24.0/12.0) * (72.0/14.0);

    public static final double STEER_DEGREES_PER_TICK           = 360.0 / CANCODER_CPR;
    public static final double STEER_DEGREES_PER_COUNT          = 360.0 / (FALCON_CPR*STEER_GEAR_RATIO);
    public static final double STEER_MAX_REQ_VEL                = 1000.0;   // deg/sec. max commanded velocity, not necessarily max vel
    public static final double STEER_MAX_ACCEL                  = 5000.0;   // deg/sec^2
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    public static final double STEER_MAX_VEL                    = ((18700.0 * 0.81 / 56.67) / 60.0) * 360.0;        // deg/sec
    public static final double STEER_MAX_VEL_COUNT_PER_100MS    = (STEER_MAX_VEL / STEER_DEGREES_PER_COUNT) / 10.0; // count/100ms

    // order is lf, rf, lr, rr
    // steerzeros.txt: 3974, 3748, 1192, 3487
    public static final int[] STEER_ZEROS                       = new int[] {2167, 3756, 1194, 3485};//{ 3971, 3743, 1189, 3490 };  // this is a backup if the zeros file isn't found

    public static final TrcPidController.PidCoefficients magicSteerCoeff =
        new TrcPidController.PidCoefficients(2.0, 0.01, 0.0, 1023.0 / STEER_MAX_VEL_COUNT_PER_100MS, 5.0 / STEER_DEGREES_PER_COUNT);
    public static final double STEER_KP                         = 1.0;  //0.8;//0.8;//1.2
    public static final double STEER_KI                         = 0.0;
    public static final double STEER_KD                         = 14.0; //2.0;//2.0;//16.0
    public static final double STEER_KF                         = 0.0;
    public static final double STEER_CAL_POWER                  = 0.1;
    public static final TrcPidController.PidCoefficients steerCoeffs =
        new TrcPidController.PidCoefficients(STEER_KP, STEER_KI, STEER_KD, STEER_KF);
    //current kp and kd constants are tuned off of the falcon's integrated sensor, NOT off cancoder (yet)
    public static final double PPD_FOLLOWING_DISTANCE           = 12.0;
    public static final double PPD_POS_TOLERANCE                = 2.0;
    public static final double PPD_TURN_TOLERANCE               = 3.0;
    public static final double PPD_MOVE_DEF_OUTPUT_LIMIT        = 0.5;
    public static final double PPD_ROT_DEF_OUTPUT_LIMIT         = 0.3;

    //
    // Other subsystems.
    //

    // Shooter subsystem.
    public static final double FLYWHEEL_KP                      = 0.05;
    public static final double FLYWHEEL_KI                      = 1e-4;
    public static final double FLYWHEEL_KD                      = 5.0;
    public static final double FLYWHEEL_KF                      = 0.0479;
    public static final double FLYWHEEL_IZONE                   = 2000.0;
    public static final double FLYWHEEL_TOLERANCE               = 50.0;     // in RPM //used to be 100
    public static final double FLYWHEEL_SETTLING_TIME           = 0.35;      // 0.35; //Try .01? worked similarly
    // TODO Settling time should be 0.5 for Autonomous, 0.35 for Teleop
    public static final double FLYWHEEL_UPPER2LOWER_VALUE_RATIO = 1.0;
    public static final double FLYWHEEL_UPDATE_INTERVAL         = 0.1;      // in seconds
    public static final int FLYWHEEL_MAX_RPM                    = 6400;
    public static final double FLYWHEEL_ENCODER_PPR             = 2048;     // Falcon integrated encoder: 2048 CPR
    public static final double FLYWHEEL_GEAR_RATIO              = 1.0;
    public static final double FLYWHEEL_MAX_VEL                 =
        FLYWHEEL_MAX_RPM / 60.0 * FLYWHEEL_GEAR_RATIO * FLYWHEEL_ENCODER_PPR;//Tested to be about 220000 SU/s
    public static final TrcPidController.PidCoefficients SHOOTER_COEFFS =
        new TrcPidController.PidCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KF, FLYWHEEL_IZONE);
    public static final double SHOT_COMPLETION_DELAY            = 0.3;      // in seconds

    public static final double TILTER_CLOSE_ANGLE               = 43.0;
    public static final double TILTER_FAR_ANGLE                 = 31.0;

    // Intake subsystem.
    public static final double INTAKE_PICKUP_POWER              = 0.75;
    public static final double INTAKE_SPITOUT_POWER             = -0.75;
    public static final double INTAKE_PICKUP_DELAY              = 0.5;      // in seconds.

    // Conveyor subsystem.
    public static final boolean CONVEYOR_MOTOR_INVERTED         = false;
    public static final boolean CONVEYOR_ENTRANCE_SENSOR_INVERTED=true;
    public static final boolean CONVEYOR_EXIT_SENSOR_INVERTED   = true;
    public static final double CONVEYOR_MOVE_POWER              = 0.3;

    // Climber subsystem.
    public static final double CLIMBER_KP                       = 0.2;      //0.06;
    public static final double CLIMBER_KI                       = 0.0;
    public static final double CLIMBER_KD                       = 0.0;      //0.005;
    public static final double CLIMBER_TOLERANCE                = 1.0;
    public static final int CLIMBER_ENCODER_PPR                 = 4096;
    public static final double CLIMBER_INCHES_PER_COUNT         = 1.392027924751009e-4;
    public static final double CLIMBER_OFFSET                   = 29.6785;
    public static final double CLIMBER_CAL_POWER                = 0.5;
    public static final boolean CLIMBER_MOTOR_INVERTED          = true;
    public static final double CLIMBER_MIN_POS                  = 20.0;
    public static final double CLIMBER_MAX_POS                  = 65.0;

}   //class RobotParams
