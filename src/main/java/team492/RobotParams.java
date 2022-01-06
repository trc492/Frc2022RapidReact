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

public class RobotParams
{
    //
    // Robot preferences.
    //
    static class Preferences
    {
        static final boolean isPracticeBot = false;
        static final boolean useExternalOdometry = false;
        static final boolean useXboxController = true;
        static final boolean useTraceLog = true;
        static final boolean useNavX = true;
        static final boolean useGyroAssist = false;
        static final boolean useVision = true;
        static final boolean useStreamCamera = true;
        static final boolean doAutoUpdates = false;

        static final boolean debugPowerConsumption = false;
        static final boolean debugDriveBase = false;
        static final boolean debugPidDrive = false;
        static final boolean debugSubsystems = false;
        static final boolean debugVision = false;
        static final boolean debugLoopTime = true;
    }   //class Preferences

    static enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE
    }   //enum DriveMode

    static final String GAME_NAME = "RapidReact";
    static final double DASHBOARD_UPDATE_INTERVAL = 0.1;

    //
    // Field dimensions in inches.
    //
    static final double FIELD_LENGTH                            = 52*12 + 5.25;
    static final double FIELD_WIDTH                             = 26*12 + 11.25;

    //
    // Robot dimensions in inches.
    //
    static final double ROBOT_WIDTH                             = 36;
    static final double ROBOT_LENGTH                            = 37;

    static final double ROBOT_DRIVE_WIDTH                       = 21;
    static final double ROBOT_DRIVE_LENGTH                      = 21;

    //
    // Joystick ports.
    //
    static final int XBOX_DRIVERCONTROLLER                      = 0;
    static final int JSPORT_OPERATORSTICK                       = 1;
    static final int JSPORT_BUTTON_PANEL                        = 2;
    static final int JSPORT_SWITCH_PANEL                        = 3;

    //
    // CAN IDs.
    //
    static final int CANID_LEFTFRONT_DRIVE                      = 3;    // 40A: Orange
    static final int CANID_RIGHTFRONT_DRIVE                     = 4;    // 40A: Yellow
    static final int CANID_LEFTBACK_DRIVE                       = 5;    // 40A: Green
    static final int CANID_RIGHTBACK_DRIVE                      = 6;    // 40A: Blue
    static final int CANID_LEFTFRONT_STEER                      = 23;   // 40A: Orange
    static final int CANID_RIGHTFRONT_STEER                     = 24;   // 40A: Yellow
    static final int CANID_LEFTBACK_STEER                       = 25;   // 40A: Green
    static final int CANID_RIGHTBACK_STEER                      = 26;   // 40A: Blue

    static final int CANID_PDP                                  = 16;
    static final int CANID_PCM                                  = 17;

    //
    // PDP Channels.
    //
    static final int PDP_CHANNEL_LEFT_FRONT_WHEEL               = 3;
    static final int PDP_CHANNEL_RIGHT_FRONT_WHEEL              = 0;
    static final int PDP_CHANNEL_LEFT_BACK_WHEEL                = 12;
    static final int PDP_CHANNEL_RIGHT_BACK_WHEEL               = 15;
    static final double BATTERY_NOMINAL_VOLTAGE                 = 12.0;
    static final double BATTERY_CAPACITY_WATT_HOUR              = 18.0*12.0;

    //
    // Analog Input ports.
    //
    static final int AIN_PRESSURE_SENSOR                        = 0;

    //
    // Digital Input/Output ports.
    //

    //
    // Relay channels.
    //

    //
    // Solenoid channels.
    //

    //
    // Vision subsystem.
    //

    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    // public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // DriveBase subsystem.
    //
    static final double ENCODER_INCHES_PER_COUNT                = 2.2421;
    static final double ENCODER_KP                              = 0.011;
    static final double ENCODER_KI                              = 0.0;
    static final double ENCODER_KD                              = 0.0013;
    static final double ENCODER_KF                              = 0.0;
    static final double ENCODER_TOLERANCE                       = 2.0;

    static final double GYRO_TURN_KP                            = 0.013;
    static final double GYRO_TURN_KI                            = 0.0;
    static final double GYRO_TURN_KD                            = 0.0;
    static final double GYRO_TURN_KF                            = 0.0;
    static final double GYRO_TURN_TOLERANCE                     = 1.0;

    static final double ROBOT_MAX_VELOCITY                      = 300.0;
    static final double ROBOT_MAX_ACCELERATION                  = 200.0;
    static final double ROBOT_VEL_KP                            = 0.0;
    static final double ROBOT_VEL_KI                            = 0.0;
    static final double ROBOT_VEL_KD                            = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    static final double ROBOT_VEL_KF                            = 1.0 / ROBOT_MAX_VELOCITY;

    static final double DRIVE_MAX_XPID_POWER                    = 0.5;
    static final double DRIVE_MAX_YPID_POWER                    = 0.6;
    static final double DRIVE_MAX_TURNPID_POWER                 = 1.0;

    static final double DRIVE_MAX_XPID_RAMP_RATE                = 0.5;
    static final double DRIVE_MAX_YPID_RAMP_RATE                = 0.6;
    static final double DRIVE_MAX_TURNPID_RAMP_RATE             = 1.0;

    static final double DRIVE_RAMP_RATE                         = 0.2;

    static final double STEER_DEGREES_PER_TICK                  = 360.0 / 4096.0;
    static final double STEER_MAX_REQ_VEL                       = 1000.0; // deg/sec. max commanded velocity, not necessarily max vel
    static final double STEER_MAX_ACCEL                         = 5000; // deg/sec^2
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    static final double STEER_MAX_VEL                           = ((18700 * 0.81 / 56.67) / 60.0) * 360.0; // deg/sec
    static final double STEER_MAX_VEL_TICKS_PER_100MS           = (STEER_MAX_VEL / STEER_DEGREES_PER_TICK) / 10.0; // ticks/100ms

    // order is lf, rf, lr, rr
    static final int[] STEER_ZEROS                              = new int[]{ 3551, 479, 3656, 1270 }; // this is a backup if the zeros file isn't found
    static final TrcPidController.PidCoefficients magicSteerCoeff=
        new TrcPidController.PidCoefficients(2.0, 0.01, 0, 1023.0 / STEER_MAX_VEL_TICKS_PER_100MS, 5.0 / STEER_DEGREES_PER_TICK);


    static final double PPD_FOLLOWING_DISTANCE                  = 10.0;
    static final double PPD_POS_TOLERANCE                       = 2.0;
    static final double PPD_TURN_TOLERANCE                      = 2.0;
    static final double PPD_MOVE_OUTPUT_LIMIT                   = 0.6;

    //
    // Other subsystems.
    //

}   // class RobotParams
