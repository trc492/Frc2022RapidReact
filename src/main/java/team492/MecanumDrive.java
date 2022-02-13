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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcPdp;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class MecanumDrive
{
    //
    // Global objects.
    //

    //
    // Sensors.
    //
    public final FrcAHRSGyro gyro;
    //
    // Drive motors.
    //
    public final FrcCANTalon lfDriveMotor, lbDriveMotor, rfDriveMotor, rbDriveMotor;
    //
    // Drive Base.
    //
    public final TrcMecanumDriveBase driveBase;

    public final TrcPidController encoderXPidCtrl;
    public final TrcPidController encoderYPidCtrl;
    public final TrcPidController gyroTurnPidCtrl;
    public final TrcPidDrive pidDrive;
    public final TrcPurePursuitDrive purePursuitDrive;
    //
    // Coefficients for PID controllers.
    //
    public final TrcPidController.PidCoefficients xPosPidCoeff;
    public final TrcPidController.PidCoefficients yPosPidCoeff;
    public final TrcPidController.PidCoefficients turnPidCoeff;
    public final TrcPidController.PidCoefficients velPidCoeff;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public MecanumDrive(Robot robot)
    {
        gyro = RobotParams.Preferences.useNavX ? new FrcAHRSGyro("NavX", SPI.Port.kMXP) : null;

        lfDriveMotor = new FrcCANTalon("lfDriveMotor", RobotParams.CANID_LEFTFRONT_DRIVE);
        lbDriveMotor = new FrcCANTalon("lbDriveMotor", RobotParams.CANID_LEFTBACK_DRIVE);
        rfDriveMotor = new FrcCANTalon("rfDriveMotor", RobotParams.CANID_RIGHTFRONT_DRIVE);
        rbDriveMotor = new FrcCANTalon("rbDriveMotor", RobotParams.CANID_RIGHTBACK_DRIVE);

        lfDriveMotor.setBrakeModeEnabled(true);
        lbDriveMotor.setBrakeModeEnabled(true);
        rfDriveMotor.setBrakeModeEnabled(true);
        rbDriveMotor.setBrakeModeEnabled(true);

        rfDriveMotor.setInverted(true);
        rbDriveMotor.setInverted(true);

        driveBase = new TrcMecanumDriveBase(lfDriveMotor, lbDriveMotor, rfDriveMotor, rbDriveMotor, gyro);
        driveBase.setOdometryScales(RobotParams.MECANUM_X_INCHES_PER_COUNT, RobotParams.MECANUM_Y_INCHES_PER_COUNT);

        robot.pdp.registerEnergyUsed(
            new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_FRONT_DRIVE, "lfDriveMotor"),
            new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_BACK_DRIVE, "lbDriveMotor"),
            new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_FRONT_DRIVE, "rfDriveMotor"),
            new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_BACK_DRIVE, "rbDriveMotor"));

        // if (RobotParams.Preferences.useExternalOdometry)
        // {
        //     //
        //     // Create the external odometry device that uses the left front encoder port as the X odometry and
        //     // the left and right back encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
        //     // odometry.
        //     //
        //     TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
        //         new TrcDriveBaseOdometry.AxisSensor(rightBackWheel, RobotParams.X_ODOMETRY_WHEEL_OFFSET),
        //         new TrcDriveBaseOdometry.AxisSensor[] {
        //             new TrcDriveBaseOdometry.AxisSensor(leftFrontWheel, RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
        //             new TrcDriveBaseOdometry.AxisSensor(rightFrontWheel, RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
        //         gyro);
        //     //
        //     // Set the drive base to use the external odometry device overriding the built-in one.
        //     //
        //     driveBase.setDriveBaseOdometry(driveBaseOdometry);
        //     driveBase.setOdometryScales(RobotParams.ODWHEEL_X_INCHES_PER_COUNT, RobotParams.ODWHEEL_Y_INCHES_PER_COUNT);
        // }
        // else
        // {
        //     driveBase.setOdometryScales(RobotParams.ENCODER_X_INCHES_PER_COUNT, RobotParams.ENCODER_Y_INCHES_PER_COUNT);
        // }

        //
        // Create and initialize PID controllers.
        //
        xPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.MECANUM_X_KP, RobotParams.MECANUM_X_KI, RobotParams.MECANUM_X_KD, RobotParams.MECANUM_X_KF);
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.MECANUM_Y_KP, RobotParams.MECANUM_Y_KI, RobotParams.MECANUM_Y_KD, RobotParams.MECANUM_Y_KF);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_TURN_KP, RobotParams.GYRO_TURN_KI, RobotParams.GYRO_TURN_KD, RobotParams.GYRO_TURN_KF);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl", xPosPidCoeff, RobotParams.MECANUM_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl", yPosPidCoeff, RobotParams.MECANUM_Y_TOLERANCE, driveBase::getYPosition);
        gyroTurnPidCtrl = new TrcPidController(
            "gyroPidCtrl", turnPidCoeff, RobotParams.GYRO_TURN_TOLERANCE, driveBase::getHeading);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);

        encoderXPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_XPID_POWER);
        encoderYPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_YPID_POWER);
        gyroTurnPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_TURNPID_POWER);
        encoderXPidCtrl.setRampRate(RobotParams.DRIVE_MAX_XPID_RAMP_RATE);
        encoderYPidCtrl.setRampRate(RobotParams.DRIVE_MAX_YPID_RAMP_RATE);
        gyroTurnPidCtrl.setRampRate(RobotParams.DRIVE_MAX_TURNPID_RAMP_RATE);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setMsgTracer(robot.globalTracer, true, true);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase, RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE,
            RobotParams.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_OUTPUT_LIMIT);
        purePursuitDrive.setStallDetectionEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, true, true);
    }   //MecanumDrive

    /**
     * This method is called to prepare the robot base before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    public void startMode(RunMode runMode, RunMode prevMode)
    {
        if (runMode != RunMode.DISABLED_MODE)
        {
            driveBase.setOdometryEnabled(true);
        }
    }   //startMode

    /**
     * This method is called to prepare the robot base right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    public void stopMode(RunMode runMode, RunMode nextMode)
    {
        if (runMode != RunMode.DISABLED_MODE)
        {
            driveBase.setOdometryEnabled(false);
        }
    }   //stopMode

    /**
     * This method cancels any PIDDrive operation still in progress.
     */
    public void cancel()
    {
        if (pidDrive.isActive())
        {
            pidDrive.cancel();
        }

        if (purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel();
        }

        driveBase.stop();
    }   //cancel

    public void startCalibrate()
    {
        throw new UnsupportedOperationException("Mecanum Drive does not support calibration.");
    }   //startCalibrate

    public void calibratePeriodic()
    {
        throw new UnsupportedOperationException("Mecanum Drive does not support calibration.");
    }   //calibratePeriodic

}   //class MecanumDrive
