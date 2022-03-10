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

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcCANFalcon;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class RobotDrive
{
    //
    // Global objects.
    //
    protected Robot robot;
    //
    // Sensors.
    //
    public TrcGyro gyro;
    //
    // Drive motors.
    //
    public FrcCANFalcon lfDriveMotor, lbDriveMotor, rfDriveMotor, rbDriveMotor;
    //
    // Drive Base.
    //
    public TrcDriveBase driveBase;
    //
    // PID Coefficients and Controllers.
    //
    public TrcPidController.PidCoefficients xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff;
    public TrcPidController encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl;
    //
    // Drive Controllers.
    //
    public TrcPidDrive pidDrive;
    public TrcPurePursuitDrive purePursuitDrive;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public RobotDrive(Robot robot)
    {
        this.robot = robot;
        gyro = RobotParams.Preferences.useNavX ? new FrcAHRSGyro("NavX", SPI.Port.kMXP) : null;
    }   //RobotDrive

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
            driveBase.setOdometryEnabled(true, true);

            if (runMode == RunMode.AUTO_MODE)
            {
                lfDriveMotor.motor.configOpenloopRamp(0.0);
                rfDriveMotor.motor.configOpenloopRamp(0.0);
                lbDriveMotor.motor.configOpenloopRamp(0.0);
                rbDriveMotor.motor.configOpenloopRamp(0.0);
            }
            else
            {
                lfDriveMotor.motor.configOpenloopRamp(RobotParams.DRIVE_RAMP_RATE);
                rfDriveMotor.motor.configOpenloopRamp(RobotParams.DRIVE_RAMP_RATE);
                lbDriveMotor.motor.configOpenloopRamp(RobotParams.DRIVE_RAMP_RATE);
                rbDriveMotor.motor.configOpenloopRamp(RobotParams.DRIVE_RAMP_RATE);
                if (RobotParams.Preferences.useGyroAssist)
                {
                    // driveBase.enableGyroAssist(RobotParams.GYRO_MAX_ROT_RATE, RobotParams.GYRO_ASSIST_GAIN);
                }
            }
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

    protected FrcCANFalcon createDriveMotor(String name, int motorCanID, boolean inverted)
    {
        FrcCANFalcon driveMotor = new FrcCANFalcon(name, motorCanID);

        driveMotor.motor.configFactoryDefault();
        driveMotor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        driveMotor.motor.enableVoltageCompensation(true);
        driveMotor.setInverted(inverted);
        driveMotor.setBrakeModeEnabled(true);

        return driveMotor;
    }   //createDriveMotor

    /**
     * This method is called to start steering calibration for Swerve Drive.
     */
    public void startSteerCalibrate()
    {
        throw new UnsupportedOperationException("Drivebase does not support calibration.");
    }   //startSteerCalibrate

    /**
     * This method is called periodically to calibrate steering for Swerve Drive.
     */
    public void steerCalibratePeriodic()
    {
        throw new UnsupportedOperationException("Drivebase does not support calibration.");
    }   //steerCalibratePeriodic

}   //class RobotDrive
