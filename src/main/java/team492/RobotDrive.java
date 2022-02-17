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
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcRobot.RunMode;

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
    public TrcMotor lfDriveMotor, lbDriveMotor, rfDriveMotor, rbDriveMotor;
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
