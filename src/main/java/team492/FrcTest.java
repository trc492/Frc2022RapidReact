/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import TrcCommonLib.command.CmdDriveMotorsTest;
import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.command.CmdTimedDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import TrcFrcLib.frclib.FrcChoiceMenu;
import TrcCommonLib.trclib.TrcRobot.RunMode;

public class FrcTest extends FrcTeleOp
{
    //
    // Global constants.
    //

    //
    // Tests enum.
    //
    private enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        X_PID_DRIVE,
        Y_PID_DRIVE,
        TURN_PID_DRIVE,
        TUNE_X_PID,
        TUNE_Y_PID,
        TUNE_TURN_PID,
        LIVE_WINDOW
    }   //enum Test

    //
    // Global objects.
    //

    //
    // Test choice menu.
    //
    private FrcChoiceMenu<Test> testMenu;
    private Test test;

    //
    // Test Cmd objects.
    //
    private CmdDriveMotorsTest driveMotorsTestCmd;
    private CmdTimedDrive timedDriveCmd;
    private CmdPidDrive pidDriveCmd;

    public FrcTest(Robot robot)
    {
        //
        // Call TeleOp constructor.
        //
        super(robot);

        //
        // Create and initialize global objects.
        //

        //
        // Create and populate Test Mode specific menus.
        //
        testMenu = new FrcChoiceMenu<>("Test/Tests:");
        testMenu.addChoice("Sensors Test", FrcTest.Test.SENSORS_TEST, true, false);
        testMenu.addChoice("Subsystems Test", FrcTest.Test.SUBSYSTEMS_TEST);
        testMenu.addChoice("Drive Motors Test", FrcTest.Test.DRIVE_MOTORS_TEST);
        testMenu.addChoice("X Timed Drive", FrcTest.Test.X_TIMED_DRIVE);
        testMenu.addChoice("Y Timed Drive", FrcTest.Test.Y_TIMED_DRIVE);
        testMenu.addChoice("X PID Drive", FrcTest.Test.X_PID_DRIVE);
        testMenu.addChoice("Y PID Drive", FrcTest.Test.Y_PID_DRIVE);
        testMenu.addChoice("Turn PID Drive", FrcTest.Test.TURN_PID_DRIVE);
        testMenu.addChoice("Tune X PID", FrcTest.Test.TUNE_X_PID);
        testMenu.addChoice("Tune Y PID", FrcTest.Test.TUNE_Y_PID);
        testMenu.addChoice("Tune Turn PID", FrcTest.Test.TUNE_TURN_PID);
        testMenu.addChoice("Live Window", FrcTest.Test.LIVE_WINDOW, false, true);

    } // FrcTest

    //
    // Overriding TrcRobot.RobotMode.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Call TeleOp startMode.
        //
        super.startMode(prevMode, nextMode);

        //
        // Retrieve menu choice values.
        //
        test = testMenu.getCurrentChoiceObject();

        //
        // Create Cmd objects according to test choice.
        //
        boolean liveWindowEnabled = false;

        switch (test)
        {
            case SENSORS_TEST:
                //
                // Make sure no joystick controls on sensors test.
                //

                //
                // Sensors Test is the same as Subsystems Test without joystick control.
                // So let it flow to the next case.
                //
            case SUBSYSTEMS_TEST:
                break;

            case DRIVE_MOTORS_TEST:
                //
                // Initialize motor array with the wheel motors. For 2-motor drive base, it is leftWheel and
                // rightWheel. For 4-motor drive base, it is lfWheel, rfWheel, lbWheel, rbWheel.
                //
                // driveMotorsTestCmd = new CmdDriveMotorsTest(
                //     new TrcMotor[] {robot.lfWheel, robot.rfWheel, robot.lbWheel, robot.rbWheel}, 5.0, 0.5);
                break;

            case X_TIMED_DRIVE:
                // timedDriveCmd = new CmdTimedDrive(robot, 0.0, robot.driveTime, robot.drivePower, 0.0, 0.0);
                break;

            case Y_TIMED_DRIVE:
                // timedDriveCmd = new CmdTimedDrive(robot, 0.0, robot.driveTime, 0.0, robot.drivePower, 0.0);
                break;

            case X_PID_DRIVE:
                // pidDriveCmd = new CmdPidDrive(
                //     robot.driveBase, robot.pidDrive, 0.0, robot.driveDistance, 0.0, 0.0, robot.drivePowerLimit,
                //     false, null);
                break;

            case Y_PID_DRIVE:
                // pidDriveCmd = new CmdPidDrive(
                //     robot.driveBase, robot.pidDrive, 0.0, 0.0, robot.driveDistance, 0.0, robot.drivePowerLimit,
                //     false, null);
                break;

            case TURN_PID_DRIVE:
                // pidDriveCmd = new CmdPidDrive(
                //     robot.driveBase, robot.pidDrive, 0.0, 0.0, 0.0, robot.turnDegrees, robot.drivePowerLimit,
                //     false, null);
                break;

            case TUNE_X_PID:
                // pidDriveCmd = new CmdPidDrive(
                //     robot.driveBase, robot.pidDrive, 0.0, robot.driveDistance, 0.0, 0.0, robot.drivePowerLimit,
                //     false, robot.tunePidCoeff);
                break;

            case TUNE_Y_PID:
                // pidDriveCmd = new CmdPidDrive(
                //     robot.driveBase, robot.pidDrive, 0.0, 0.0, robot.driveDistance, 0.0, robot.drivePowerLimit,
                //     false, robot.tunePidCoeff);
                break;

            case TUNE_TURN_PID:
                // pidDriveCmd = new CmdPidDrive(
                //     robot.driveBase, robot.pidDrive, 0.0, 0.0, 0.0, robot.turnDegrees, robot.drivePowerLimit,
                //     false, robot.tunePidCoeff);
                break;

            case LIVE_WINDOW:
                liveWindowEnabled = true;
                break;

            default:
                break;
        }

        LiveWindow.setEnabled(liveWindowEnabled);

        //
        // Start test state machine if necessary.
        //

    } // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        super.stopMode(prevMode, nextMode);
    }   // stopMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //
    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // Call super.runPeriodic only if you need TeleOp control of the robot.
        //
        switch (test)
        {
            case SENSORS_TEST:
                doSensorsTest();
                break;

            case SUBSYSTEMS_TEST:
                //
                // Allow TeleOp to run so we can control the robot in subsystems test mode.
                //
                super.runPeriodic(elapsedTime);
                doSensorsTest();
                break;

            default:
                break;
        }

        //
        // Update Dashboard here.
        //
        robot.updateDashboard(RunMode.TEST_MODE);
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        //
        // Run test Cmd.
        //
        switch (test)
        {
            case SENSORS_TEST:
                super.runContinuous(elapsedTime);
                break;

            case DRIVE_MOTORS_TEST:
                driveMotorsTestCmd.cmdPeriodic(elapsedTime);
                break;

            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                // double lfEnc = robot.lfWheel.getPosition();
                // double rfEnc = robot.rfWheel.getPosition();
                // double lbEnc = robot.lfWheel.getPosition();
                // double rbEnc = robot.rbWheel.getPosition();
                // robot.dashboard.displayPrintf(2, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
                // robot.dashboard.displayPrintf(3, "Enc:lb=%.0f,rb=%.0f", lbEnc, rbEnc);
                // robot.dashboard.displayPrintf(4, "average=%f", (lfEnc + rfEnc + lbEnc + rbEnc) / 4.0);
                // robot.dashboard.displayPrintf(5, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                //     robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading());
                timedDriveCmd.cmdPeriodic(elapsedTime);
                break;

            case X_PID_DRIVE:
            case Y_PID_DRIVE:
            case TURN_PID_DRIVE:
            case TUNE_X_PID:
            case TUNE_Y_PID:
            case TUNE_TURN_PID:
                // robot.dashboard.displayPrintf(2, "xPos=%.1f,yPos=%.1f,heading=%.1f, lf=%.2f,rf=%.2f,lb=%.2f,rb=%.2f",
                //     robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading(),
                //     robot.lfWheel.getPosition(), robot.rfWheel.getPosition(),
                //     robot.lbWheel.getPosition(), robot.rbWheel.getPosition());
                // robot.encoderXPidCtrl.displayPidInfo(3);
                // robot.encoderYPidCtrl.displayPidInfo(5);
                // robot.gyroTurnPidCtrl.displayPidInfo(7);
                pidDriveCmd.cmdPeriodic(elapsedTime);
                break;

            default:
                break;
        }

        // if (robot.pidDrive.isActive())
        // {
        //     robot.encoderXPidCtrl.printPidInfo(robot.globalTracer, false, robot.battery);
        //     robot.encoderYPidCtrl.printPidInfo(robot.globalTracer, false, robot.battery);
        //     robot.gyroTurnPidCtrl.printPidInfo(robot.globalTracer, false, robot.battery);
        // }

    } // runContinuous

    //
    // Overriding ButtonEvent here if necessary.
    //

    //
    // Implement tests.
    //
    /**
     * This method reads all sensors and prints out their values. This is a very
     * useful diagnostic tool to check if all sensors are working properly. For
     * encoders, since test subsystem mode is also teleop mode, you can operate
     * the joysticks to turn the motors and check the corresponding encoder
     * counts.
     */
    private void doSensorsTest()
    {
        //
        // Display drivebase info.
        //
        // robot.dashboard.displayPrintf(1, "Sensors Test (Batt=%.1f/%.1f):", robot.battery.getVoltage(),
        //     robot.battery.getLowestVoltage());
        // robot.dashboard.displayPrintf(2, "DriveBase: X=%.1f,Y=%.1f,Heading=%.1f,xVel=%.2f,yVel=%.2f",
        //     robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading(),
        //     robot.driveBase.getXVelocity(), robot.driveBase.getYVelocity());
        // robot.dashboard.displayPrintf(3, "Encoders: lf=%.1f,rf=%.1f,lr=%.1f,rr=%.1f",
        //     robot.lfWheel.getPosition(), robot.rfWheel.getPosition(), robot.lbWheel.getPosition(),
        //     robot.rbWheel.getPosition());
        // robot.dashboard.displayPrintf(4, "Power: lf=%.2f,rf=%.2f,lr=%.2f,rr=%.2f",
        //     robot.lfWheel.getPower(), robot.rfWheel.getPower(), robot.lbWheel.getPower(),
        //     robot.rbWheel.getPower());

        //
        // Display other subsystems and sensor info.
        //

    }   //doSensorsTest

} // class FrcTest
