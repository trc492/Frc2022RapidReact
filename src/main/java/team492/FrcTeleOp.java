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

import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcPidController.PidCoefficients;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcXboxController;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    public enum DriveOrientation
    {
        ROBOT, FIELD, INVERTED
    }   //enum DriveOrientation

    //
    // Global objects.
    //
    protected final Robot robot;
    private boolean controlsEnabled = false;
    private DriveOrientation driveOrientation = DriveOrientation.FIELD;
    private double driveSpeedScale = RobotParams.DRIVE_MEDIUM_SCALE;
    private double turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
    private double intakePower = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcTeleOp(Robot robot)
    {
        //
        // Create and initialize global object.
        //
        this.robot = robot;
    }   //FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    /**
     * This method is called when the teleop mode is about to start. Typically, you put code that will prepare
     * the robot for start of teleop here such as creating and configuring joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Enabling joysticks.
        //
        setControlsEnabled(true);
        //
        // Initialize subsystems for TeleOp mode if necessary.
        //
        driveOrientation = DriveOrientation.FIELD;
        driveSpeedScale = RobotParams.DRIVE_MEDIUM_SCALE;
        turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
    }   //startMode

    /**
     * This method is called when teleop mode is about to end. Typically, you put code that will do clean
     * up here such as disabling joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disabling joysticks.
        //
        setControlsEnabled(false);
        //
        // Disable subsystems before exiting if necessary.
        //

    }   //stopMode

    /**
     * This method is called periodically about 50 times a second. Typically, you put code that doesn't require
     * frequent update here such as reading joystick analog controls and updating dashboard.
     * 
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (controlsEnabled)
        {
            //
            // DriveBase operation.
            //

            // D-pad Gearshift
            switch (robot.driverController.getPOV())
            {
                case 0:
                    driveSpeedScale = RobotParams.DRIVE_FAST_SCALE;
                    turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
                    break;

                case 270:
                    driveSpeedScale = RobotParams.DRIVE_MEDIUM_SCALE;
                    turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
                    break;

                case 180:
                    driveSpeedScale = RobotParams.DRIVE_SLOW_SCALE;
                    turnSpeedScale = RobotParams.TURN_SLOW_SCALE;
                    break;
            }

            double[] inputs = getDriveInputs();
            if (robot.robotDrive.driveBase.supportsHolonomicDrive())
            {
                robot.robotDrive.driveBase.holonomicDrive(inputs[0], inputs[1], inputs[2], getDriveGyroAngle());
            }
            else
            {
                robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
            }
            //
            // Analog control of subsystem is done here if necessary.
            //

            //Tim XUter
            double intakePower = robot.operatorStick.getY();
            robot.intakeMotor.set(intakePower);

            double shooterLowerPower = (robot.operatorStick.getZ() + 1.0)/2.0;
            double shooterUpperPower = (robot.leftDriveStick.getZ() + 1.0)/2.0;
            double shooterLowerVel = robot.shooter.getLowerFlywheelVelocity();
            double shooterUpperVel = robot.shooter.getUpperFlywheelVelocity();

            robot.shooter.setFlywheelPower(shooterLowerPower, shooterUpperPower);
            robot.dashboard.displayPrintf(11, "Shooter: Lower:%.1f/%.1f, Upper:%.1f/%.1f",
                shooterLowerVel, shooterLowerPower*RobotParams.SHOOTER_FLYWHEEL_MAX_VEL,
                shooterUpperVel, shooterUpperPower*RobotParams.SHOOTER_FLYWHEEL_MAX_VEL);
        }
        //
        // Update dashboard.
        //
        if (RobotParams.Preferences.doAutoUpdates)
        {
            robot.updateStatus();
        }
    }   //runPeriodic

    /**
     * This method is called periodically as fast as the control system allows. Typically, you put code that requires
     * servicing at a higher frequency here such as running the auto-assist commands which requires responsiveness and
     * accuracy.
     * 
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
        //
        // Do subsystem auto-assist here if necessary.
        //

    }   //runContinuous

    /**
     * This method enables/disables joystick controls.
     *
     * @param enabled specifies true to enable joystick control, false to disable.
     */
    protected void setControlsEnabled(boolean enabled)
    {
        controlsEnabled = enabled;

        if (RobotParams.Preferences.useXboxController)
        {
            robot.driverController.setButtonHandler(enabled? this::driverControllerButtonEvent: null);
        }
        else
        {
            robot.rightDriveStick.setButtonHandler(enabled? this::rightDriveStickButtonEvent: null);
        }

        robot.operatorStick.setButtonHandler(enabled? this::operatorStickButtonEvent: null);
        if (RobotParams.Preferences.useButtonPanels)
        {
            robot.buttonPanel.setButtonHandler(enabled? this::buttonPanelButtonEvent: null);
            robot.switchPanel.setButtonHandler(enabled? this::switchPanelButtonEvent: null);
        }
    }   //setControlsEnabled

    /**
     * This method returns robot heading to be maintained in teleop drive according to drive orientation mode.
     *
     * @return robot heading to be maintained.
     */
    private double getDriveGyroAngle()
    {
        switch (driveOrientation)
        {
            case ROBOT:
                return 0.0;

            case INVERTED:
                return 180.0;

            default:
            case FIELD:
                return robot.robotDrive.driveBase.getHeading();
        }
    }   //getDriveGyroAngle

    /**
     * This method reads various joystick/gamepad control values and returns the drive powers for all three degrees
     * of robot movement.
     *
     * @return an array of 3 values for x, y and rotation power.
     */
    private double[] getDriveInputs()
    {
        double x, y, rot;
        double mag;
        double newMag;

        if (RobotParams.Preferences.useXboxController)
        {
            x = robot.driverController.getLeftXWithDeadband(false);
            y = robot.driverController.getLeftYWithDeadband(false);
            rot = robot.driverController.getRightXWithDeadband(true);
            mag = TrcUtil.magnitude(x, y);
            if (mag > 1.0)
            {
                x /= mag;
                y /= mag;
                mag = 1.0;
            }
            newMag = Math.pow(mag, 3);
        }
        else
        {
            x = robot.rightDriveStick.getXWithDeadband(false);
            y = robot.rightDriveStick.getYWithDeadband(false);
            if(RobotParams.Preferences.timDrive) {
                rot = -1*robot.rightDriveStick.getTwistWithDeadband(true);
            } else {
                rot = robot.leftDriveStick.getXWithDeadband(true);
            }
            mag = TrcUtil.magnitude(x, y);
            if (mag > 1.0)
            {
                x /= mag;
                y /= mag;
                mag = 1.0;
            }
            newMag = Math.pow(mag, 2);
        }

        newMag *= driveSpeedScale;
        rot *= turnSpeedScale;

        if (mag != 0.0)
        {
            x *= newMag / mag;
            y *= newMag / mag;
        }

        return new double[] { x, y, rot };
    }   //getDriveInput

    //
    // Implements FrcButtonHandler.
    //

    /**
     * This method is called when a right driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void rightDriveStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, " RightDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                if (pressed)
                {
                    if (driveOrientation != DriveOrientation.FIELD)
                    {
                        driveOrientation = DriveOrientation.FIELD;
                    }
                    else
                    {
                        driveOrientation = DriveOrientation.ROBOT;
                    }
                }
                break;
        }
    }   //rightDriveStickButtonEvent

    /**
     * This method is called when a driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void driverControllerButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, " DriverController: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                break;

            case FrcXboxController.BUTTON_B:
                break;

            case FrcXboxController.BUTTON_X:
                break;

            case FrcXboxController.BUTTON_Y:
                if (pressed)
                {
                    if (driveOrientation != DriveOrientation.FIELD)
                    {
                        driveOrientation = DriveOrientation.FIELD;
                    }
                    else
                    {
                        driveOrientation = DriveOrientation.ROBOT;
                    }
                }
                break;

            case FrcXboxController.LEFT_BUMPER:
                if (pressed)
                {
                    driveOrientation = DriveOrientation.INVERTED;
                }
                else
                {
                    driveOrientation = DriveOrientation.FIELD;
                }
                break;

            case FrcXboxController.RIGHT_BUMPER:
                break;

            case FrcXboxController.BACK:
                break;

            case FrcXboxController.START:
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                break;
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void operatorStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "  OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                if(pressed) {
                    intakePower = -1.0;
                } else {
                    intakePower = 0.0;
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    }   //operatorStickButtonEvent

    /**
     * This method is called when a button panel button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void buttonPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "  ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON_RED1:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN1:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE1:
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE2:
                break;
        }
    }   //buttonPanelButtonEvent

    /**
     * This method is called when a switch panel button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void switchPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "  SwitchPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_SWITCH_WHITE1:
                break;

            case FrcJoystick.PANEL_SWITCH_RED1:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN1:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE1:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW1:
                break;

            case FrcJoystick.PANEL_SWITCH_WHITE2:
                break;

            case FrcJoystick.PANEL_SWITCH_RED2:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN2:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE2:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW2:
                break;
        }
    }   //switchPanelButtonEvent

}   //class FrcTeleOp
