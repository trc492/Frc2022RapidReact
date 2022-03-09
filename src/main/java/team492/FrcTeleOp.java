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
    private boolean flywheelEnabled = false;
    private boolean turnOffFlywheel = false;
    private boolean tilterControl = false;
    private boolean climberControl = false;
    private DriveOrientation driveOrientation = DriveOrientation.ROBOT;
    private double driveSpeedScale = RobotParams.DRIVE_MEDIUM_SCALE;
    private double turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;

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
        driveOrientation = DriveOrientation.ROBOT;
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
        if(RobotParams.Preferences.useVision)
        {
            robot.vision.vision.setEnabled(false);
        }
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
            if (robot.driverController != null)
            {
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
            }

            if (robot.robotDrive != null)
            {
                double[] inputs = getDriveInputs();
                if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    robot.robotDrive.driveBase.holonomicDrive(inputs[0], inputs[1], inputs[2], getDriveGyroAngle());
                }
                else
                {
                    robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
                }
            }
            //
            // Analog control of subsystem is done here if necessary.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (flywheelEnabled)
                {
                    // robot.shooterLowerVelocity = (robot.leftDriveStick.getZ() + 1.0)/2.0 * RobotParams.FLYWHEEL_MAX_RPM;
                    // robot.shooterUpperVelocity = (robot.operatorStick.getZ() + 1.0)/2.0 * RobotParams.FLYWHEEL_MAX_RPM;
                    robot.shooter.setFlywheelValue(robot.lowerFlywheelUserVel, robot.upperFlywheelUserVel);
                }
                else if (turnOffFlywheel)
                {
                    // Just make sure the flywheels are off but don't lock it in zero power so others may control them.
                    robot.shooter.setFlywheelValue(0.0, 0.0);
                    turnOffFlywheel = false;
                }

                if (tilterControl)
                {
                    // double tilterPower = robot.operatorStick.getYWithDeadband(true);
                    // robot.shooter.setTilterPower(tilterPower);
                }

                if (climberControl)
                {
                    double climberPower = robot.operatorStick.getYWithDeadband(true);
                    robot.climber.setPower(climberPower);
                }
            }
        }
        //
        // Update robot status
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
            robot.leftDriveStick.setButtonHandler(enabled? this::leftDriveStickButtonEvent: null);
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
            if(RobotParams.Preferences.timDrive)
            {
                rot = robot.rightDriveStick.getTwistWithDeadband(true);
            }
            else
            {
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

    /**
     * This method is called when a driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void driverControllerButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "DriverController: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released");

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
     * This method is called when a right driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void leftDriveStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "  LeftDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                if (pressed)
                {
                    robot.lowerFlywheelUserVel += 100;
                    if (robot.lowerFlywheelUserVel > RobotParams.FLYWHEEL_MAX_RPM)
                    {
                        robot.lowerFlywheelUserVel = RobotParams.FLYWHEEL_MAX_RPM;
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                if (pressed)
                {
                    robot.lowerFlywheelUserVel -= 100;
                    if (robot.lowerFlywheelUserVel < 0)
                    {
                        robot.lowerFlywheelUserVel = 0;
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                if (pressed)
                {
                    robot.upperFlywheelUserVel += 100;
                    if (robot.upperFlywheelUserVel > RobotParams.FLYWHEEL_MAX_RPM)
                    {
                        robot.upperFlywheelUserVel = RobotParams.FLYWHEEL_MAX_RPM;
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                if (pressed)
                {
                    robot.upperFlywheelUserVel -= 100;
                    if (robot.upperFlywheelUserVel < 0)
                    {
                        robot.upperFlywheelUserVel = 0;
                    }
                }
                break;
        }
    }   //leftDriveStickButtonEvent

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
                    driveOrientation = DriveOrientation.INVERTED;
                }
                else
                {
                    driveOrientation = DriveOrientation.ROBOT;
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                robot.robotDrive.setAntiDefenseEnabled("AntiDefense", pressed);
                break;
        }
    }   //rightDriveStickButtonEvent

    //TODO: operator stick button event
    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void operatorStickButtonEvent(int button, boolean pressed)
    {
        System.out.println("entered operator event");
        robot.dashboard.displayPrintf(
            8, "   OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                if (pressed)
                {
                    robot.shooter.shootAllBallsNoVision("teleOp", null);
                }
            break;

            case FrcJoystick.LOGITECH_BUTTON2:
                if (pressed)
                {
                    robot.shooter.shootAllBallsNoVision("teleOp", null, robot.presets.get("tarmac_auto"), true);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                if (pressed)
                {
                    robot.intake.extend();
                    robot.intake.pickup();
                }
                else
                {
                    robot.intake.stop();
                    robot.intake.retract();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                if (pressed)
                {
                    robot.intake.extend();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                if (pressed)
                {
                    robot.conveyor.setPower(0.0, 0.4, 0.1);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                if (pressed)
                {
                    robot.shooter.shootAllBallsNoVision(
                        "teleOp", null,
                        robot.shooter.new ShootParams(
                            "calibration", robot.lowerFlywheelUserVel, robot.upperFlywheelUserVel,
                            robot.shooter.getTilterPosition()),
                        false);
                }
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
            8, "     ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON_RED1:
                    robot.intake.retract();
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                robot.shooter.shootAllBallsNoVision("teleOp", null, robot.presets.get("tarmac_mid"), true);
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                robot.shooter.shootAllBallsNoVision("teleOp", null, robot.presets.get("tarmac_auto"), true);
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE1:
                //TODO: Owner overrides
                if (pressed)
                {
                    robot.shooter.cancel();
                    robot.conveyor.setPower(0.0);
                    robot.intake.setPower(0.0);
                    robot.intake.retract();
                    robot.shooter.setFlywheelValue(0.0);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                double power = pressed? -0.5: 0.0;
                robot.shooter.setFlywheelPower(power);
                robot.conveyor.setPower(power);
                robot.intake.setPower(power);
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                if (pressed)
                {
                    robot.climber.climber.setTarget(63.5);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                if (pressed)
                {
                    robot.climber.climber.setTarget(31.0, true);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                if (pressed)
                {
                    // robot.shooter.zeroCalibrateTilter();
                }
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE2:
                if (pressed)
                {
                    robot.climber.zeroCalibrateClimber();
                }
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
            8, "     SwitchPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_SWITCH_WHITE1:
                break;

            case FrcJoystick.PANEL_SWITCH_RED1:
                if (robot.pdp != null)
                {
                    robot.pdp.setSwitchableChannel(pressed);
                }
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
