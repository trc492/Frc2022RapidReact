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

import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcXboxController;
import team492.ShootParamTable.ShootLoc;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    //
    // Global objects.
    //
    protected final Robot robot;
    private boolean controlsEnabled = false;
    private boolean flywheelEnabled = false;
    private boolean climberControl = false;
    private boolean hookArmExtended = false;
    private boolean tilterClose = false;
    private ShootParamTable.Params currShootParams = null;
    private CmdAutoBalance autoBalance;


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
        robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.FIELD);
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
        if (robot.vision != null)
        {
            robot.vision.setEnabled(false);
        }
    }   //stopMode

    /**
     * This method is called periodically at a slow rate. Typically, you put code that doesn't require frequent
     * update here. For example, TeleOp joystick code or status display code can be put here since human responses
     * are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if(autoBalance != null)
        {
            autoBalance.cmdPeriodic(elapsedTime);
        }
        //
        // Do subsystem auto-assist here if necessary.
        //

        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                //
                // DriveBase operation.
                //
                if (robot.driverController != null)
                {
                    switch (robot.driverController.getPOV())
                    {
                        case 0:
                            robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_FAST_SCALE;
                            robot.robotDrive.turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
                            break;

                        case 270:
                            robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_MEDIUM_SCALE;
                            robot.robotDrive.turnSpeedScale = RobotParams.TURN_MEDIUM_SCALE;
                            break;

                        case 180:
                            robot.robotDrive.driveSpeedScale = RobotParams.DRIVE_SLOW_SCALE;
                            robot.robotDrive.turnSpeedScale = RobotParams.TURN_SLOW_SCALE;
                            break;
                    }
                }

                if (robot.robotDrive != null && !robot.robotDrive.isAntiDefenseEnabled())
                {
                    double[] inputs = robot.robotDrive.getDriveInputs();
                    if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                    {
                        robot.robotDrive.driveBase.holonomicDrive(null, inputs[0], inputs[1], inputs[2], getDriveGyroAngle());
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
                        robot.shooter.setFlywheelValue(robot.lowerFlywheelUserVel, robot.upperFlywheelUserVel);
                    }

                    if (climberControl)
                    {
                        double climberPower = robot.operatorStick.getYWithDeadband(true);
                        robot.climber.setPower(climberPower);
                    }
                }
            }
            //
            // Update robot status.
            //
            if (RobotParams.Preferences.doAutoUpdates)
            {
                robot.updateStatus();
            }
        }
    }   //slowPeriodic

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
        switch (robot.robotDrive.driveOrientation)
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

    //
    // Implements FrcButtonHandler.
    //

    /**
     * This method is called when a driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void driverControllerButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "DriverController: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                if (pressed)
                {
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robotPose.angle = 0.0;
                    robot.robotDrive.driveBase.setFieldPosition(robotPose);
                }
                break;

            case FrcXboxController.BUTTON_B:
                break;

            case FrcXboxController.BUTTON_X:
                robot.robotDrive.setAntiDefenseEnabled(null, pressed);
                break;

            case FrcXboxController.BUTTON_Y:
                if (pressed)
                {
                    if (robot.robotDrive.driveOrientation != RobotDrive.DriveOrientation.FIELD)
                    {
                        robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.FIELD);
                    }
                    else
                    {
                        robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.ROBOT);
                    }
                }
                break;

            case FrcXboxController.LEFT_BUMPER:
                if (pressed)
                {
                    robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.INVERTED);
                }
                else
                {
                    robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.FIELD);
                    robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(robot.robotDrive.driveBase.getXPosition(), 
                        robot.robotDrive.driveBase.getYPosition(), 0));
                }
                break;

            case FrcXboxController.RIGHT_BUMPER:
                if (pressed)
                {
                    robot.autoBalance.autoAssistBalance();
                }
                break;

            case FrcXboxController.BACK:
                if(pressed)
                {
                    robot.autoBalance.autoAssistBalanceCancel();
                }
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
            8, "LeftDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

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

            case FrcJoystick.LOGITECH_BUTTON7:
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
            8, "RightDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                if (pressed)
                {
                    robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.INVERTED);
                }
                else
                {
                    robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.FIELD);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                robot.robotDrive.setAntiDefenseEnabled(null, pressed);
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                if (pressed)
                {
                    if (robot.robotDrive.driveOrientation != RobotDrive.DriveOrientation.FIELD)
                    {
                        robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.FIELD);
                    }
                    else
                    {
                        robot.robotDrive.setDriveOrientation(RobotDrive.DriveOrientation.ROBOT);
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                if (pressed)
                {
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robotPose.angle = 0.0;
                    robot.robotDrive.driveBase.setFieldPosition(robotPose);
                }
                break;
        }
    }   //rightDriveStickButtonEvent

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    private void operatorStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            8, "OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                // if(pressed){
                //     robot.shooter.oneClickShooting = true; 
                //     robot.shooter.shootWithVision("teleOp", null);
                // }
                if (pressed)
                {
                    robot.shooter.shootWithVision("teleOp", null, currShootParams);
                }
                else
                {
                    robot.shooter.commitToShoot("teleOp");
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                // robot.robotDrive.driveBase.setFieldPosition(RobotParams.STARTPOS_AUTO_5BALL);
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                if (pressed)
                {
                    robot.intake.extend();
                    robot.intake.pickup();
                }
                else
                {
                    robot.intake.stop(0.2);
                    robot.intake.retract();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                if (pressed)
                {
                    if (robot.intake.isExtended())
                    {
                        robot.intake.retract();
                    }
                    else
                    {
                        robot.intake.extend();
                    }
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
                    robot.shooter.shootWithVision(
                        "teleOp", null,
                        new ShootParamTable.Params(
                            ShootLoc.Calibration, 0.0, robot.lowerFlywheelUserVel, robot.upperFlywheelUserVel,
                            robot.shooter.getTilterPosition()));
                }
                else
                {
                    robot.shooter.commitToShoot("teleOp");
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                // climberControl = pressed;
                // robot.climber.climber.setManualOverride(pressed);
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                if (pressed)
                {
                    tilterClose = !tilterClose;
                    if (tilterClose)
                    {
                        robot.shooter.setTilterPositionClose();
                    }
                    else
                    {
                        robot.shooter.setTilterPositionFar();
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                if(pressed)
                {
                    robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0, 0, 0));
                    //RobotParams.BLUE_START_POS_2_BALL_PICKUP_FIRST); 
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                if (pressed)
                {
                    hookArmExtended = !hookArmExtended;
                    if (hookArmExtended)
                    {
                        robot.climber.extendHookArm();
                    }
                    else
                    {
                        robot.climber.retractHookArm();
                    }
                }
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
            8, "ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON_RED1:
                if (pressed)
                {
                    currShootParams = robot.shootParamTable.get(ShootLoc.LaunchPad);
                }
                else
                {
                    currShootParams = null;
                }
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN1:
                if (pressed)
                {
                    currShootParams = robot.shootParamTable.get(ShootLoc.RingMid);
                }
                else
                {
                    currShootParams = null;
                }
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                if (pressed)
                {
                    currShootParams = robot.shootParamTable.get(ShootLoc.TarmacMid);
                }
                else
                {
                    currShootParams = null;
                }
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                if (pressed)
                {
                    robot.climber.traverseOneRung(true);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE1:
                //TODO: Owner overrides?
                if (pressed)
                {
                    robot.shooter.shutdown();
                    robot.conveyor.cancel();
                    robot.intake.cancel();
                    robot.climber.cancel();
                }
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                double power = pressed? -0.5: 0.0;
                robot.shooter.setFlywheelValue(3200.0 * power, 0.0);
                robot.conveyor.setPower(power);
                robot.intake.setPower(power);
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                if (pressed)
                {
                    robot.climber.climber.setTarget(63.0);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                if (pressed)
                {
                    robot.climber.traverseOneRung(false);
                }
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                if (pressed)
                {
                    robot.climber.climber.setTarget(26.0, true);
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
            8, "SwitchPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_SWITCH_WHITE1:
                // if (pressed)
                // {
                //     robot.shooter.setFlywheelValue(RobotParams.FLYWHEEL_MAX_RPM, 0.5*RobotParams.FLYWHEEL_MAX_RPM);
                //     // robot.shooter.setFlywheelValue(-0.5*RobotParams.FLYWHEEL_MAX_RPM, 0.0);
                // }
                // else
                // {
                //     robot.shooter.shutdown();
                // }
                break;

            case FrcJoystick.PANEL_SWITCH_RED1:
                if (robot.pdp != null)
                {
                    robot.pdp.setSwitchableChannel(pressed);
                }
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN1:
                if (robot.shooter != null)
                {
                    robot.shooter.setVisionAlignEnabled(pressed);
                }
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE1:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW1:
                break;

            case FrcJoystick.PANEL_SWITCH_WHITE2:
                if (pressed)
                {
                    hookArmExtended = !hookArmExtended;
                    if (hookArmExtended)
                    {
                        robot.climber.extendHookArm();
                    }
                    else
                    {
                        robot.climber.retractHookArm();
                    }
                }
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
