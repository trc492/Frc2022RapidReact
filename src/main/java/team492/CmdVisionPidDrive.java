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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcFrcLib.frclib.FrcJoystick;
import team492.ShootParamTable.ShootLoc;

class CmdVisionPidDrive implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdVisionPidDrive";

    private enum State
    {
        START,
        PREP_TO_SHOOT,
        WAIT_FOR_RELEASE,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcTest.TestChoices testChoices;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param testChoices specifies all the choices from the test menus.
     */
    CmdVisionPidDrive(Robot robot, FrcTest.TestChoices testChoices)
    {
        robot.globalTracer.traceInfo(
            moduleName, ">>> robot=%s, choices=%s", robot, testChoices);

        this.robot = robot;
        this.testChoices = testChoices;
        event = new TrcEvent(moduleName + ".event");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.PREP_TO_SHOOT);
    }   //CmdVisionPidDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        robot.shooter.cancel();
        robot.shooter.stopFlywheel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=%s)...", sm.getNextState());
        }
        else
        {
            robot.dashboard.displayPrintf(8, "State: %s", state);
            switch (state)
            {
                case START:
                    // Set shooter to use vision alignment, since this is what we are tuning.
                    robot.shooter.setVisionAlignEnabled(true);
                    // Trigger must be pressed (& held) to start the alignment process.
                    if (robot.operatorStick.isButtonPressed(FrcJoystick.LOGITECH_TRIGGER))
                    {
                        // Read dashboard for the new PID.
                        robot.robotDrive.pidDrive.getTurnPidCtrl().setPidCoefficients(
                            testChoices.getTunePidCoefficients());
                        sm.setState(State.PREP_TO_SHOOT);
                    }
                    break;

                case PREP_TO_SHOOT:
                    // Start angling the robot. Shooter method allows driver to turn the robot at the same time,
                    // this is to make it easier to watch the robot snap back to the target.
                    sm.waitForSingleEvent(event, State.WAIT_FOR_RELEASE);
                    robot.shooter.shootWithVision(moduleName, event, ShootLoc.TarmacAuto);
                    // Once PID determines the robot is on target, it will move on to the next state.
                    break;

                case WAIT_FOR_RELEASE:
                    // We are just waiting for the operator to release the trigger button.
                    if (!robot.operatorStick.isButtonPressed(FrcJoystick.LOGITECH_TRIGGER))
                    {
                        // After the trigger is released, we are done.
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    cancel();
                    break;
            }

            robot.globalTracer.traceStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdVisionPidDrive
