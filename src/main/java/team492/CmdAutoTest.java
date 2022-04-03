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
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;

class CmdAutoTest implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoTest";

    private enum State
    {
        START,
        PP_DRIVE,
        DONE
    }   //enum State

    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAutoTest(Robot robot)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s", robot);

        this.robot = robot;
        event = new TrcEvent(moduleName + ".event");
        sm = new TrcStateMachine<>(moduleName);

        sm.start(State.START);
    }   //CmdAutoTest

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
        robot.robotDrive.cancel();
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
            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START:
                    // robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0.0, 0.0, -45.0));
                    // robot.robotDrive.driveBase.setFieldPosition(RobotParams.STARTPOS_AUTO_5BALL);
                    robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(90.0, -25.0, 88.5));
                    sm.setState(State.PP_DRIVE);
                    break;

                case PP_DRIVE:
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
                        // Ball 2 Location: 88.303, -124.946, 0.0
                        // new TrcPose2D(90.0 + 10.0, -25.0 - 140.0, 90.0 - 135.0));
                        new TrcPose2D(0.0, -100.0, 0.0));
                        // new TrcPose2D(60.0, 60.0, 90.0),
                        // new TrcPose2D(120.0, 60.0, -90.0));
                    sm.waitForSingleEvent(event, State.DONE);
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

}   //class CmdAutoTest
