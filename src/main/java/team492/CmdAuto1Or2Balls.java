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
import TrcCommonLib.trclib.TrcTimer;
import team492.ShootParamTable.ShootLoc;

class CmdAuto1Or2Balls implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto1Or2Balls";

    private enum State
    {
        START_DELAY,
        AIM_TO_SHOOT,
        SHOOT_BALL,
        TURN_TO_2ND_BALL,
        PICKUP_2ND_BALL,
        TURN_AROUND,
        GET_OFF_TARMAC,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final boolean do2Balls;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private boolean got2ndBall = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     * @param do2Balls specifies true to shoot 2 balls, false to shoot only pre-loaded ball.
     */
    CmdAuto1Or2Balls(Robot robot, FrcAuto.AutoChoices autoChoices, boolean do2Balls)
    {
        robot.globalTracer.traceInfo(
            moduleName, ">>> robot=%s, choices=%s, do2Balls=%s", robot, autoChoices, do2Balls);

        this.robot = robot;
        this.autoChoices = autoChoices;
        this.do2Balls = do2Balls;
        timer = new TrcTimer(moduleName + ".timer");
        event = new TrcEvent(moduleName + ".event");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);
    }   //CmdAuto1Or2Balls

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
            robot.dashboard.displayPrintf(8, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Do start delay if any.
                    //
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.AIM_TO_SHOOT);
                    }
                    else
                    {
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.AIM_TO_SHOOT);
                        break;
                    }

                case AIM_TO_SHOOT:
                    if (got2ndBall)
                    {
                        robot.shooter.prepareToShootWithVision(moduleName, event, ShootLoc.TarmacAuto);
                    }
                    else
                    {
                        robot.shooter.prepareToShootNoVision(moduleName, event, ShootLoc.TarmacAuto);
                    }
                    sm.waitForSingleEvent(event, State.SHOOT_BALL);
                    break;

                case SHOOT_BALL:
                    sm.waitForSingleEvent(
                        event, !do2Balls? State.GET_OFF_TARMAC: !got2ndBall? State.TURN_TO_2ND_BALL: State.DONE);
                    robot.shooter.shootAllBalls(moduleName, event);
                    break;

                case TURN_TO_2ND_BALL:
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -10.0, 180.0));
                    sm.waitForSingleEvent(event, State.PICKUP_2ND_BALL);
                    break;

                case PICKUP_2ND_BALL:
                    //drive to the ball while running the intake
                    robot.intake.extend();
                    robot.intake.pickup(event);
                    robot.robotDrive.purePursuitDrive.start(
                        null, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 26.0, 0));
                    sm.waitForSingleEvent(event, State.TURN_AROUND);
                    break;

                case TURN_AROUND:
                    got2ndBall = true;
                    robot.intake.retract();
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -20.0, 180.0));
                    sm.waitForSingleEvent(event, State.AIM_TO_SHOOT);
                    break;

                case GET_OFF_TARMAC:
                    // robot.robotDrive.purePursuitDrive.start(
                    //     event, robot.robotDrive.driveBase.getFieldPosition(), true,
                    //     new TrcPose2D(0.0, -40.0, 0.0));
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.2, 0.0);
                    timer.set(2.0, event);
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
                state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto1Or2Balls