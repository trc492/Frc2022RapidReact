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
        BACKUP,
        SHOOT,
        TURN_TO_2ND_BALL,
        PICKUP_2ND_BALL,
        TURN_AROUND,
        GET_OFF_TARMAC,
        DONE
    }   //enum State

    private final Robot robot;
    private final boolean do2Balls;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private boolean got2ndBall = false;
    private boolean lastResort = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param do2Balls specifies true to shoot 2 balls, false to shoot only pre-loaded ball.
     * @param lastResort specifies true if our robot is completely broken, can't even drive,
     *        we would just stay inside Tarmac and shoot that one ball.
     */
    CmdAuto1Or2Balls(Robot robot, boolean do2Balls, boolean lastResort)
    {
        robot.globalTracer.traceInfo(
            moduleName, ">>> robot=%s, do2Balls=%s, choices=%s", robot, do2Balls, robot.autoChoices);

        this.robot = robot;
        this.do2Balls = do2Balls;
        this.lastResort = lastResort;
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
        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        robot.shooter.shutdown();
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
                    // We shoot in place the pre-loaded ball if it's lastResort or if we are doing 2 balls.
                    State nextState = lastResort || do2Balls? State.SHOOT: State.BACKUP;
                    //
                    // Set robot starting position in the field.
                    //
                    robot.robotDrive.setFieldPosition(false);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
                    //
                    // Do start delay if any.
                    //
                    double startDelay = robot.autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        sm.setState(nextState);
                    }
                    else
                    {
                        sm.waitForSingleEvent(event, nextState);
                        timer.set(startDelay, event);
                    }
                    break;

                case BACKUP:
                    // We are backing up to shoot if we are doing 1-ball and not lastResort.
                    sm.waitForSingleEvent(event, State.SHOOT);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 1.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -40.0, 0.0));
                    break;

                case SHOOT:
                    // If we are doing lastResort or we are just shooting one ball, we are done.
                    // Otherwise, if we are doing the first ball of two, we will go get the 2nd ball next.
                    // Otherwise, we are doing the 2nd ball, get off the tarmac next.
                    nextState = lastResort || !do2Balls? State.DONE: !got2ndBall? State.TURN_TO_2ND_BALL: State.GET_OFF_TARMAC;
                    sm.waitForSingleEvent(event, nextState);
                    if (lastResort || do2Balls && !got2ndBall)
                    {
                        robot.shooter.shootWithVision(moduleName, event, ShootLoc.TarmacAuto);
                    }
                    else
                    {
                        robot.shooter.shootWithVision(moduleName, event);
                    }
                    break;

                case TURN_TO_2ND_BALL:
                    sm.waitForSingleEvent(event, State.PICKUP_2ND_BALL);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -10.0, 180.0));
                    break;

                case PICKUP_2ND_BALL:
                    //drive to the ball while running the intake
                    robot.intake.extend();
                    sm.waitForSingleEvent(event, State.TURN_AROUND);
                    robot.intake.pickup(event);
                    robot.robotDrive.purePursuitDrive.start(
                        null, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 30.0, 0.0));
                    break;

                case TURN_AROUND:
                    robot.robotDrive.purePursuitDrive.cancel();
                    got2ndBall = true;
                    robot.intake.retract();
                    sm.waitForSingleEvent(event, State.SHOOT);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 0.0, 180.0));
                    break;

                case GET_OFF_TARMAC:
                    sm.waitForSingleEvent(event, State.DONE);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -50.0, 0.0));
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

}   //class CmdAuto1Or2Balls
