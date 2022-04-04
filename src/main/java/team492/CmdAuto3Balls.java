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

class CmdAuto3Balls implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto3Balls";

    private enum State
    {
        //copied from CmdAUto1Or2Balls
        //minor change for shooting 2nd ball- dont go forward as much to shoot 2nd ball,  use tarmac_auto preset because within tarmac
        START_DELAY,
        SHOOT,
        TURN_TO_2ND_BALL,
        PICKUP_2ND_BALL,
        TURN_AROUND,
        GET_OFF_TARMAC,
        DONE,
        //new 3ball auto states
        //turn -100 degrees drive forward to pickup 3rd ball
        PICKUP_3RD_BALL,
        //once 3rd ball picked up turn forward 50 degrees 
        TURN_AROUND_3RD_BALL,
        //shoot with vision(no preset provided )
        AIM_TO_SHOOT_THIRD,
        SHOOT_THIRD_BALL,

    }   //enum State

    private final Robot robot;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private boolean got2ndBall = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAuto3Balls(Robot robot)
    {
        robot.globalTracer.traceInfo(
            moduleName, ">>> robot=%s, choices=%s, do2Balls=%s", robot, robot.autoChoices);

        this.robot = robot;
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
        robot.shooter.cancel();
        robot.shooter.stopFlywheel();
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
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.SHOOT);
                    }
                    else
                    {
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.SHOOT);
                        break;
                    }

                case SHOOT:
                    sm.waitForSingleEvent(
                        event, !got2ndBall? State.TURN_TO_2ND_BALL: State.PICKUP_3RD_BALL);
                    if (got2ndBall)
                    {
                        robot.shooter.shootWithVision(moduleName, event);
                    }
                    else
                    {
                        robot.shooter.shootWithNoVision(moduleName, event, ShootLoc.TarmacAuto);
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
                        new TrcPose2D(0.0, 26.0, 0));
                    break;

                case TURN_AROUND:
                    robot.robotDrive.purePursuitDrive.cancel();
                    got2ndBall = true;
                    robot.intake.retract();
                    sm.waitForSingleEvent(event, State.SHOOT);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -36.0, 180.0));
                    break;

                case PICKUP_3RD_BALL:
                    robot.intake.extend();
                    sm.waitForSingleEvent(event, State.TURN_AROUND_3RD_BALL);
                    robot.intake.pickup(event);
                    robot.robotDrive.purePursuitDrive.start(
                        null, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(-100.0, 0.0, -90.0));
                    break;

                case TURN_AROUND_3RD_BALL:
                    robot.robotDrive.purePursuitDrive.cancel();
                    //retract intake
                    //turn 100 degrees clockwise and backup to get in shooting position
                    robot.intake.retract();
                    sm.waitForSingleEvent(event, State.AIM_TO_SHOOT_THIRD);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 2.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 10.0, 100.0));
                    break;

                case AIM_TO_SHOOT_THIRD:
                    //shoot with vision because interpolation works pretty well at far distances 
                    sm.waitForSingleEvent(event, State.DONE);
                    robot.shooter.shootWithVision(moduleName, event);
                    break; 

                case GET_OFF_TARMAC:
                    sm.waitForSingleEvent(event, State.DONE);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -40.0, 0.0));
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
