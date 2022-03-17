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
import edu.wpi.first.wpilibj.DriverStation;
import team492.ShootParamTable.ShootLoc;

class CmdAuto3Or5Balls implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto3Or5Balls";

    private enum State
    {
        START_DELAY,
        PREPARE_TO_SHOOT,
        SHOOT,
        TURN_TO_2ND_BALL,
        PICKUP_RING_BALLS,
        PICKUP_ONE_MORE,
        //ignore this for now
        GO_TO_TERMINAL,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final boolean do5Balls;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent driveEvent;
    private final TrcStateMachine<State> sm;

    //keeps track of number of balls robot has already shot
    int numBallsShot = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     * @param do5Balls specifies true to shoot 2 balls, false to shoot only pre-loaded ball.
     */
    CmdAuto3Or5Balls(Robot robot, FrcAuto.AutoChoices autoChoices, boolean do5Balls)
    {
        robot.globalTracer.traceInfo(
            moduleName, ">>> robot=%s, choices=%s, do5Balls=%s", robot, autoChoices, do5Balls);

        this.robot = robot;
        this.autoChoices = autoChoices;
        this.do5Balls = do5Balls;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName + ".event");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);
    }   //CmdAuto3Or5Balls

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
                    TrcPose2D startPose = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                        RobotParams.AUTO_5BALL_STARTPOS_RED: RobotParams.AUTO_5BALL_STARTPOS_BLUE;
                    robot.robotDrive.driveBase.setFieldPosition(startPose);
                    //
                    // Do start delay if any.
                    //
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.PREPARE_TO_SHOOT);
                    }
                    else
                    {
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.PREPARE_TO_SHOOT);
                    }
                    break;

                case PREPARE_TO_SHOOT:
                    //if we havent shot any balls, we are only shooting the preload
                    //otherwise we will shoot 2 balls at once
                    if(numBallsShot == 0)
                    {
                        robot.shooter.prepareToShootNoVision(moduleName, event, ShootLoc.TarmacAuto);
                        numBallsShot++;
                    }
                    else
                    {
                        robot.shooter.prepareToShootWithVision(moduleName, event, ShootLoc.TarmacMid);
                        numBallsShot += 2;
                    }
                    sm.waitForSingleEvent(event, State.SHOOT);
                    break;

                case SHOOT:
                    //only shot one ball before, need to pickup the 2 ring balls
                    sm.waitForSingleEvent(
                        event, numBallsShot == 1? State.TURN_TO_2ND_BALL:
                               numBallsShot == 3 && do5Balls? State.GO_TO_TERMINAL: State.DONE);
                    robot.shooter.shootAllBalls(moduleName, event);
                    break;

                case TURN_TO_2ND_BALL:
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, -10.0, 180.0));
                    sm.waitForSingleEvent(event, State.PICKUP_RING_BALLS);
                    break;

                // case PICKUP_2ND_BALL:
                //     //drive to the ball while running the intake
                //     robot.intake.extend();
                //     robot.intake.pickup(event);
                //     robot.robotDrive.purePursuitDrive.start(
                //         null, robot.robotDrive.driveBase.getFieldPosition(), true,
                //         new TrcPose2D(0.0, 26.0, 0));
                //     sm.waitForSingleEvent(event, State.TURN_AROUND);
                //     break;

                case PICKUP_RING_BALLS:
                    robot.intake.extend();
                    robot.intake.pickup(event);
                    sm.waitForSingleEvent(event, State.PICKUP_ONE_MORE);
                    if (autoChoices.getAlliance() == DriverStation.Alliance.Red)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                            new TrcPose2D(26.5, 108.5, 358.0),
                            new TrcPose2D(125.75, 88.3, 302.0),
                            new TrcPose2D(283.5, 117, 259.5));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                            new TrcPose2D(-24, -132, 177.0),
                            new TrcPose2D(-111.4, -95.8, 302),
                            new TrcPose2D(-282.0, -117.7, 259.5));
                    }
                    break;

                case PICKUP_ONE_MORE:
                    robot.intake.pickup();
                    sm.waitForSingleEvent(driveEvent, State.PREPARE_TO_SHOOT);
                    break;

                case GO_TO_TERMINAL:
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

}   //class CmdAuto3Or5Balls
