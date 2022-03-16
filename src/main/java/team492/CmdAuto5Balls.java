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

class CmdAuto5Balls implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto5Balls";

    private enum State
    {
        START_DELAY,
        PREPARE_TO_SHOOT,
        SHOOT,
        PICKUP_RING_BALLS,
        PICKUP_ONE_MORE,
        //ignore this for now
        PICKUP_FAR_BALLS,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
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
     */
    CmdAuto5Balls(Robot robot, FrcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName + ".event");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);
    }   //CmdAuto5Balls

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
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            robot.dashboard.displayPrintf(1, "State: %s", state);

            //sequence of states:
            //shoot no vision
            //pick up ball
            //drive to shooting spot
            //shoot with vision
            //done 
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    TrcPose2D startPose = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                        RobotParams.RED_START_POS_5_BALL: RobotParams.BLUE_START_POS_5_BALL;
                    robot.robotDrive.driveBase.setFieldPosition(startPose);
                    //quick reference - delete this once points finalized
                    //  RED_START_POS_5_BALL = new TrcPose2D(26, 89.3, 358.0);
                    //  BLUE_START_POS_5_BALL = new TrcPose2D(-25.3, -88.0, 177.0 );
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
                        numBallsShot = 1;
                        robot.shooter.prepareToShootWithVision(moduleName, event, ShootLoc.TarmacAuto);
                    }
                    else
                    {
                        numBallsShot += 2;
                        robot.shooter.prepareToShootWithVision(moduleName, event, true);
                    }
                    sm.waitForSingleEvent(event, State.SHOOT);
                    break;

                case SHOOT:
                    robot.shooter.shootAllBalls(moduleName, event);
                    //only shot one ball before, need to pickup the 2 ring balls
                    sm.waitForSingleEvent(event, numBallsShot == 1? State.PICKUP_RING_BALLS: State.DONE);
                    break;

                //case for picking up balls in the rings just outside tarmac
                case PICKUP_RING_BALLS:
                    //drive to point while running intake
                    //move on to drive to shoot position when intake has a ball
                    //Make sure to test this with raised intake first

                    robot.intake.pickup(event);
                    //TODO: need to check the points
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
                    //might have to change this if it makes ball catch on flywheel
                    sm.waitForSingleEvent(event, State.PREPARE_TO_SHOOT);
                    break;

                case PICKUP_ONE_MORE:
                    robot.intake.pickup();
                    sm.waitForSingleEvent(driveEvent, State.PREPARE_TO_SHOOT);
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

}   //class CmdAuto5Balls
