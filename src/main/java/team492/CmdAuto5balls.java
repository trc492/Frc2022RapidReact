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
import TrcCommonLib.trclib.TrcPath;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class CmdAuto5Balls implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto5Balls";

    private enum State
    {
        START_DELAY,
        PICKUP_SECOND_BALL,         // Picks up the second ball which is right in front of it.
        GO_TO_FIRST_SHOOT_POS,      // Drives behind third ball to shoot the two balls in the robot.
        SHOOT,                      // Shoots all balls in the robot.
        PICKUP_THIRD_BALL,          // Picks up the ball right in front of it and shoots it immediately.
        PICKUP_TERMINAL_BALL,       // Picks up the ball at the terminal.
        PICKUP_HUMAN_PLAYER_BALL,   // Spins the intake until the human player's ball is collected.
        GO_TO_FINAL_SHOOT_POS,      // Drives closer to target to shoot the final two balls in the robot.
        DONE
    }   //enum State

    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent driveEvent;
    private final TrcStateMachine<State> sm;
    private final TrcTimer timer;

    // Keeps track of the number of balls the robot has already shot
    private int numBallsShot = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAuto5Balls(Robot robot)
    {
        robot.globalTracer.traceInfo(
            moduleName, ">>> robot=%s, choices=%s", robot, robot.autoChoices);

        this.robot = robot;
        event = new TrcEvent(moduleName + ".event");
        timer = new TrcTimer(moduleName);
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
            TrcPath path;

            robot.dashboard.displayPrintf(8, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    TrcPose2D startPos = robot.autoChoices.getAlliance() == Alliance.Red?
                                            RobotParams.STARTPOS_RED_AUTO_5BALL: RobotParams.STARTPOS_BLUE_AUTO_5BALL;
                    robot.robotDrive.setFieldPosition(startPos, true);
                    //
                    // Do start delay if any.
                    //
                    double startDelay = robot.autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        sm.setState(State.PICKUP_SECOND_BALL);
                    }
                    else
                    {
                        sm.waitForSingleEvent(event, State.PICKUP_SECOND_BALL);
                        timer.set(startDelay, event);
                    }
                    break;

                case PICKUP_SECOND_BALL:
                    // Sets the flywheel speed to lower:2000, upper:2000 while picking up the ball right in front of it.
                    robot.shooter.setFlywheelValue(2000.0, 2000.0);
                    robot.intake.extend();
                    // After we pick up the ball, we can go to the first shoot position.
                    sm.waitForSingleEvent(event, State.GO_TO_FIRST_SHOOT_POS);
                    robot.intake.pickup(event);
                    robot.robotDrive.purePursuitDrive.start(
                        null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0, 40, 0));
                    break;

                case GO_TO_FIRST_SHOOT_POS:
                    robot.robotDrive.purePursuitDrive.cancel();
                    // Go right behind the third ball to pick up (closer to the tarmac) to shoot the first two balls
                    robot.intake.retract();
                    // Shoot after we get to the position we want to be
                    sm.waitForSingleEvent(event, State.SHOOT);
                    if (robot.autoChoices.getAlliance() == DriverStation.Alliance.Red)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            new TrcPose2D(166, 111 + 20, -125),
                            new TrcPose2D(166-5, 111-5, -125));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.pathPoint(RobotParams.BALLPOS_BLUE_2, 0.0, 0.0, 56));
                    }
                    break;

                case SHOOT:
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.intake.retract();
                    // The first and last time we are shooting, we shoot two balls in total (1 & 2, 4 & 5)
                    // The only time we shoot one ball by itself is the third ball (if we've only shot two so far)
                    numBallsShot += (numBallsShot == 2) ? 1 : 2;
                    // Decides the next state based on how many balls we've shot so far
                    sm.waitForSingleEvent(
                        event, numBallsShot == 2? State.PICKUP_THIRD_BALL:
                               numBallsShot == 3? State.PICKUP_TERMINAL_BALL: 
                               State.DONE);
                    robot.shooter.shootWithVision(moduleName, event);
                    break;

                case PICKUP_THIRD_BALL:
                    // Picks up the ball close to the tarmac, which is right in front of us
                    robot.intake.extend();
                    sm.waitForSingleEvent(event, State.SHOOT); // Shoot after picking up the ball
                    robot.intake.pickup(event);
                    if (robot.autoChoices.getAlliance() == DriverStation.Alliance.Red)
                    {
                        // path = robot.buildPath(
                        //     false,
                        //     robot.pathPoint(RobotParams.AUTO_5BALL_BALL2_RED, 0.0, 0.0, 60.0),
                        //     new TrcPose2D(0.0, -10.0, 180.0));
                        robot.robotDrive.purePursuitDrive.start(
                            null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            new TrcPose2D(166 - 20, 111 - 20, -125), // We move forward and then immediately back up to the same position
                            new TrcPose2D(166, 111, -125));
                    }
                    else
                    {
                        path = robot.buildPath(
                            false,
                            robot.pathPoint(RobotParams.BALLPOS_BLUE_2, 0.0, 0.0, 240.0),
                            new TrcPose2D(0.0, -10.0, 180.0));
                    }
                    // robot.robotDrive.purePursuitDrive.start(path, driveEvent, 0.0); //goes to the location of the third ball
                    break;

                case PICKUP_TERMINAL_BALL:
                    // Drives to the terminal and picks up the ball resting there
                    robot.intake.extend();
                    sm.waitForSingleEvent(event, State.DONE);//PICKUP_HUMAN_PLAYER_BALL); TODO
                    robot.intake.pickup(event);
                    // turns about halfway in the path in order to be facing the correct angle when picking up the ball
                    if (robot.autoChoices.getAlliance() == DriverStation.Alliance.Red)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            // Turns about halfway in the path in order to be facing the correct angle when picking up the ball
                            new TrcPose2D(200, 100, 0),
                            new TrcPose2D(260, 90, 45));
                    }
                    else
                    {
                        path = robot.buildPath(
                            false,
                            robot.pathPoint(RobotParams.BALLPOS_BLUE_7, 50.0, 50.0, 260.0),
                            robot.pathPoint(RobotParams.BALLPOS_BLUE_7, 0.0, 0.0, 230.0));
                    }
                    break;

                case PICKUP_HUMAN_PLAYER_BALL:
                    robot.robotDrive.purePursuitDrive.cancel();
                    // Runs intake until human player ball is inputted, then goes to shoot position
                    robot.intake.extend();
                    sm.waitForSingleEvent(event, State.GO_TO_FINAL_SHOOT_POS);
                    robot.intake.pickup(event); // When the ball is picked up we can shoot
                    break;

                case GO_TO_FINAL_SHOOT_POS:
                    // Drives about 8 ft closer to the goal in order to shoot with a value from the shootParams table
                    // Shoot once we arrive at the final shoot position
                    sm.waitForSingleEvent(driveEvent, State.SHOOT);
                    //TODO: buildPath() not working, use relative coordinates instead
                    path = robot.buildPath(
                        false,
                        new TrcPose2D(0.0, -94.44, 180.0));
                    robot.robotDrive.purePursuitDrive.start(path, driveEvent, 0.0);
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

}   //class CmdAuto5Balls
