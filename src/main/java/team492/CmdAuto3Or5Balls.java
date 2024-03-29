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

class CmdAuto3Or5Balls implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto3Or5Balls";
    private static final double PPD_DRIVE_FAST_LIMIT = 0.8;

    private enum State
    {
        START_DELAY,
        PICKUP_SECOND_BALL,         // Picks up the second ball which is right in front of it.
        GO_TO_FIRST_SHOOT_POS,      // Drives behind third ball to shoot the two balls in the robot.
        SHOOT,                      // Shoots all balls in the robot.
        PICKUP_THIRD_BALL,          // Picks up the ball right in front of it and shoots it immediately.
        DRIVE_TO_TERMINAL,
        PICKUP_TERMINAL_BALL,       // Picks up the ball at the terminal.
        PICKUP_HUMAN_PLAYER_BALL,   // Spins the intake until the human player's ball is collected.
        GO_TO_FINAL_SHOOT_POS,      // Drives closer to target to shoot the final two balls in the robot.
        DONE
    }   //enum State

    private final Robot robot;
    private final boolean do5Balls;
    private final TrcEvent timerEvent;
    private final TrcEvent driveEvent;
    private final TrcEvent intakeEvent;
    private final TrcEvent shootEvent;
    private final TrcStateMachine<State> sm;
    private final TrcTimer timer;

    // Keeps track of the number of balls the robot has already shot
    private int numBallsShot = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param do5Balls specifies true to shoot 5 balls, false to shoot 3 balls.
     */
    CmdAuto3Or5Balls(Robot robot, boolean do5Balls)
    {
        robot.globalTracer.traceInfo(
            moduleName, ">>> robot=%s, do5Balls=%s, choices=%s", robot, do5Balls, FrcAuto.autoChoices);

        this.robot = robot;
        this.do5Balls = do5Balls;
        timer = new TrcTimer(moduleName);
        timerEvent = new TrcEvent(moduleName + ".timerEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        intakeEvent = new TrcEvent(moduleName + ".intakeEvent");
        shootEvent = new TrcEvent(moduleName + ".shootEvent");
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
            robot.dashboard.displayPrintf(9, "State: disabled or waiting (nextState=%s)...", sm.getNextState());
        }
        else
        {
            robot.dashboard.displayPrintf(9, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    double wallAngle = robot.wallAlignSensor != null? robot.wallAlignSensor.getAngleToWall(): Double.NaN;
                    TrcPose2D startPos = RobotParams.STARTPOS_AUTO_5BALL.clone();
                    // Check if the wall angle is reasonable. If not, don't trust it.
                    if (wallAngle > RobotParams.WALL_ALIGN_LOW_THRESHOLD &&
                        wallAngle < RobotParams.WALL_ALIGN_HIGH_THRESHOLD)
                    {
                        startPos.angle = 90.0 + wallAngle;
                    }
                    robot.robotDrive.setFieldPosition(startPos, false);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(PPD_DRIVE_FAST_LIMIT);
                    //
                    // Do start delay if any.
                    //
                    double startDelay = FrcAuto.autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        sm.setState(State.PICKUP_SECOND_BALL);
                    }
                    else
                    {
                        sm.waitForSingleEvent(timerEvent, State.PICKUP_SECOND_BALL);
                        timer.set(startDelay, timerEvent);
                    }
                    break;

                case PICKUP_SECOND_BALL:
                    // Sets flywheel speed to the expected velocities while picking up the ball right in front of it.
                    // This drastically improves the latency for the first shoot.
                    robot.shooter.setFlywheelValue(1900.0, 2500.0);
                    robot.intake.extend();
                    // After we pick up the ball, we can go to the first shoot position.
                    //sm.waitForSingleEvent(intakeEvent, State.GO_TO_FIRST_SHOOT_POS);
                    robot.intake.pickup(intakeEvent);
                    // Move forward to pick up the ball right in front of the robot, stop as soon as pickup event triggers.
                    robot.robotDrive.purePursuitDrive.start(
                        null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 40.0, 0.0));
                    //sm.addEvent(driveEvent);
                    sm.addEvent(intakeEvent);
                    sm.waitForEvents(State.GO_TO_FIRST_SHOOT_POS, 1.25);
                    break;

                case GO_TO_FIRST_SHOOT_POS:
                    // Cancel the previous PurePursuit drive.
                    robot.robotDrive.purePursuitDrive.cancel();
                    // Go right behind the third ball to pick up (closer to the tarmac) to shoot the first two balls.
                    sm.waitForSingleEvent(driveEvent, State.SHOOT);
                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 2.5, robot.robotDrive.driveBase.getFieldPosition(), false,
                        new TrcPose2D(125.7, -147.7, 0),
                        robot.ballPickupPoint(RobotParams.BALLPOS_2, 35, -35.25, -15.0, 0.0));
                    robot.intake.retract();
                    break;

                case SHOOT:
                    robot.robotDrive.purePursuitDrive.cancel();
                    if(numBallsShot == 0) { robot.intake.extend(); }
                    //robot.intake.retract();
                    // The first and last time we are shooting, we shoot two balls in total (1 & 2, 4 & 5)
                    // The only time we shoot one ball by itself is the third ball (if we've only shot two so far)
                    numBallsShot += (numBallsShot == 0 || numBallsShot == 3) ? 2 : 1;
                    // Decides the next state based on how many balls we've shot so far
                    sm.waitForSingleEvent(
                        shootEvent, numBallsShot == 2? State.PICKUP_THIRD_BALL:
                                    numBallsShot == 3 && do5Balls? State.DRIVE_TO_TERMINAL: State.DONE);
                    robot.shooter.shootWithVision(moduleName, shootEvent);
                    break;

                case PICKUP_THIRD_BALL:
                    // robot.intake.extend();
                    // Picks up the ball close to the tarmac, which is right in front of us
                    // Start setting flywheel velocity to be the one we use for the 3rd ball
                    robot.shooter.setFlywheelValue(2228, 1900);
                    // robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    sm.waitForSingleEvent(intakeEvent, State.SHOOT, 1.5); // Shoot after picking up the ball
                    robot.intake.pickup(intakeEvent);
                    robot.robotDrive.purePursuitDrive.start(
                        null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        new TrcPose2D(0.0, 40.0, 0.0));
                    // robot.robotDrive.
                    break;

                case DRIVE_TO_TERMINAL:
                    // Drives to the terminal and picks up the ball resting there
                    // Set flywheels to the velocity it uses for shooting the 4th and 5th ball
                    robot.shooter.setFlywheelValue(1900, 2600); // 2228, 1886
                    robot.intake.extend();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(PPD_DRIVE_FAST_LIMIT);
                    robot.intake.pickup(intakeEvent);
                    // robot.robotDrive.purePursuitDrive.start(
                    //     null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                    //     robot.ballPickupPoint(
                    //         RobotParams.BALLPOS_7, RobotParams.INTAKE_OFFSET + 30.0, 157.35, -2.0, 0.0),
                    //     robot.ballPickupPoint(
                    //         RobotParams.BALLPOS_7, RobotParams.INTAKE_OFFSET - 10.0, 157.35, -2.0, 0.0));
                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 2.5, robot.robotDrive.driveBase.getFieldPosition(), false,
                        new TrcPose2D(58.5, -219, 136), //63.5, -219, 136
                        new TrcPose2D(91.6, -263.8, 136));
                    sm.addEvent(driveEvent);
                    sm.addEvent(intakeEvent);
                    sm.waitForEvents(State.PICKUP_TERMINAL_BALL);
                    break;

                case PICKUP_TERMINAL_BALL:
                    // if there is a ball, don't drive forward so go to pickup human player ball state
                    if (robot.conveyor.isExitSensorActive() || robot.conveyor.isEntranceSensorActive())
                    {
                        sm.setState(State.PICKUP_HUMAN_PLAYER_BALL);
                    }
                    else
                    {
                        //move forward slowly until robot has a ball
                        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.1);
                        robot.robotDrive.purePursuitDrive.start(
                            null, robot.robotDrive.driveBase.getFieldPosition(), true, new TrcPose2D(0, 24, 0));

                        sm.waitForSingleEvent(intakeEvent, State.PICKUP_HUMAN_PLAYER_BALL);
                        robot.intake.pickup(intakeEvent);
                    }
                    break;

                case PICKUP_HUMAN_PLAYER_BALL:
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.robotDrive.purePursuitDrive.start(
                        null, robot.robotDrive.driveBase.getFieldPosition(), true, new TrcPose2D(0.0, -12.0, 0));
                    // Human player ball is the amount of time robot spends at the terminal position
                    double humanBallTimeout = 11.5 - elapsedTime;
                    robot.globalTracer.traceInfo(moduleName, "HumanBall timeout=%.3f", humanBallTimeout);
                    robot.dashboard.displayPrintf(14, "HumanBall timeout: %.3f", humanBallTimeout);
                    if (humanBallTimeout < 0)
                    {
                        sm.setState(State.GO_TO_FINAL_SHOOT_POS);
                    }
                    else
                    {
                        // Runs intake until human player ball is inputted, then goes to shoot position
                        sm.waitForSingleEvent(intakeEvent, State.GO_TO_FINAL_SHOOT_POS, humanBallTimeout);
                        robot.intake.pickup(intakeEvent); // When the ball is picked up we can shoot
                    }
                    break;

                case GO_TO_FINAL_SHOOT_POS:
                    robot.intake.stop();
                    // Drives about 8 ft closer to the goal in order to shoot with a value from the shootParams table
                    // Shoot once we arrive at the final shoot position
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.8);
                    sm.waitForSingleEvent(driveEvent, State.SHOOT);
                    robot.robotDrive.purePursuitDrive.start(
                        driveEvent, 2.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        new TrcPose2D(81.0, -180.8, -20.0));
                    robot.intake.retract();
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.dashboard.displayPrintf(13, "Done: %.3f", elapsedTime);
                    cancel();
                    break;
            }
    
            robot.globalTracer.traceStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }
    
        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto3Or5Balls
