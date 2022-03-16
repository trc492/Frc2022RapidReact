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

class CmdAuto2Balls implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto2Balls";

    private enum State
    {
        START_DELAY,
        PREPARE_TO_SHOOT,
        SHOOT,
        PICKUP_BALL,
        DRIVE_TO_SHOOT_POSITION,
        //TODOL implement when i have time
        PICKUP_ENEMY_BALL,
        DEPOSIT_ENEMY_BALL,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private boolean shootWithVision = false; 
    private int pointNumber; 

    private  TrcPose2D[] path; 
    Double expireTime; 
    private boolean pidOnly = false;
    //may convert this into an autochoice  
    private boolean pickupFirst = true; 
    State nextState = null; 

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAuto2Balls(Robot robot, FrcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName + ".event");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);

        // int startPos = autoChoices.getStartPos();
        if(!pidOnly){
            path = autoChoices.getAlliance() == DriverStation.Alliance.Red?
            RobotParams.RED_2_BALL_PATH: RobotParams.BLUE_2_BALL_PATH;
        }

        pointNumber = 0; 
        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.2);
    }   //CmdAuto2Balls

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
            boolean traceState = true;

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
                    if(pickupFirst){
                        robot.intake.extend();
                        TrcPose2D startPose = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                            RobotParams.RED_START_POS_2_BALL_PICKUP_FIRST: RobotParams.BLUE_START_POS_2_BALL_PICKUP_FIRST;
                            robot.robotDrive.driveBase.setFieldPosition(startPose);
                        nextState = State.PICKUP_BALL; 
                    }
                    else{
                        TrcPose2D startPose = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                        RobotParams.RED_START_POS_2_BALL: RobotParams.BLUE_START_POS_2_BALL;
                        robot.robotDrive.driveBase.setFieldPosition(startPose); 
                        //shootWithVision is false
                        nextState = State.PREPARE_TO_SHOOT;
                    }
                    //
                    // Do start delay if any.
                    //
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(nextState);
                    }
                    else
                    {
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                break; 

                case PREPARE_TO_SHOOT:
                    robot.dashboard.displayPrintf(15, "PREPARING TO SHOOT");
                    if (!shootWithVision)
                    {
                        //this is shooting first ball no vision
                        robot.shooter.prepareToShootWithVision(moduleName, event, "tarmac_auto");
                    }
                    else
                    {
                        //this is second ball, after this we are done
                        //TODO change this preset
                        robot.shooter.prepareToShootWithVision(moduleName, event, "tarmac_auto");
                    }
                    sm.waitForSingleEvent(event, State.SHOOT);
                    break;

                case SHOOT:
                    robot.shooter.shootAllBalls(moduleName, event);
                    sm.waitForSingleEvent(event, shootWithVision? State.DONE: State.PICKUP_BALL);
                    break;

                case PICKUP_BALL:
                    //drive to point while running intake
                    //move on to drive to shoot position when intake has a ball 
                    //shootWithVision  true because next ball(s) shot with vision
                    if(pidOnly){
                        robot.robotDrive.pidDrive.setRelativeYTarget(40, null);
                    }
                    else{
                        robot.robotDrive.purePursuitDrive.start(
                            null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true, new TrcPose2D(0, 36, 0));
                        pointNumber++; 

                    }
       
                    robot.intake.pickup(event);
                    shootWithVision = true; 
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SHOOT_POSITION);
                    break;

                case DRIVE_TO_SHOOT_POSITION:
                    //if pid only this is just turning to face the goal 
                    if(pidOnly){
                        //adding 180 to turn makes the robot face target pretty well for red and blue 
                        double turnDelta = 180; 
                        robot.robotDrive.pidDrive.setRelativeTurnTarget(turnDelta, event);
                    }
                    else{
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true, new TrcPose2D(0, 0, 180));
                    }
                    sm.waitForSingleEvent(event, State.PREPARE_TO_SHOOT);

                    break; 

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    cancel();
                    break;
            }

            if (traceState)
            {
                robot.globalTracer.traceStateInfo(
                    state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto2Balls