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
import TrcCommonLib.trclib.TrcUtil;
import edu.wpi.first.wpilibj.DriverStation;

class CmdAuto2Ball implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto2Ball";
    private enum State
    {
        START_DELAY,
        SHOOT,
        PICKUP_BALL,
        DRIVE_TO_SHOOT_POSITION,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    Double expireTime; 
    private boolean shootWithVision = false; 
    private int pointNumber; 
    private final TrcPose2D[] path; 


    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAuto2Ball(Robot robot, FrcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName + ".event");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);

        // int startPos = autoChoices.getStartPos();
        path = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                     RobotParams.RED_2_BALL_PATH: RobotParams.BLUE_2_BALL_PATH;
        pointNumber = 0; 
        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
    }   //CmdAuto

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
                    TrcPose2D startPose = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                        RobotParams.RED_START_POS_2_BALL: RobotParams.BLUE_START_POS_2_BALL;
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
                        sm.setState(State.SHOOT);
                    }
                    else
                    {
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.SHOOT);
                        break;
                    }

                case SHOOT:
                    //BUGBUG: fix parameters by look up table on the StartPosition.
                    if(!shootWithVision){
                        //this is shooting first ball no vision 
                        robot.shooter.shootAllBallsNoVision(moduleName, event, 1900.0, 1700.0, 45.0);
                        sm.waitForSingleEvent(event, State.PICKUP_BALL);
                    }
                    else{
                        //this is second ball, after this we are done
                        robot.shooter.shootAllBallsWithVision(moduleName, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case PICKUP_BALL:
                    //drive to point while running intake
                    //move on to shoot state when intake has a ball 
                    //after first ball use vision to shoot 
                    robot.robotDrive.purePursuitDrive.start(null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), 
                        false, path[pointNumber]);
                    robot.intake.pickup(event);
                    shootWithVision = true; 
                    pointNumber++; 
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SHOOT_POSITION);
                    break;

                case DRIVE_TO_SHOOT_POSITION:
                    robot.robotDrive.purePursuitDrive.start(null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), 
                        false, path[pointNumber]);
                    sm.waitForSingleEvent(event, State.SHOOT);
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

}   //class CmdAuto