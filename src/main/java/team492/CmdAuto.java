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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class CmdAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto";

    private enum State
    {
        START_DELAY,
        PICKUP_BALL,
        DRIVE_TO_SHOOT_POSITION,
        SHOOT,  
        DONE
 
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent event2; 
    private final TrcStateMachine<State> sm;
    private TrcPose2D[] path; 
    //pathIndex is which point we are driving to 
    private Integer pathIndex; 
    private int pathNumber; 

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAuto(Robot robot, FrcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        event2 = new TrcEvent("event2");
        if(autoChoices.getAlliance()==DriverStation.Alliance.Red){
            switch(autoChoices.getStartPos()){
                case 1:
                    path = RobotParams.RED_1_PATH; 
                case 2:
                    path = RobotParams.RED_2_PATH;
                case 3: 
                    path = RobotParams.RED_3_PATH; 
                break; 
            }
        }
        else{
            switch(autoChoices.getStartPos()){
                case 1:
                    path = RobotParams.BLUE_1_PATH; 
                case 2:
                    path = RobotParams.BLUE_2_PATH;
                case 3: 
                    path = RobotParams.BLUE_3_PATH; 
                break; 
            }
        }
        sm = new TrcStateMachine<>(moduleName);
        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
        sm.start(State.START_DELAY);
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
            TrcPose2D startPose;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    startPose = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                        RobotParams.RED_START_POSES[autoChoices.getStartPos()]:
                        RobotParams.BLUE_START_POSES[autoChoices.getStartPos()];
                    robot.robotDrive.driveBase.setFieldPosition(startPose);
                    robot.intake.acquireExclusiveAccess(moduleName);
                    //
                    // Do start delay if any.
                    //
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.PICKUP_BALL);
                    }
                    else
                    {
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.PICKUP_BALL);
                        break;
                    }

                case PICKUP_BALL:
                    //drive to the ball while running the intake 
                    robot.robotDrive.purePursuitDrive.start(event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false, path[0]);
                    robot.intake.pickup(moduleName, event2);
                    //when robot arrived at location and intake event is signaled, move on to shoot event 
                    sm.addEvent(event);
                    sm.addEvent(event2);
                    sm.waitForEvents(State.SHOOT, 0.0, true); 
                break; 

                case SHOOT:
                    //use the auto assist shooter to handle aiming and shooting both balls in the robot 
                    robot.shooter.shootAllBalls(event);
                    sm.waitForSingleEvent(event, State.DONE); 
                break; 
                
                case DONE:
                default:
                    //
                    // We are done, zero calibrate the arm will lower it.
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