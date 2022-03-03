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

class CmdAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAuto";

    private enum State
    {
        START_DELAY,
        SHOOT_TEMP,
        TIMED_DRIVE_TEMP,

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
    Double expireTime; 

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
        event = new TrcEvent(moduleName + ".event");
        event2 = new TrcEvent(moduleName + ".event2");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);

        int startPos = autoChoices.getStartPos();
        path = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                    RobotParams.RED_PATHS[startPos]: RobotParams.BLUE_PATHS[startPos];

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
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    int pos = autoChoices.getStartPos();
                    TrcPose2D startPose = autoChoices.getAlliance() == DriverStation.Alliance.Red?
                        RobotParams.RED_START_POSES[pos]: RobotParams.BLUE_START_POSES[pos];
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
                        sm.setState(State.SHOOT_TEMP);//PICKUP_BALL);
                    }
                    else
                    {
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.SHOOT_TEMP);//PICKUP_BALL);
                        break;
                    }
                
                case SHOOT_TEMP:
                    robot.shooter.shootAllBalls(moduleName, event); 
                    sm.waitForSingleEvent(event, State.TIMED_DRIVE_TEMP);
                break; 
                case TIMED_DRIVE_TEMP:
                    if(expireTime==null){
                        expireTime = TrcUtil.getCurrentTime() + 2.0; 
                    }
                    else if(TrcUtil.getCurrentTime()>=expireTime){
                        expireTime = null; 
                        sm.setState(State.DONE);
                    }
                    robot.robotDrive.driveBase.holonomicDrive(moduleName, 0.0, -0.5, 0.0);
                break; 
                // CodeReview: should shoot the preloaded ball first.
                case PICKUP_BALL:
                    //drive to the ball while running the intake 
                    robot.robotDrive.purePursuitDrive.start(
                        event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false, path[0]);
                    robot.intake.pickup(event2);
                    //when robot arrived at location and intake event is signaled, move on to shoot event 
                    sm.addEvent(event);
                    sm.addEvent(event2);
                    sm.waitForEvents(State.SHOOT, 0.0, true); 
                    break;

                case SHOOT:
                    //use the auto assist shooter to handle aiming and shooting both balls in the robot 
                    robot.shooter.shootAllBalls("autoShoot", event);
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