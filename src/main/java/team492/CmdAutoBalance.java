/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import javax.lang.model.util.ElementScanner14;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcEvent.Callback;
import TrcFrcLib.frclib.FrcAHRSGyro;

class CmdAutoBalance implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoPickup";

    private enum State
    {
        START_DELAY,
        MOVE_FORWARD,
        IMPULSE_BACK,
        DONE
    }

    private final Robot robot;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;

    public CmdAutoBalance(Robot robot)
    {
        this.robot = robot;
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);

    }

    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }

    @Override
    public void cancel()
    {
        robot.robotDrive.pidDrive.cancel();
        sm.stop();
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime) {
        State state = sm.checkReadyAndGetState();

        if (state == null) {
            robot.dashboard.displayPrintf(13, "State: disabled or waiting...");
        } else {
            String msg;
            robot.dashboard.displayPrintf(13, "State: %s", state);
            switch (state) {
                case START_DELAY:
                    // if (autoChoices.getStartDelay() == 0.0) {
                        sm.setState(State.MOVE_FORWARD);
                    // } else {
                    //     timer.set(autoChoices.getStartDelay(), new TrcEvent.Callback() {
                    //         @Override
                    //         public void notify(Object context) {
                    //             sm.setState(State.MOVE_FORWARD);
                    //         }
                    //     });
                    // }
                    break;

                case MOVE_FORWARD:
                    robot.robotDrive.pidDrive.driveMaintainHeading(0.0, 0.1, 0.0);
                    if(Math.abs(((FrcAHRSGyro)robot.robotDrive.gyro).ahrs.getRoll()) < 1.0) {
                        sm.setState(State.IMPULSE_BACK);
                    }
                    break;

                case IMPULSE_BACK:
                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.4, 0.0);
                    timer.set(0.05, new TrcEvent.Callback() {
                        @Override
                        public void notify(Object context) {
                            sm.setState(State.DONE);
                        }
                    });
                    break;

                case DONE:
                default:
                    robot.robotDrive.setAntiDefenseEnabled("CmdAutoBalanec", true);
                    cancel();
                 
                   break;
            }
        }
        return !isActive();
    }
}