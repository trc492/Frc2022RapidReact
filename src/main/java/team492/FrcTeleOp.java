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

import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcRobot.RunMode;

public class FrcTeleOp implements TrcRobot.RobotMode
{
    //
    // Global objects.
    //
    protected final Robot robot;

    public FrcTeleOp(Robot robot)
    {
        //
        // Create and initialize global object.
        //
        this.robot = robot;

    }   // FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Configure joysticks.
        //

        //
        // Initialize subsystems for TeleOp mode if necessary.
        //

    }   // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disable subsystems before exiting if necessary.
        //

    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase operation.
        //

        //
        // Analog control of subsystem is done here if necessary.
        //

        //
        // Update dashboard
        //
        robot.updateDashboard(RunMode.TELEOP_MODE);
    }   // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        //
        // Do subsystem auto-assist here if necessary.
        //

    } // runContinuous

    //
    // Implements FrcButtonHandler.
    //

    public void joystickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "Joystick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
        }
    }   // joystickButtonEvent

}   // class FrcTeleOp
