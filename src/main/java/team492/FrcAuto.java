/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
import TrcFrcLib.frclib.FrcChoiceMenu;

public class FrcAuto implements TrcRobot.RobotMode
{
    //
    // Auto strategy enums.
    //
    private enum AutoStrategy
    {
        DRIVE_FORWARD_10_FEET,
        CRAB_LEFT_5_FEET,
        DRIVE_BACKWARD_7_FEET,
        TURN_LEFT_90_DEGREES
    }


    //
    // Autonomous Menus.
    //

    //
    // Global objects.
    //
    private final Robot robot;
    private FrcChoiceMenu<AutoStrategy> autoStrategyMenu;
    private AutoStrategy strategyChoice = null;

    public FrcAuto(Robot robot)
    {
        //
        // Create and initialize global objects.
        //
        this.robot = robot;

        //
        // Create Autonomous Mode specific menus.
        //
        autoStrategyMenu = new FrcChoiceMenu<>("Autonomous Strategies:");

        //
        // Populate Autonomous Mode menus.
        //
        autoStrategyMenu.addChoice("Drive forward 10 feet", AutoStrategy.DRIVE_FORWARD_10_FEET);
        autoStrategyMenu.addChoice("Strafe left 5 feet", AutoStrategy.CRAB_LEFT_5_FEET);
        autoStrategyMenu.addChoice("Drive backward 7 feet", AutoStrategy.DRIVE_BACKWARD_7_FEET);
        autoStrategyMenu.addChoice("Turn left 90 degrees", AutoStrategy.TURN_LEFT_90_DEGREES);

    }   // FrcAuto

    //
    // Implements TrcRobot.RunMode.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Retrieve Auto choices.
        //
        strategyChoice = autoStrategyMenu.getCurrentChoiceObject();
        robot.dashboard.displayPrintf(1, "Autonomous strategy choice is %s", strategyChoice);

        //
        // Create Auto Cmd.
        //

    }   // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Stop Auto Cmd.
        //

    }   // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // Update Dashboard here.
        //
        robot.updateDashboard(RunMode.AUTO_MODE);
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        //
        // Run Auto Cmd.
        //
        
    } // runContinuous

} // class FrcAuto
