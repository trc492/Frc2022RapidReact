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

import TrcCommonLib.trclib.TrcDashboard;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcRobotBase;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobotBase
{
    //
    // Robot preferences.
    //

    //
    // Global constants.
    //
    public static final String programName = "FrcTemplate";
 
    //
    // Global objects.
    //
    public final TrcDashboard dashboard = TrcDashboard.getInstance();

    //
    // Inputs.
    //

    //
    // Sensors.
    //

    //
    // DriveBase subsystem.
    //

    //
    // Vision subsystem.
    //

    //
    // Miscellaneous subsystem.
    //

    //
    // FMS Match info.
    //

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize global objects.
        //

        //
        // Create and initialize inputs.
        //

        //
        // Create and initialize sensors.
        //

        //
        // Create and initialize DriveBase subsystem.
        //

        //
        // Create PID controllers for DriveBase PID drive.
        //

        //
        // Create and initialize Vision subsystem.
        //

        //
        // Create and initialize other subsystems.
        //

        //
        // AutoAssist commands.
        //

        //
        // Create Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));

    }   //robotInit

    @Override
    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        //
        // Read FMS Match info.
        //

        //
        // Start trace logging.
        //

        //
        // Start subsystems.
        //

        //
        // Read Tune PID Coefficients if in TEST_MODE.
        //

    }   //robotStartMode

    @Override
    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        //
        // Stop subsystems.
        //

        //
        // Stop trace logging.
        //

    }   //robotStopMode

    public void updateDashboard(RunMode runMode)
    {

    }   //updateDashboard

}   //class Robot
