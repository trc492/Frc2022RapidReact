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

import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobotBattery;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcDashboard;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcMatchInfo;
import TrcFrcLib.frclib.FrcPdp;
import TrcFrcLib.frclib.FrcRobotBase;
import TrcFrcLib.frclib.FrcRobotBattery;
import TrcFrcLib.frclib.FrcXboxController;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobotBase
{
    class TestChoices
    {
        double driveTime;
        double drivePower;
        double driveDistance;
        double turnDegrees;
        double drivePowerLimit;
        TrcPidController.PidCoefficients tunePidCoeff;
    
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "driveTime=%.3f, drivePower=%.1f, driveDistance=%.1f, turnDegrees=%.1f, powerLimit=%.1f, PidCoeff=%s",
                driveTime, drivePower, driveDistance, turnDegrees, drivePowerLimit, tunePidCoeff);
        }   //toString

    }   //class TestChocies

    //
    // Global objects.
    //
    public final FrcDashboard dashboard = FrcDashboard.getInstance();
    public final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private boolean traceLogOpened = false;
    private TestChoices testChoices = new TestChoices();

    //
    // Inputs.
    //
    public FrcJoystick leftDriveStick, rightDriveStick;
    public FrcXboxController driverController;
    public FrcJoystick operatorStick;
    public FrcJoystick buttonPanel;
    public FrcJoystick switchPanel;
    //
    // Sensors.
    //
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public AnalogInput pressureSensor;
    //
    // DriveBase subsystem.
    //
    public RobotDrive robotDrive;

    //
    // Vision subsystem.
    //

    //
    // Other subsystem.
    //

    /**
     * Constructor.
     */
    public Robot()
    {
        super(RobotParams.GAME_NAME);
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
        if (RobotParams.Preferences.useXboxController)
        {
            driverController = new FrcXboxController("DriverController", RobotParams.XBOX_DRIVERCONTROLLER);
            operatorStick = new FrcJoystick("operatorStick", RobotParams.JSPORT_OPERATORSTICK);
            buttonPanel = new FrcJoystick("buttonPanel", RobotParams.JSPORT_BUTTON_PANEL);
            switchPanel = new FrcJoystick("switchPanel", RobotParams.JSPORT_SWITCH_PANEL);
            driverController.setLeftYInverted(true);
        }
        else
        {
            leftDriveStick = new FrcJoystick("DriverLeftStick", RobotParams.XBOX_DRIVERCONTROLLER);
            rightDriveStick = new FrcJoystick("DriverRightStick", RobotParams.XBOX_DRIVERCONTROLLER + 1);
            operatorStick = new FrcJoystick("operatorStick", RobotParams.JSPORT_OPERATORSTICK + 1);
            buttonPanel = new FrcJoystick("buttonPanel", RobotParams.JSPORT_BUTTON_PANEL + 1);
            switchPanel = new FrcJoystick("switchPanel", RobotParams.JSPORT_SWITCH_PANEL + 1);
            rightDriveStick.setYInverted(true);
        }
        operatorStick.setYInverted(false);

        //
        // Create and initialize sensors.
        //
        pdp = new FrcPdp(RobotParams.CANID_PDP);
        battery = new FrcRobotBattery(pdp);
        pressureSensor = new AnalogInput(RobotParams.AIN_PRESSURE_SENSOR);

        //
        // Create and initialize DriveBase subsystem.
        //
        robotDrive = new RobotDrive(this);

        //
        // Create and initialize Vision subsystem.
        //

        //
        // Create and initialize other subsystems.
        //

        //
        // AutoAssist commands.
        //

        pdp.registerEnergyUsedForAllUnregisteredChannels();

        dashboard.refreshKey("Test/DriveTime", 5.0);
        dashboard.refreshKey("Test/DrivePower", 0.2);
        dashboard.refreshKey("Test/DriveDistance", 6.0);
        dashboard.refreshKey("Test/TurnDegrees", 90.0);
        dashboard.refreshKey("Test/DrivePowerLimit", 0.5);
        dashboard.refreshKey("Test/TuneKp", RobotParams.GYRO_TURN_KP);
        dashboard.refreshKey("Test/TuneKi", RobotParams.GYRO_TURN_KI);
        dashboard.refreshKey("Test/TuneKd", RobotParams.GYRO_TURN_KD);
        dashboard.refreshKey("Test/TuneKf", 0.0);
        //
        // Create Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));
    }   //robotInit

    @Override
    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        final String funcName = "robotStartMode";
        //
        // Read FMS Match info.
        //
        FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();

        //
        // Start trace logging.
        //
        if (runMode != RunMode.DISABLED_MODE && RobotParams.Preferences.useTraceLog)
        {
            openTraceLog(matchInfo);
            setTraceLogEnabled(true);
        }
        globalTracer.traceInfo(
            funcName, "[%.3f] %s: ***** %s *****", TrcUtil.getModeElapsedTime(),
            matchInfo.eventDate, runMode);

        //
        // Start subsystems.
        //
        robotDrive.startMode(runMode, prevMode);

        //
        // Read Tune PID Coefficients if in TEST_MODE.
        //
        if (runMode == RunMode.AUTO_MODE || runMode == RunMode.TEST_MODE)
        {
            testChoices.driveTime = dashboard.getNumber("Test/DriveTime", 5.0);
            testChoices.drivePower = dashboard.getNumber("Test/DrivePower", 0.2);
            testChoices.driveDistance = dashboard.getNumber("Test/DriveDistance", 6.0);
            testChoices.turnDegrees = dashboard.getNumber("Test/TurnDegrees", 90.0);
            testChoices.drivePowerLimit = dashboard.getNumber("Test/DrivePowerLimit", 0.5);
            if (runMode == RunMode.TEST_MODE)
            {
                testChoices.tunePidCoeff = new TrcPidController.PidCoefficients(
                    dashboard.getNumber("Test/TuneKp", RobotParams.GYRO_TURN_KP),
                    dashboard.getNumber("Test/TuneKi", RobotParams.GYRO_TURN_KI),
                    dashboard.getNumber("Test/TuneKd", RobotParams.GYRO_TURN_KD),
                    dashboard.getNumber("Test/TuneKf", 0.0));
            }
        }
    }   //robotStartMode

    @Override
    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        final String funcName = "robotStopMode";
        //
        // Stop subsystems.
        //
        robotDrive.stopMode(runMode, nextMode);

        double totalEnergy = battery.getTotalEnergy();
        globalTracer.traceInfo(funcName, "TotalEnergy=%.3fWh (%.2f%%)", totalEnergy,
            totalEnergy * 100.0 / RobotParams.BATTERY_CAPACITY_WATT_HOUR);

        //
        // Stop trace logging.
        //
        setTraceLogEnabled(false);
        closeTraceLog();

    }   //robotStopMode

    public void updateDashboard(RunMode runMode)
    {

    }   //updateDashboard

    public void openTraceLog(FrcMatchInfo matchInfo)
    {
        if (RobotParams.Preferences.useTraceLog && !traceLogOpened)
        {
            String fileName = matchInfo.eventName != null?
                String.format(Locale.US, "%s_%s%03d", matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber):
                getCurrentRunMode().name();

            traceLogOpened = globalTracer.openTraceLog("/home/lvuser/tracelog", fileName);
        }
    }

    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            globalTracer.closeTraceLog();
            traceLogOpened = false;
        }
    }

    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            globalTracer.setTraceLogEnabled(enabled);
        }
    }

    /**
     * This method is typically called in the autonomous state machine to log the autonomous state info as a state
     * event in the trace log file. The logged event can be used to play back autonomous path movement.
     *
     * @param state specifies the current state of the state machine.
     */
    public void traceStateInfo(Object state)
    {
        final String funcName = "traceStateInfo";

        if (robotDrive != null)
        {
            StringBuilder msg = new StringBuilder();

            msg.append(String.format(Locale.US, "tag=\">>>>>\" state=\"%s\"", state));
            if (robotDrive.pidDrive.isActive())
            {
                TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
                TrcPose2D targetPose = robotDrive.pidDrive.getAbsoluteTargetPose();
                msg.append(" RobotPose=" + robotPose + " TargetPose=" + targetPose);
            }
            else if (robotDrive.purePursuitDrive.isActive())
            {
                TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
                TrcPose2D robotVel = robotDrive.driveBase.getFieldVelocity();
                TrcPose2D targetPose = robotDrive.purePursuitDrive.getTargetFieldPosition();
                msg.append(" RobotPose=" + robotPose +
                           " TargetPose=" + targetPose +
                           " vel=" + robotVel +
                           " Path=" + robotDrive.purePursuitDrive.getPath());
            }

            if (battery != null)
            {
                msg.append(String.format(
                    Locale.US, " volt=\"%.2fV(%.2fV)\"", battery.getVoltage(), battery.getLowestVoltage()));
            }

            globalTracer.logEvent(funcName, "StateInfo", "%s", msg);
        }
    }   //traceStateInfo

    //
    // Getters for sensor data.
    //

    public double getPressure()
    {
        return (pressureSensor.getVoltage() - 0.5) * 50.0;
    }   //getPressure

}   //class Robot
