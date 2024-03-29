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
import TrcCommonLib.trclib.TrcPath;
import TrcCommonLib.trclib.TrcPathBuilder;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobotBattery;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcWaypoint;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcDashboard;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcMatchInfo;
import TrcFrcLib.frclib.FrcPdp;
import TrcFrcLib.frclib.FrcRemoteVisionProcessor;
import TrcFrcLib.frclib.FrcRobotBase;
import TrcFrcLib.frclib.FrcRobotBattery;
import TrcFrcLib.frclib.FrcXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import team492.ShootParamTable.ShootLoc;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobotBase
{
    //
    // Global objects.
    //
    public final FrcDashboard dashboard = FrcDashboard.getInstance();
    public final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private double nextDashboardUpdateTime = TrcUtil.getModeElapsedTime();
    private boolean traceLogOpened = false;

    //
    // Inputs.
    //
    public FrcJoystick leftDriveStick, rightDriveStick;
    public FrcJoystick operatorStick;
    public FrcJoystick buttonPanel;
    public FrcJoystick switchPanel;
    public FrcXboxController driverController;

    //
    // Sensors.
    //
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public AnalogInput pressureSensor;  // TO-DO: change to use the REV PCM to read the pressure sensor.
    public WallAlignSensor wallAlignSensor;

    //
    // Miscellaneous hardware.
    //
    public LEDIndicator ledIndicator;

    //
    // Vision subsystem.
    //
    public VisionTargeting vision;
    // Old set before tuning at Royal.
    public final ShootParamTable shootParamTable = new ShootParamTable()
        .add(ShootLoc.Tower,        71.83, 2700, 1200, RobotParams.TILTER_CLOSE_ANGLE)
        .add(ShootLoc.Distance7ft,  84.0, 2500, 1400, RobotParams.TILTER_CLOSE_ANGLE) //Tune?
        .add(ShootLoc.TarmacAuto,   95.4, 2200, 1500, RobotParams.TILTER_CLOSE_ANGLE)
        .add(ShootLoc.TarmacMid,    107.0, 2100, 1600, RobotParams.TILTER_CLOSE_ANGLE)
        .add(ShootLoc.Distance10ft, 120.0, 1900, 2000, RobotParams.TILTER_CLOSE_ANGLE)
        .add(ShootLoc.FarThreshold, 132.0, 2000, 2100, RobotParams.TILTER_CLOSE_ANGLE) //Tune inflection?
        .add(ShootLoc.OldThreshold, 132.0001, 2550, 1300, RobotParams.TILTER_FAR_ANGLE)
        .add(ShootLoc.StartFar,     146.0, 2450, 1500, RobotParams.TILTER_FAR_ANGLE)
        .add(ShootLoc.RingMid,      150.0, 2300, 1800, RobotParams.TILTER_FAR_ANGLE)
        .add(ShootLoc.Distance13ft, 156.0, 2200, 1900, RobotParams.TILTER_FAR_ANGLE)
        .add(ShootLoc.Distance14ft, 168.0, 2100, 2000, RobotParams.TILTER_FAR_ANGLE)
        .add(ShootLoc.Distance15ft, 180.0, 1900, 2200, RobotParams.TILTER_FAR_ANGLE)
        .add(ShootLoc.LaunchPad,    186.0, 1900, 2300, RobotParams.TILTER_FAR_ANGLE)
        .add(ShootLoc.Distance17ft, 204.0, 1800, 2600, RobotParams.TILTER_FAR_ANGLE) //double check
        .add(ShootLoc.Distance18ft, 216.0, 1700, 2800, RobotParams.TILTER_FAR_ANGLE);
    // public final ShootParamTable shootParamTable = new ShootParamTable()
    //     .add(ShootLoc.Tower,         71.83, 2600, 900, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.Distance7ft,   84.0, 2500, 1400, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.TarmacAuto,    95.4, 2200, 1500, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.TarmacMid,     107.0, 2200, 1300, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.Distance12ft,  120.0, 2100, 1400, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.OldThreshold,  132.0, 1800, 2000, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.TarmacEdge,    146.0, 1800, 2200, RobotParams.TILTER_CLOSE_ANGLE) //1922
    //     .add(ShootLoc.RingMid,       150.0, 1700, 2300, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.Distance156in, 156.0, 1700, 2300, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.Distance162in, 162.0, 1700, 2400, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.Distance170in, 170.0, 1800, 2500, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.FarThreshold, 179.9999, 2000, 2600, RobotParams.TILTER_CLOSE_ANGLE)
    //     .add(ShootLoc.Distance15ft,  180.0, 1800, 2400, RobotParams.TILTER_FAR_ANGLE)
    //     .add(ShootLoc.LaunchPad,     186.0, 2000, 2300, RobotParams.TILTER_FAR_ANGLE)
    //     .add(ShootLoc.Distance195in, 195.0, 1900, 2400, RobotParams.TILTER_FAR_ANGLE)
    //     .add(ShootLoc.Distance17ft,  204.0, 1900, 2600, RobotParams.TILTER_FAR_ANGLE)
    //     .add(ShootLoc.Distance18ft,  216.0, 1900, 2700, RobotParams.TILTER_FAR_ANGLE)
    //     .add(ShootLoc.Distance250in, 250.0, 1900, 3000, RobotParams.TILTER_FAR_ANGLE)
    //     .add(ShootLoc.Distance275in, 275.0, 1800, 3600, RobotParams.TILTER_FAR_ANGLE)
    //     .add(ShootLoc.Distance300in, 300.0, 1800, 3900, RobotParams.TILTER_FAR_ANGLE);
    //tune100again194190

    //
    // DriveBase subsystem.
    //
    public SwerveDrive robotDrive;

    //
    // Other subsystems.
    //
    public Conveyor conveyor;
    public Intake intake;
    public Shooter shooter;
    public Climber climber;

    //
    // Miscellaneous.
    //
    public double lowerFlywheelUserVel;
    public double upperFlywheelUserVel;

    /**
     * Constructor: Create an instance of the object.
     */
    public Robot()
    {
        super(RobotParams.ROBOT_NAME);
    }   //Robot

    /**
     * This method is called when the robot is first started up and should be used for any initialization code
     * including creation and initialization of all robot hardware and subsystems.
     *
     * To create new hardware or subsystem, follow the steps below:
     * 1. Create a public class variable for the new hardware/subsystem.
     * 2. Instantiate and initialize the new hardware/subsystem object in this method.
     * 3. Put code in updateDashboard to display status of the new hardware/subsystem if necessary.
     * 4. Put code in robotStartMode or robotStopMode to configure/reset hardware/subsystem if necessary.
     * 5. Put code in FrcTeleOp to operate the subsystem if necessary (i.e. slowPeriodic/xxxButtonEvent).
     * 6. Create a getter method for the new sensor only if necessary (e.g. sensor value needs translation).
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
            driverController.setLeftYInverted(true);
        }
        else
        {
            leftDriveStick = new FrcJoystick("DriverLeftStick", RobotParams.JSPORT_DRIVER_LEFTSTICK);
            leftDriveStick.setYInverted(true);
            rightDriveStick = new FrcJoystick("DriverRightStick", RobotParams.JSPORT_DRIVER_RIGHTSTICK);
            rightDriveStick.setYInverted(true);
        }
        operatorStick = new FrcJoystick("operatorStick", RobotParams.JSPORT_OPERATORSTICK);
        operatorStick.setYInverted(false);
        if (RobotParams.Preferences.useButtonPanels)
        {
            buttonPanel = new FrcJoystick("buttonPanel", RobotParams.JSPORT_BUTTON_PANEL);
            switchPanel = new FrcJoystick("switchPanel", RobotParams.JSPORT_SWITCH_PANEL);
        }

        //
        // Create and initialize sensors.
        //
        if (RobotParams.Preferences.useStreamCamera)
        {
            UsbCamera camera = CameraServer.startAutomaticCapture("DriverDisplay", 0);
            camera.setResolution(160, 120);
            camera.setFPS(10);
        }

        if (RobotParams.Preferences.usePdp)
        {
            pdp = new FrcPdp(RobotParams.CANID_PDP, ModuleType.kRev);
            pdp.setSwitchableChannel(false);
            battery = new FrcRobotBattery(pdp);
        }

        pressureSensor = new AnalogInput(RobotParams.AIN_PRESSURE_SENSOR);
        if (RobotParams.Preferences.useWallAlignSensor)
        {
            wallAlignSensor = new WallAlignSensor();
        }

        //
        // Create and initialize miscellaneous hardware.
        //
        ledIndicator = new LEDIndicator();

        //
        // Create and initialize Vision subsystem.
        //
        if (RobotParams.Preferences.useVision)
        {
            vision = new VisionTargeting();
        }

        //
        // Create and initialize RobotDrive subsystem.
        //
        robotDrive = new SwerveDrive(this);

        //
        // Create and initialize other subsystems.
        //
        if (RobotParams.Preferences.useSubsystems)
        {
            // Intake needs Conveyor, so Conveyor must be created before Intake.
            conveyor = new Conveyor();
            // conveyor.setMsgTracer(globalTracer);

            intake = new Intake(this);
            // intake.setMsgTracer(globalTracer);
            intake.retract();

            climber = new Climber(this);
            climber.setMsgTracer(globalTracer);
            climber.climberPneumatic.retract();
            climber.zeroCalibrateClimber();

            shooter = new Shooter(this);
            shooter.setMsgTracer(globalTracer);
        }

        //
        // Miscellaneous.
        //
        if (pdp != null)
        {
            pdp.registerEnergyUsedForAllUnregisteredChannels();
        }

        //
        // Create Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));
    }   //robotInit

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
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
        if (runMode == RunMode.TELEOP_MODE || runMode == RunMode.TEST_MODE)
        {
            if (pdp != null)
            {
                pdp.setSwitchableChannel(switchPanel.isButtonPressed(FrcJoystick.PANEL_SWITCH_RED1));
            }
            shooter.setVisionAlignNoOscillation(true);
            shooter.setVisionAlignEnabled(true);
        }
        else if (runMode == RunMode.AUTO_MODE)
        {
            shooter.setVisionAlignNoOscillation(false);
            shooter.setVisionAlignEnabled(true);
        }

        if (vision != null && runMode != RunMode.DISABLED_MODE)
        {
            vision.setEnabled(true);
        }

        robotDrive.startMode(runMode, prevMode);
        ledIndicator.reset();
    }   //robotStartMode

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    @Override
    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        final String funcName = "robotStopMode";

        //
        // Stop subsystems.
        //
        if (vision != null)
        {
            vision.setEnabled(false);
        }

        robotDrive.stopMode(runMode, nextMode);
        if (RobotParams.Preferences.useSubsystems)
        {
            shooter.cancel();
            climber.cancel();
            intake.cancel();
            conveyor.cancel();
        }
        ledIndicator.reset();

        if (runMode == RunMode.TELEOP_MODE && pdp != null)
        {
            pdp.setSwitchableChannel(false);
        }
        //
        // Performance status report.
        //
        if (battery != null)
        {
            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(
                funcName, "TotalEnergy=%.3fWh (%.2f%%)",
                totalEnergy, totalEnergy * 100.0 / RobotParams.BATTERY_CAPACITY_WATT_HOUR);
        }

        if (runMode != RunMode.DISABLED_MODE)
        {
            printPerformanceMetrics(globalTracer);
        }

        //
        // Stop trace logging.
        //
        setTraceLogEnabled(false);
        closeTraceLog();
    }   //robotStopMode

    /**
     * This method is called periodically to update various hardware/subsystem status of the robot to the dashboard
     * and trace log. In order to lower the potential impact these updates, this method will only update the dashboard
     * at DASHBOARD_UPDATE_INTERVAL.
     */
    public void updateStatus()
    {
        final String funcName = "updateStatus";
        double currTime = TrcUtil.getModeElapsedTime();
        RunMode runMode = getCurrentRunMode();

        if (currTime >= nextDashboardUpdateTime)
        {
            nextDashboardUpdateTime = currTime + RobotParams.DASHBOARD_UPDATE_INTERVAL;

            if (RobotParams.Preferences.debugPowerConsumption)
            {
                if (pdp != null)
                {
                    dashboard.putNumber("Power/pdpTotalCurrent", pdp.getTotalCurrent());
                    dashboard.putNumber("Power/totalEnergy", battery.getTotalEnergy());
                    dashboard.putData("Power/pdpInfo", pdp.getPdpSendable());
                    if (runMode == RunMode.TELEOP_MODE)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] Battery: currVoltage=%.2f, lowestVoltage=%.2f",
                            currTime, battery.getVoltage(), battery.getLowestVoltage());
                        globalTracer.traceInfo(funcName, "[%.3f] Total=%.2fA", currTime, pdp.getTotalCurrent());
                    }
                }
            }

            if (RobotParams.Preferences.debugDriveBase)
            {
                TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();

                dashboard.putNumber("DriveBase/xPos", robotPose.x);
                dashboard.putNumber("DriveBase/yPos", robotPose.y);
                dashboard.putData("DriveBase/heading", ((FrcAHRSGyro) robotDrive.gyro).getGyroSendable());
                dashboard.putNumber("DriveBase/Yaw", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getYaw());
                dashboard.putNumber("DriveBase/Pitch", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getPitch());
                dashboard.putNumber("DriveBase/Roll", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getRoll());
                dashboard.putNumber("DriveBase/AccelX", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelX());
                dashboard.putNumber("DriveBase/AccelY", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelY());
                dashboard.putNumber("DriveBase/AccelZ", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelZ());
                dashboard.putNumber("DriverBase/Compass", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getCompassHeading());

                //
                // DriveBase debug info.
                //
                double lfDriveEnc = robotDrive.lfDriveMotor.getPosition();
                double rfDriveEnc = robotDrive.rfDriveMotor.getPosition();
                double lbDriveEnc = robotDrive.lbDriveMotor.getPosition();
                double rbDriveEnc = robotDrive.rbDriveMotor.getPosition();

                dashboard.displayPrintf(
                    8, "DriveBase-Drive: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                    lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                    (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / 4.0);

                double lfSteerEnc = robotDrive.lfSteerMotor.getPosition();
                double rfSteerEnc = robotDrive.rfSteerMotor.getPosition();
                double lbSteerEnc = robotDrive.lbSteerMotor.getPosition();
                double rbSteerEnc = robotDrive.rbSteerMotor.getPosition();

                dashboard.displayPrintf(
                    9, "DriveBase-Steer: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f",
                    lfSteerEnc, rfSteerEnc, lbSteerEnc, rbSteerEnc);

                if (RobotParams.Preferences.debugPidDrive)
                {
                    int lineNum = 10;
                    if (robotDrive.encoderXPidCtrl != null)
                    {
                        robotDrive.encoderXPidCtrl.displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                    robotDrive.encoderYPidCtrl.displayPidInfo(lineNum);
                    lineNum += 2;
                    robotDrive.gyroTurnPidCtrl.displayPidInfo(lineNum);
                }
                else
                {
                    dashboard.displayPrintf(
                        10, "Gyro: Yaw=%.2f, Pitch=%.2f, Roll=%.2f",
                        ((FrcAHRSGyro) robotDrive.gyro).ahrs.getYaw(),
                        ((FrcAHRSGyro) robotDrive.gyro).ahrs.getPitch(),
                        ((FrcAHRSGyro)robotDrive.gyro).ahrs.getRoll());
                    dashboard.displayPrintf(
                        11, "Accel: X=%.2f, Y=%.2f, Z=%.2f",
                        ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelX(),
                        ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelY(),
                        ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelZ());
                    dashboard.displayPrintf(
                        12, "Compass: Heading=%.2f", ((FrcAHRSGyro)robotDrive.gyro).ahrs.getCompassHeading());
                }
            }

            if (RobotParams.Preferences.showVisionStatus)
            {
                if (vision != null)
                {
                    FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();

                    if (pose != null)
                    {
                        double horiAngle = vision.getTargetHorizontalAngle();
                        double vertAngle = vision.getTargetVerticalAngle();
                        double distanceToTarget = vision.getTargetDistance();
                        dashboard.putNumber("Camera/distance", distanceToTarget + RobotParams.VISION_TARGET_RADIUS);
                        dashboard.putNumber("Camera/horiAngle", horiAngle);
                        dashboard.putNumber("Camera/vertAngle", vertAngle);
                        if (RobotParams.Preferences.debugVision)
                        {
                            dashboard.displayPrintf(
                                15, "VisionTarget: x=%.1f,y=%.1f,depth=%.1f/%.1f,horiAngle=%.1f,vertAngle=%.1f",
                                pose.x, pose.y, pose.r, distanceToTarget + RobotParams.VISION_TARGET_RADIUS,
                                horiAngle, vertAngle);
                        }
                    }
                    else if (RobotParams.Preferences.debugVision)
                    {
                        dashboard.displayPrintf(15, "VisionTarget: No target found!");
                    }
                }
            }

            if (RobotParams.Preferences.showSubsystemStatus)
            {
                if (robotDrive != null && robotDrive.driveBase != null)
                {
                    if (wallAlignSensor != null)
                    {
                        dashboard.displayPrintf(
                            1, "RobotPose: %s, angleToWall=%.1f",
                            robotDrive.driveBase.getFieldPosition(), wallAlignSensor.getAngleToWall());
                    }
                    else
                    {
                        dashboard.displayPrintf(
                            1, "RobotPose: %s", robotDrive.driveBase.getFieldPosition());
                    }
                }

                if (shooter != null)
                {
                    dashboard.displayPrintf(
                        2, "Shooter.SetVel: Lower=%.0f, Upper=%.0f, VelMode=%s",
                        lowerFlywheelUserVel, upperFlywheelUserVel, shooter.isFlywheelInVelocityMode());
                    dashboard.displayPrintf(
                        3, "Shooter.Flywheel: Pwr=%.1f/%.1f, Vel=%.1f/%.1f, OnTarget=%s, AlignEnabled=%s",
                        shooter.getLowerFlywheelPower(), shooter.getUpperFlywheelPower(),
                        shooter.getLowerFlywheelVelocity(), shooter.getUpperFlywheelVelocity(),
                        shooter.isFlywheelVelOnTarget(), shooter.isVisionAlignEnabled());
                    dashboard.displayPrintf(
                        4, "Shooter.Tilter: Far=%s, Pos=%.1f, Pressure=%.1f",
                        shooter.isTilterAtFarPosition(), shooter.getTilterPosition(), getPressure());
                    // dashboard.displayPrintf(
                    //     4, "Shooter.Tilter: Pwr=%.1f, Pos=%.1f, limitSW=%s",
                    //     shooter.getTilterPower(), shooter.getTilterPosition(),
                    //     shooter.isTilterLowerLimitSwitchActive());
                }

                if (conveyor != null)
                {
                    boolean entranceHasBall = conveyor.isEntranceSensorActive();
                    boolean exitHasBall = conveyor.isExitSensorActive();

                    dashboard.displayPrintf(
                        5, "Conveyor: Power=%.1f, entrance=%s, exit=%s",
                        conveyor.getMotorPower(), entranceHasBall, exitHasBall);

                    ledIndicator.setConveyorFull(entranceHasBall && exitHasBall);
                }

                if (intake != null)
                {
                    dashboard.displayPrintf(
                        6, "Intake: Power=%.1f, extended=%s",
                        intake.getMotorPower(), intake.isExtended());
                }

                if (climber != null)
                {
                    dashboard.displayPrintf(
                        7, "Climber: Pwr=%.1f, Pos=%.1f, LimitSW=%s, PneumaticExtended=%s",
                        climber.climberMotor.getMotorPower(), climber.climber.getPosition(),
                        climber.isLowerLimitSwitchActive(), climber.climberPneumatic.isExtended());
                }
            }
        }
    }   //updateStatus

    /**
     * This method creates and opens the trace log with the file name derived from the given match info.
     * Note that the trace log is disabled after it is opened. The caller must explicitly call setTraceLogEnabled
     * to enable/disable it.
     *
     * @param matchInfo specifies the match info from which the trace log file name is derived.
     */
    public void openTraceLog(FrcMatchInfo matchInfo)
    {
        if (RobotParams.Preferences.useTraceLog && !traceLogOpened)
        {
            String fileName = matchInfo.eventName != null?
                String.format(Locale.US, "%s_%s%03d", matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber):
                getCurrentRunMode().name();

            traceLogOpened = globalTracer.openTraceLog(RobotParams.TEAM_FOLDER + "/tracelogs", fileName);
        }
    }   //openTraceLog

    /**
     * This method closes the trace log if it was opened.
     */
    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            globalTracer.closeTraceLog();
            traceLogOpened = false;
        }
    }   //closeTraceLog

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false to disable.
     */
    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            globalTracer.setTraceLogEnabled(enabled);
        }
    }   //setTraceLogEnabled

    /**
     * This method calculates the target alignment angle using robot's odometry location.
     *
     * @param robotX specifies the robot's absolute X location.
     * @param robotY specifies the robot's absolute Y location.
     * @return target absolute field angle from robot's location.
     */
    public double getAlignAngleFromOdometry(double robotX, double robotY)
    {
        final String funcName = "getAlignAngleFromOdometry";
        double angle;
        double theta = robotX != 0.0? 90.0 - Math.abs(Math.atan(robotY / robotX)): 0.0;

        if (robotX > 0.0 && robotY > 0.0)
        {
            // Quadrant 1
            angle = 180.0 + theta;
        }
        else if (robotX < 0.0 && robotY > 0.0)
        {
            // Quadrant 2
            angle = 180.0 - theta;
        }
        else if (robotX < 0.0 && robotY <= 0.0)
        {
            // Quadrant 3
            angle = theta;
        }
        else if (robotX > 0.0 && robotY <= 0.0)
        {
            // Quadrant 4
            angle = -theta;
        }
        else
        {
            // robotX is 0.0
            angle = robotY > 0.0? 180.0: 0.0;
        }

        globalTracer.traceInfo(funcName, "x=%.2f, y=%.2f, angle=%.2f", robotX, robotY, angle);
        return angle;
    }   //getAlignAngleFromOdometry

    /**
     * This method calculates the location of the robot for picking the ball given the ball's location and the
     * distance in front of the ball as well as the angle of approach.
     *
     * @param ballPoint specifies the ball location.
     * @param distance specifies the distance in front of the ball.
     * @param angle specifies the approach angle.
     * @param xAdj specifies adjustment in X.
     * @param yAdj specifies adjustment in Y.
     * @return robot's location to pickup the ball.
     */
    public TrcPose2D ballPickupPoint(TrcPose2D ballPoint, double distance, double angle, double xAdj, double yAdj)
    {
        TrcPose2D pickupPoint = ballPoint.clone();
        double deltaX, deltaY;

        deltaX = distance * Math.sin(Math.toRadians(angle));
        deltaY = distance * Math.cos(Math.toRadians(angle));
        pickupPoint.x += -deltaX + xAdj;
        pickupPoint.y += -deltaY + yAdj;
        pickupPoint.angle = angle;

        return pickupPoint;
    }   //ballPickupPoint

    /**
     * This method clones the given target point and adds adjustments to x, y and angle.
     *
     * @param targetPoint specifies the original target point.
     * @param xAdj specifies adjustment added to X.
     * @param yAdj specifies adjustment added to Y.
     * @param angleAdj specifies adjustment added to angle.
     * @return adjusted target point.
     */
    public TrcPose2D pathPoint(TrcPose2D targetPoint, double xAdj, double yAdj, double angleAdj)
    {
        TrcPose2D point = targetPoint.clone();

        point.x += xAdj;
        point.y += yAdj;
        point.angle += angleAdj;

        return point;
    }   //pathPoint

    /**
     * This method creates the path point for shooting. It only requires x and y. This method will calculate the
     * heading from x and y. Optionally, one can specify an angle adjustment to the calulated heading.
     *
     * @param x specifies the absolute X field coordinate of the shooting point.
     * @param y specifies the absolute Y field coordinate of the shooting point.
     * @param angleAdj specifies the angle adjustment to be added to the calculated heading, can be zero if no
     *                 adjustment is necessary..
     * @return shooting point.
     */
    public TrcPose2D shootingPoint(double x, double y, double angleAdj)
    {
        return new TrcPose2D(x, y, getAlignAngleFromOdometry(x, y) + angleAdj);
    }   //shootingPoint

    /**
     * This method builds a path with the given list of poses. It will also adjust each waypoint in the path to
     * compensate for the robot length and intake offset so that the intake will be reaching the target point
     * instead of the centroid of the robot.
     *
     * @param incrementalPath specifies true if the poses are incremental from the their poses.
     * @param distanceAdj specifies the distance adjustment towards the ball (positive is closer to the ball).
     * @param poses specifies an array of poses used to build the path.
     * @return path built for PurePursuit.
     */
    public TrcPath buildPath(boolean  incrementalPath, double distanceAdj, TrcPose2D... poses)
    {
        final String funcName = "buildPath";
        TrcPath path;
        TrcPathBuilder pathBuilder = new TrcPathBuilder(robotDrive.driveBase.getFieldPosition(), incrementalPath);

        for (TrcPose2D pose: poses)
        {
            pathBuilder.append(pose);
        }
        path = pathBuilder.toRelativeStartPath();

        globalTracer.traceInfo(funcName, "OriginalPath: %s", path.toAbsolute(robotDrive.driveBase.getFieldPosition()));

        TrcWaypoint secondLastWaypoint = path.getWaypoint(path.getSize() - 2);
        TrcWaypoint lastWaypoint = path.getLastWaypoint();
        TrcPose2D relativePose = lastWaypoint.pose.relativeTo(secondLastWaypoint.pose);
        double distance = TrcUtil.magnitude(relativePose.x, relativePose.y) + distanceAdj - RobotParams.INTAKE_OFFSET;

        relativePose.x = distance*Math.cos(Math.toRadians(relativePose.angle));
        relativePose.y = distance*Math.sin(Math.toRadians(relativePose.angle));
        relativePose.angle = 0.0;
        lastWaypoint.pose.setAs(secondLastWaypoint.pose.addRelativePose(relativePose));

        globalTracer.traceInfo(funcName, "AdjustedPath: %s", path.toAbsolute(robotDrive.driveBase.getFieldPosition()));

        return path;
    }   //buildPath

    //
    // Getters for sensor data.
    //

    /**
     * This method returns the pressure value from the pressure sensor.
     *
     * @return pressure value.
     */
    public double getPressure()
    {
        return (pressureSensor.getVoltage() - 0.5) * 50.0;
    }   //getPressure

}   //class Robot
