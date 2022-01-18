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

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Scanner;
import java.util.stream.IntStream;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import TrcCommonLib.trclib.TrcEnhancedServo;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcPdp;
import TrcFrcLib.frclib.FrcTalonServo;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class RobotDriveSwerve
{
    //
    // Global objects.
    //
    private final Robot robot;

    //
    // Sensors.
    //
    public final FrcAHRSGyro gyro;

    //
    // Drive motors.
    //

    //
    // Drive Base.
    //
    public final FrcCANSparkMax lfDriveMotor, rfDriveMotor, lbDriveMotor, rbDriveMotor;
    public final FrcCANTalon lfSteerMotor, rfSteerMotor, lbSteerMotor, rbSteerMotor;
    public final TrcSwerveModule lfWheel, lbWheel, rfWheel, rbWheel;
    public final TrcSwerveDriveBase driveBase;

    public final TrcPidController encoderXPidCtrl;
    public final TrcPidController encoderYPidCtrl;
    public final TrcPidController gyroTurnPidCtrl;
    public final TrcPidDrive pidDrive;
    public final TrcPurePursuitDrive purePursuitDrive;
    //
    // Coefficients for PID controllers.
    //
    public final TrcPidController.PidCoefficients xPosPidCoeff;
    public final TrcPidController.PidCoefficients yPosPidCoeff;
    public final TrcPidController.PidCoefficients turnPidCoeff;
    public final TrcPidController.PidCoefficients velPidCoeff;

    /**
     * Constructor: Create an instance of the object.
     */
    public RobotDriveSwerve(Robot robot)
    {
        this.robot = robot;
        gyro = RobotParams.Preferences.useNavX ? new FrcAHRSGyro("NavX", SPI.Port.kMXP) : null;

        lfDriveMotor = createSparkMax("lfDrive", RobotParams.CANID_LEFTFRONT_DRIVE);
        rfDriveMotor = createSparkMax("rfDrive", RobotParams.CANID_RIGHTFRONT_DRIVE);
        lbDriveMotor = createSparkMax("lbDrive", RobotParams.CANID_LEFTBACK_DRIVE);
        rbDriveMotor = createSparkMax("rbDrive", RobotParams.CANID_RIGHTBACK_DRIVE);

        // rf lb are inverted always, lf and rb are inverted on comp, not inverted on practice
        lfSteerMotor = createSteerTalon("lfSteer", RobotParams.CANID_LEFTFRONT_STEER, true);
        rfSteerMotor = createSteerTalon("rfSteer", RobotParams.CANID_RIGHTFRONT_STEER, true);
        lbSteerMotor = createSteerTalon("lbSteer", RobotParams.CANID_LEFTBACK_STEER, true);
        rbSteerMotor = createSteerTalon("rbSteer", RobotParams.CANID_RIGHTBACK_STEER, true);

        int[] zeros = getSteerZeroPositions();
        lfWheel = createModule("lfWheel", lfDriveMotor, lfSteerMotor, zeros[0]);
        rfWheel = createModule("rfWheel", rfDriveMotor, rfSteerMotor, zeros[1]);
        lbWheel = createModule("lbWheel", lbDriveMotor, lbSteerMotor, zeros[2]);
        rbWheel = createModule("rbWheel", rbDriveMotor, rbSteerMotor, zeros[3]);

        robot.pdp.registerEnergyUsed(
            new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_FRONT_WHEEL, "lfWheel"),
            new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_BACK_WHEEL, "lbWheel"),
            new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_FRONT_WHEEL, "rfWheel"),
            new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_BACK_WHEEL, "rbWheel"));

        driveBase = new TrcSwerveDriveBase(
            lfWheel, lbWheel, rfWheel, rbWheel, gyro, RobotParams.ROBOT_DRIVE_WIDTH, RobotParams.ROBOT_DRIVE_LENGTH);
        driveBase.setSynchronizeOdometriesEnabled(false);
        driveBase.setOdometryScales(RobotParams.ENCODER_INCHES_PER_COUNT);

        // if (RobotParams.Preferences.useExternalOdometry)
        // {
        //     //
        //     // Create the external odometry device that uses the left front encoder port as the X odometry and
        //     // the left and right back encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
        //     // odometry.
        //     //
        //     TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
        //         new TrcDriveBaseOdometry.AxisSensor(rightBackWheel, RobotParams.X_ODOMETRY_WHEEL_OFFSET),
        //         new TrcDriveBaseOdometry.AxisSensor[] {
        //             new TrcDriveBaseOdometry.AxisSensor(leftFrontWheel, RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
        //             new TrcDriveBaseOdometry.AxisSensor(rightFrontWheel, RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
        //         gyro);
        //     //
        //     // Set the drive base to use the external odometry device overriding the built-in one.
        //     //
        //     driveBase.setDriveBaseOdometry(driveBaseOdometry);
        //     driveBase.setOdometryScales(RobotParams.ODWHEEL_X_INCHES_PER_COUNT, RobotParams.ODWHEEL_Y_INCHES_PER_COUNT);
        // }
        // else
        // {
        //     driveBase.setOdometryScales(RobotParams.ENCODER_X_INCHES_PER_COUNT, RobotParams.ENCODER_Y_INCHES_PER_COUNT);
        // }

        //
        // Create and initialize PID controllers.
        //
        xPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ENCODER_KP, RobotParams.ENCODER_KI, RobotParams.ENCODER_KD, RobotParams.ENCODER_KF);
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ENCODER_KP, RobotParams.ENCODER_KI, RobotParams.ENCODER_KD, RobotParams.ENCODER_KF);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_TURN_KP, RobotParams.GYRO_TURN_KI, RobotParams.GYRO_TURN_KD, RobotParams.GYRO_TURN_KF);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl", xPosPidCoeff, RobotParams.ENCODER_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl", yPosPidCoeff, RobotParams.ENCODER_TOLERANCE, driveBase::getYPosition);
        gyroTurnPidCtrl = new TrcPidController(
            "gyroPidCtrl", turnPidCoeff, RobotParams.GYRO_TURN_TOLERANCE, driveBase::getHeading);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);

        encoderXPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_XPID_POWER);
        encoderYPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_YPID_POWER);
        gyroTurnPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_TURNPID_POWER);
        encoderXPidCtrl.setRampRate(RobotParams.DRIVE_MAX_XPID_RAMP_RATE);
        encoderYPidCtrl.setRampRate(RobotParams.DRIVE_MAX_YPID_RAMP_RATE);
        gyroTurnPidCtrl.setRampRate(RobotParams.DRIVE_MAX_TURNPID_RAMP_RATE);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setMsgTracer(robot.globalTracer, true, true);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase, RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE,
            RobotParams.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_OUTPUT_LIMIT);
        purePursuitDrive.setStallDetectionEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, true, true);
    }   //RobotDriveSwerve

    /**
     * This method is called to prepare the robot base before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    public void startMode(RunMode runMode, RunMode prevMode)
    {
        if (runMode != RunMode.DISABLED_MODE)
        {
            setOdometryEnabled(true);
        }

        if (runMode == RunMode.AUTO_MODE)
        {
            lfDriveMotor.motor.setOpenLoopRampRate(0);
            rfDriveMotor.motor.setOpenLoopRampRate(0);
            lbDriveMotor.motor.setOpenLoopRampRate(0);
            rbDriveMotor.motor.setOpenLoopRampRate(0);
        }
        else
        {
            lfDriveMotor.motor.setOpenLoopRampRate(RobotParams.DRIVE_RAMP_RATE);
            rfDriveMotor.motor.setOpenLoopRampRate(RobotParams.DRIVE_RAMP_RATE);
            lbDriveMotor.motor.setOpenLoopRampRate(RobotParams.DRIVE_RAMP_RATE);
            rbDriveMotor.motor.setOpenLoopRampRate(RobotParams.DRIVE_RAMP_RATE);
        }
    }   //startMode

    /**
     * This method is called to prepare the robot base right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    public void stopMode(RunMode runMode, RunMode nextMode)
    {
        if (runMode != RunMode.DISABLED_MODE)
        {
            setOdometryEnabled(false);
        }
    }   //stopMode

    /**
     * This method cancels any PIDDrive operation still in progress.
     */
    public void cancel()
    {
        if (pidDrive.isActive())
        {
            pidDrive.cancel();
        }

        if (purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel();
        }

        driveBase.stop();
    }   //cancel

    /**
     * This method enables/disables robot base odometry.
     *
     * @param enabled specifies true to enable odometry, false to disable.
     */
    public void setOdometryEnabled(boolean enabled)
    {
        if (enabled) driveBase.resetOdometry(true, false);
        lfDriveMotor.setOdometryEnabled(enabled);
        rfDriveMotor.setOdometryEnabled(enabled);
        lbDriveMotor.setOdometryEnabled(enabled);
        rbDriveMotor.setOdometryEnabled(enabled);
        driveBase.setOdometryEnabled(enabled);
    }   //setOdometryEnabled

    /**
     * This method creates and initializes a SparkMax motor controller as one of the drive wheels motors.
     *
     * @param name specifies the name of the drive wheel motor.
     * @param id specifies the CAN ID of the motor controller.
     * @return the created SparkMax controller.
     */
    private FrcCANSparkMax createSparkMax(String name, int id)
    {
        FrcCANSparkMax spark = new FrcCANSparkMax(name, id, true);
        spark.motor.restoreFactoryDefaults();
        spark.setInverted(false);
        spark.setPositionSensorInverted(false);
        spark.setBrakeModeEnabled(true);
        spark.motor.enableVoltageCompensation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        spark.motor.burnFlash();
        return spark;
    }   //createSparkMax

    /**
     * This method creates and initializes a Talon motor controller as one of the steering motors.
     *
     * @param name specifies the name of the steering motor.
     * @param id specifies the CAN ID of the motor controller.
     * @return the created Talon controller.
     */
    private FrcCANTalon createSteerTalon(String name, int id, boolean inverted)
    {
        FrcCANTalon talon = new FrcCANTalon(name, id);
        talon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talon.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        talon.motor.enableVoltageCompensation(true);
        talon.motor.overrideLimitSwitchesEnable(false);
        talon.configFwdLimitSwitchNormallyOpen(true);
        talon.configRevLimitSwitchNormallyOpen(true);
        talon.setBrakeModeEnabled(true);
        talon.setPositionSensorInverted(inverted);
        talon.setInverted(!inverted);
        return talon;
    }   //createSteerTalon

    /**
     * This method creates and initializes a swerve module that consists of a drive motor and a steering motor.
     *
     * @param name specifies the name of the swerve module.
     * @param drive specifies the drive motor object.
     * @param steer specifies the steering motor object.
     * @param steerZero specifies the zero offset of the steering encoder.
     * @return the created swerve module.
     */
    private TrcSwerveModule createModule(String name, FrcCANSparkMax drive, FrcCANTalon steer, int steerZero)
    {
        final String funcName = "createModule";
        steer.motor.getSensorCollection().setPulseWidthPosition(0, 10); // reset index
        TrcUtil.sleep(50); // guarantee reset
        ErrorCode error = steer.motor.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true, -steerZero, 10);
        if (error != ErrorCode.OK)
        {
            robot.globalTracer.traceErr(funcName, "Encoder error: module=%s, error=%s", name, error.name());
        }
        TrcUtil.sleep(50); // guarantee reset
        int modPos = (int) TrcUtil.modulo(steer.motor.getSelectedSensorPosition(), 4096);
        int pos = modPos > 2048 ? modPos - 4096 : modPos;
        steer.motor.setSelectedSensorPosition(pos, 0, 10);
        TrcUtil.sleep(50);

        robot.globalTracer.traceInfo(
            funcName, "Module=%s, Zero=%d, PwmPos=%d, quadPos=%d, selectedPos=%d",
            name, steerZero, steer.motor.getSensorCollection().getPulseWidthPosition(),
            steer.motor.getSensorCollection().getQuadraturePosition(), steer.motor.getSelectedSensorPosition());

        FrcTalonServo servo = new FrcTalonServo(name + ".servo", steer, RobotParams.magicSteerCoeff,
            RobotParams.STEER_DEGREES_PER_TICK, RobotParams.STEER_MAX_REQ_VEL, RobotParams.STEER_MAX_ACCEL);
        TrcSwerveModule module = new TrcSwerveModule(name, drive, new TrcEnhancedServo(name + ".enhancedServo", servo));
        module.disableSteeringLimits();
        return module;
    }   //createModule

    /**
     * This method retrieves the steering zero calibration data from the calibration data file.
     *
     * @return calibration data of all four swerve modules.
     */
    private int[] getSteerZeroPositions()
    {
        final String funcName = "getSteerZeroPositions";

        try (Scanner in = new Scanner(new FileReader("/home/lvuser/steerzeros.txt")))
        {
            return IntStream.range(0, 4).map(e -> in.nextInt()).toArray();
        }
        catch (Exception e)
        {
            robot.globalTracer.traceErr(funcName, "ERROR! Steer zero position file not found!");
            return RobotParams.STEER_ZEROS;
        }
    }   //getSteerZeroPositions

    /**
     * This method saves the steering zero calibration data to the calibration data file.
     */
    public void saveSteerZeroPositions()
    {
        final String funcName = "saveSteerZeroPositions";

        robot.globalTracer.traceInfo(funcName, "Saved steer zeros!");
        try (PrintStream out = new PrintStream(new FileOutputStream("/home/lvuser/steerzeros.txt")))
        {
            out.printf("%.0f\n",
                TrcUtil.modulo(lfSteerMotor.motor.getSensorCollection().getPulseWidthPosition(), 4096));
            out.printf("%.0f\n",
                TrcUtil.modulo(rfSteerMotor.motor.getSensorCollection().getPulseWidthPosition(), 4096));
            out.printf("%.0f\n",
                TrcUtil.modulo(lbSteerMotor.motor.getSensorCollection().getPulseWidthPosition(), 4096));
            out.printf("%.0f\n",
                TrcUtil.modulo(rbSteerMotor.motor.getSensorCollection().getPulseWidthPosition(), 4096));
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveSteerZeroPositions

}   //class RobotDriveSwerve
