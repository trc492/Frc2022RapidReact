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
import com.ctre.phoenix.sensors.CANCoder;

import TrcCommonLib.trclib.TrcEnhancedServo;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPidMotor;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcPidController.PidParameters;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcFalconServo;
import TrcFrcLib.frclib.FrcPdp;
import TrcFrcLib.frclib.FrcTalonServo;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class SwerveDrive extends RobotDrive
{
    private static final String DBKEY_TEST_RUN_MOTORS = "Test/RunMotors";
    private static final String DBKEY_TEST_SET_ANGLE = "Test/SetAngle";
    private static final String DBKEY_TEST_SAVE_ANGLES = "Test/SaveAngles";
    private static final String DBKEY_TEST_ANGLE_TARGET = "Test/AngleTarget";
    private static final String DBKEY_TEST_SWERVE_ANGLES = "Test/SwerveAngles";
    //
    // Swerve steering motors and modules.
    //
    public final FrcCANFalcon lfDriveMotor, rfDriveMotor, lbDriveMotor, rbDriveMotor;
    public final FrcCANFalcon lfSteerMotor, rfSteerMotor, lbSteerMotor, rbSteerMotor;
    public final CANCoder lfEncoder, rfEncoder, lbEncoder, rbEncoder;
    public final TrcSwerveModule lfWheel, lbWheel, rfWheel, rbWheel;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public SwerveDrive(Robot robot)
    {
        this.robot = robot;
        gyro = RobotParams.Preferences.useNavX ? new FrcAHRSGyro("NavX", SPI.Port.kMXP) : null;

        lfDriveMotor = new FrcCANFalcon("lfDrive", RobotParams.CANID_LEFTFRONT_DRIVE);
        rfDriveMotor = new FrcCANFalcon("rfDrive", RobotParams.CANID_RIGHTFRONT_DRIVE);
        lbDriveMotor = new FrcCANFalcon("lbDrive", RobotParams.CANID_LEFTBACK_DRIVE);
        rbDriveMotor = new FrcCANFalcon("rbDrive", RobotParams.CANID_RIGHTBACK_DRIVE);

        // rf lb are inverted always, lf and rb are inverted on comp, not inverted on practice
        lfSteerMotor = new FrcCANFalcon("lfSteer", RobotParams.CANID_LEFTFRONT_STEER);
        rfSteerMotor = new FrcCANFalcon("rfSteer", RobotParams.CANID_RIGHTFRONT_STEER);
        lbSteerMotor = new FrcCANFalcon("lbSteer", RobotParams.CANID_LEFTBACK_STEER);
        rbSteerMotor = new FrcCANFalcon("rbSteer", RobotParams.CANID_RIGHTBACK_STEER);
        lfSteerMotor.setInverted(true);
        rfSteerMotor.setInverted(true);
        lbSteerMotor.setInverted(true);
        rbSteerMotor.setInverted(true);

        lfEncoder = new CANCoder(RobotParams.CANID_LEFTFRONT_STEER_ENCODER);
        rfEncoder = new CANCoder(RobotParams.CANID_RIGHTFRONT_STEER_ENCODER);
        lbEncoder = new CANCoder(RobotParams.CANID_LEFTBACK_STEER_ENCODER);
        rbEncoder = new CANCoder(RobotParams.CANID_RIGHTBACK_STEER_ENCODER);

        // int[] zeros = getSteerZeroPositions();
        lfWheel = createModule("lfWheel", lfDriveMotor, lfSteerMotor, lfEncoder);
        rfWheel = createModule("rfWheel", rfDriveMotor, rfSteerMotor, rfEncoder);
        lbWheel = createModule("lbWheel", lbDriveMotor, lbSteerMotor, lbEncoder);
        rbWheel = createModule("rbWheel", rbDriveMotor, rbSteerMotor, rbEncoder);

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_FRONT_DRIVE, "lfDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_BACK_DRIVE, "lbDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_FRONT_DRIVE, "rfDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_BACK_DRIVE, "rbDriveMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_FRONT_STEER, "lfSteerMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LEFT_BACK_STEER, "lbSteerMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_FRONT_STEER, "rfSteerMotor"),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RIGHT_BACK_STEER, "rbSteerMotor"));
        }

        driveBase = new TrcSwerveDriveBase(
            lfWheel, lbWheel, rfWheel, rbWheel, gyro, RobotParams.ROBOT_DRIVE_WIDTH, RobotParams.ROBOT_DRIVE_LENGTH);
        driveBase.setSynchronizeOdometriesEnabled(false);
        driveBase.setOdometryScales(RobotParams.SWERVE_INCHES_PER_COUNT);

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
        // PID Coefficients for X and Y are the same for Swerve Drive.
        xPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.SWERVE_KP, RobotParams.SWERVE_KI, RobotParams.SWERVE_KD, RobotParams.SWERVE_KF);
        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl", xPosPidCoeff, RobotParams.SWERVE_TOLERANCE, driveBase::getXPosition);
        encoderXPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_XPID_POWER);
        encoderXPidCtrl.setRampRate(RobotParams.DRIVE_MAX_XPID_RAMP_RATE);
    
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.SWERVE_KP, RobotParams.SWERVE_KI, RobotParams.SWERVE_KD, RobotParams.SWERVE_KF);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl", yPosPidCoeff, RobotParams.SWERVE_TOLERANCE, driveBase::getYPosition);
        encoderYPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_YPID_POWER);
        encoderYPidCtrl.setRampRate(RobotParams.DRIVE_MAX_YPID_RAMP_RATE);
    
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_TURN_KP, RobotParams.GYRO_TURN_KI, RobotParams.GYRO_TURN_KD, RobotParams.GYRO_TURN_KF);
        gyroTurnPidCtrl = new TrcPidController(
            "gyroPidCtrl", turnPidCoeff, RobotParams.GYRO_TURN_TOLERANCE, driveBase::getHeading);
        gyroTurnPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_TURNPID_POWER);
        gyroTurnPidCtrl.setRampRate(RobotParams.DRIVE_MAX_TURNPID_RAMP_RATE);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
    
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

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
    }   //SwerveDrive

    /**
     * This method is called to prepare the robot base before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    public void startMode(RunMode runMode, RunMode prevMode)
    {
        super.startMode(runMode, prevMode);

        // if (runMode == RunMode.AUTO_MODE)
        // {
        //     ((FrcCANSparkMax)lfDriveMotor).motor.setOpenLoopRampRate(0);
        //     ((FrcCANSparkMax)rfDriveMotor).motor.setOpenLoopRampRate(0);
        //     ((FrcCANSparkMax)lbDriveMotor).motor.setOpenLoopRampRate(0);
        //     ((FrcCANSparkMax)rbDriveMotor).motor.setOpenLoopRampRate(0);
        // }
        // else
        // {
        //     ((FrcCANSparkMax)lfDriveMotor).motor.setOpenLoopRampRate(RobotParams.DRIVE_RAMP_RATE);
        //     ((FrcCANSparkMax)rfDriveMotor).motor.setOpenLoopRampRate(RobotParams.DRIVE_RAMP_RATE);
        //     ((FrcCANSparkMax)lbDriveMotor).motor.setOpenLoopRampRate(RobotParams.DRIVE_RAMP_RATE);
        //     ((FrcCANSparkMax)rbDriveMotor).motor.setOpenLoopRampRate(RobotParams.DRIVE_RAMP_RATE);
        // }
    }   //startMode

    @Override
    public void startSteerCalibrate()
    {
        lfSteerMotor.set(0);
        rfSteerMotor.set(0);
        lbSteerMotor.set(0);
        rbSteerMotor.set(0);
        robot.dashboard.putBoolean(DBKEY_TEST_RUN_MOTORS, false);
        robot.dashboard.putBoolean(DBKEY_TEST_SET_ANGLE, false);
        robot.dashboard.putBoolean(DBKEY_TEST_SAVE_ANGLES, false);
    }   //startCalibrate

    @Override
    public void steerCalibratePeriodic()
    {
        if (robot.dashboard.getBoolean(DBKEY_TEST_SET_ANGLE, false))
        {
            ((TrcSwerveDriveBase)driveBase).setSteerAngle(
                robot.dashboard.getNumber(DBKEY_TEST_ANGLE_TARGET, 0), false);
            robot.dashboard.putBoolean(DBKEY_TEST_SET_ANGLE, false);
        }
        if (robot.dashboard.getBoolean(DBKEY_TEST_SAVE_ANGLES, false))
        {
            robot.dashboard.putBoolean(DBKEY_TEST_SAVE_ANGLES, false);
            saveSteerZeroPositions();
        }
        double power = robot.dashboard.getBoolean(DBKEY_TEST_RUN_MOTORS, false) ? 0.1 : 0.0;
        lfDriveMotor.set(power);
        rfDriveMotor.set(power);
        lbDriveMotor.set(power);
        rbDriveMotor.set(power);
        robot.dashboard.putString(
            DBKEY_TEST_SWERVE_ANGLES,
            String.format(
                "lf=%.2f,rf=%.2f,lr=%.2f,rr=%.2f",
                lfWheel.getSteerAngle(), rfWheel.getSteerAngle(),
                lbWheel.getSteerAngle(), rbWheel.getSteerAngle()));
    }   //calibratePeriodic

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
            funcName, "Module=%s, Zero=%d, PwmPos=%d, quadPos=%d, selectedPos=%f",
            name, steerZero, steer.motor.getSensorCollection().getPulseWidthPosition(),
            steer.motor.getSensorCollection().getQuadraturePosition(), steer.motor.getSelectedSensorPosition());

        FrcTalonServo servo = new FrcTalonServo(name + ".servo", steer, RobotParams.magicSteerCoeff,
            RobotParams.STEER_DEGREES_PER_TICK, RobotParams.STEER_MAX_REQ_VEL, RobotParams.STEER_MAX_ACCEL);
        TrcSwerveModule module = new TrcSwerveModule(name, drive, new TrcEnhancedServo(name + ".enhancedServo", servo));
        module.disableSteeringLimits();
        return module;
    }   //createModule

    private TrcSwerveModule createModule(String name, FrcCANFalcon drive, FrcCANFalcon steer, CANCoder encoder)
    {
        // + encoder later
        FrcFalconServo servo = new FrcFalconServo(name + "Servo", steer, RobotParams.magicSteerCoeff, RobotParams.STEER_DEGREES_PER_TICK, RobotParams.STEER_MAX_REQ_VEL, RobotParams.STEER_MAX_ACCEL);
        TrcSwerveModule module = new TrcSwerveModule(name, drive, new TrcEnhancedServo(name + "EnhancedServo", servo));
        return module;
    }

    /**
     * This method retrieves the steering zero calibration data from the calibration data file.
     *
     * @return calibration data of all four swerve modules.
     */
    private int[] getSteerZeroPositions()
    {
        final String funcName = "getSteerZeroPositions";

        try (Scanner in = new Scanner(new FileReader(RobotParams.TEAM_FOLDER + "/steerzeros.txt")))
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

        try (PrintStream out = new PrintStream(new FileOutputStream(RobotParams.TEAM_FOLDER + "/steerzeros.txt")))
        {
            out.printf("%.0f\n", TrcUtil.modulo(lfSteerMotor.getMotorPosition(), 4096));
            out.printf("%.0f\n", TrcUtil.modulo(rfSteerMotor.getMotorPosition(), 4096));
            out.printf("%.0f\n", TrcUtil.modulo(lbSteerMotor.getMotorPosition(), 4096));
            out.printf("%.0f\n", TrcUtil.modulo(rbSteerMotor.getMotorPosition(), 4096));
            robot.globalTracer.traceInfo(funcName, "Saved steer zeros!");
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveSteerZeroPositions

}   //class SwerveDrive
