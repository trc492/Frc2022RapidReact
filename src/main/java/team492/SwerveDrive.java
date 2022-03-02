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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import TrcCommonLib.trclib.TrcEnhancedServo;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcFalconServo;
import TrcFrcLib.frclib.FrcPdp;

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
    public final FrcCANFalcon lfSteerMotor, rfSteerMotor, lbSteerMotor, rbSteerMotor;
    public final TrcSwerveModule lfWheel, lbWheel, rfWheel, rbWheel;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public SwerveDrive(Robot robot)
    {
        super(robot);

        lfDriveMotor = createDriveMotor("lfDrive", RobotParams.CANID_LEFTFRONT_DRIVE, true);
        rfDriveMotor = createDriveMotor("rfDrive", RobotParams.CANID_RIGHTFRONT_DRIVE, true);
        lbDriveMotor = createDriveMotor("lbDrive", RobotParams.CANID_LEFTBACK_DRIVE, true);
        rbDriveMotor = createDriveMotor("rbDrive", RobotParams.CANID_RIGHTBACK_DRIVE, true);

        lfSteerMotor = createSteerMotor(
            "lfSteer", RobotParams.CANID_LEFTFRONT_STEER, RobotParams.CANID_LEFTFRONT_STEER_ENCODER, false);
        rfSteerMotor = createSteerMotor(
            "rfSteer", RobotParams.CANID_RIGHTFRONT_STEER, RobotParams.CANID_RIGHTFRONT_STEER_ENCODER, false);
        lbSteerMotor = createSteerMotor(
            "lbSteer", RobotParams.CANID_LEFTBACK_STEER, RobotParams.CANID_LEFTBACK_STEER_ENCODER, false);
        rbSteerMotor = createSteerMotor(
            "rbSteer", RobotParams.CANID_RIGHTBACK_STEER, RobotParams.CANID_RIGHTBACK_STEER_ENCODER, false);

        int[] zeros = getSteerZeroPositions();
        lfWheel = createSwerveModule("lfWheel", lfDriveMotor, lfSteerMotor, zeros[0]);
        rfWheel = createSwerveModule("rfWheel", rfDriveMotor, rfSteerMotor, zeros[1]);
        lbWheel = createSwerveModule("lbWheel", lbDriveMotor, lbSteerMotor, zeros[2]);
        rbWheel = createSwerveModule("rbWheel", rbDriveMotor, rbSteerMotor, zeros[3]);

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

    private FrcCANFalcon createSteerMotor(String name, int motorCanID, int encoderCanID, boolean inverted)
    {
        CANCoder encoder = new CANCoder(encoderCanID);
        FrcCANFalcon steerMotor = new FrcCANFalcon(name, motorCanID);

        encoder.configFactoryDefault();
        encoder.configFeedbackCoefficient(1.0, "pulse", SensorTimeBase.PerSecond);
        encoder.configSensorDirection(true);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        steerMotor.motor.configFactoryDefault();
        steerMotor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        steerMotor.motor.enableVoltageCompensation(true);
        steerMotor.motor.configRemoteFeedbackFilter(encoder, 0);
        steerMotor.motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        steerMotor.setInverted(inverted);
        steerMotor.setBrakeModeEnabled(true);

        return steerMotor;
    }   //createSteerMotor

    private TrcSwerveModule createSwerveModule(
        String name, FrcCANFalcon driveMotor, FrcCANFalcon steerMotor, int steerZero)
    {
        FrcFalconServo servo = new FrcFalconServo(
            name + ".servo", steerMotor, RobotParams.steerCoeffs, RobotParams.STEER_DEGREES_PER_TICK, steerZero,
            RobotParams.STEER_MAX_REQ_VEL, RobotParams.STEER_MAX_ACCEL);
        TrcSwerveModule module = new TrcSwerveModule(
            name, driveMotor, new TrcEnhancedServo(name + ".enhancedServo", servo));
        module.disableSteeringLimits();
        module.setSteerAngle(0.0, false);

        return module;
    }   //createSwerveModule

    @Override
    public void startSteerCalibrate()
    {
        lfSteerMotor.set(0.0);
        rfSteerMotor.set(0.0);
        lbSteerMotor.set(0.0);
        rbSteerMotor.set(0.0);
        robot.dashboard.putBoolean(DBKEY_TEST_RUN_MOTORS, false);
        robot.dashboard.putBoolean(DBKEY_TEST_SET_ANGLE, false);
        robot.dashboard.putBoolean(DBKEY_TEST_SAVE_ANGLES, false);
    }   //startCalibrate

    @Override
    public void steerCalibratePeriodic()
    {
        if (robot.dashboard.getBoolean(DBKEY_TEST_SET_ANGLE, false))
        {
            ((TrcSwerveDriveBase) driveBase).setSteerAngle(
                robot.dashboard.getNumber(DBKEY_TEST_ANGLE_TARGET, 0), false);
            robot.dashboard.putBoolean(DBKEY_TEST_SET_ANGLE, false);
        }

        if (robot.dashboard.getBoolean(DBKEY_TEST_SAVE_ANGLES, false))
        {
            robot.dashboard.putBoolean(DBKEY_TEST_SAVE_ANGLES, false);
            saveSteerZeroPositions();
        }

        double power = robot.dashboard.getBoolean(DBKEY_TEST_RUN_MOTORS, false) ? RobotParams.STEER_CAL_POWER : 0.0;
        lfDriveMotor.set(power);
        rfDriveMotor.set(power);
        lbDriveMotor.set(power);
        rbDriveMotor.set(power);
        robot.dashboard.putString(
            DBKEY_TEST_SWERVE_ANGLES,
            String.format(
                "lf=%.2f/%.0f, rf=%.2f/%.0f, lr=%.2f/%.0f, rr=%.2f/%.0f",
                lfWheel.getSteerAngle(), lfSteerMotor.getMotorPosition(),
                rfWheel.getSteerAngle(), rfSteerMotor.getMotorPosition(),
                lbWheel.getSteerAngle(), lbSteerMotor.getMotorPosition(),
                rbWheel.getSteerAngle(), rbSteerMotor.getMotorPosition()));
    }   //calibratePeriodic

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
            robot.globalTracer.traceWarn(funcName, "Steer zero position file not found, using built-in defaults.");
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
            out.printf("%.0f\n", TrcUtil.modulo(lfSteerMotor.getMotorPosition(), RobotParams.STEER_ENCODER_PPR));
            out.printf("%.0f\n", TrcUtil.modulo(rfSteerMotor.getMotorPosition(), RobotParams.STEER_ENCODER_PPR));
            out.printf("%.0f\n", TrcUtil.modulo(lbSteerMotor.getMotorPosition(), RobotParams.STEER_ENCODER_PPR));
            out.printf("%.0f\n", TrcUtil.modulo(rbSteerMotor.getMotorPosition(), RobotParams.STEER_ENCODER_PPR));
            robot.globalTracer.traceInfo(funcName, "Saved steer zeros!");
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveSteerZeroPositions

}   //class SwerveDrive
