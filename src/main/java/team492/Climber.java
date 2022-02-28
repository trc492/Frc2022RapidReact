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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcCANTalonLimitSwitch;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber
{
    private static final String moduleName = "Climber";
    private final Robot robot;
    private final FrcCANTalon climberMotor;
    private final FrcPneumatic climberPneumatic;
    private final FrcCANTalonLimitSwitch climberLowerLimitSwitch;
    private final FrcCANTalonLimitSwitch climberUpperLimitSwitch;
    private final TrcPidActuator climber;

    /**
     * Constructor: Create an instance of the object.
     */
    public Climber(Robot robot)
    {
        this.robot = robot;
        climberMotor = createClimberMotor(moduleName + ".motor", RobotParams.CANID_CLIMBER);
        climberPneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_CLIMBER_RETRACT, RobotParams.PNEUMATIC_CLIMBER_EXTEND);
        // Limit Switches may vary, not on robot yet
        climberLowerLimitSwitch = new FrcCANTalonLimitSwitch("climberLowerLimitSwitch", climberMotor, false);
        climberUpperLimitSwitch = new FrcCANTalonLimitSwitch("climberUpperLimitSwitch", climberMotor, true);
        Parameters climberParamaters = new Parameters()
            .setMotorParams(
                RobotParams.CLIMBER_MOTOR_INVERTED,
                RobotParams.CLIMBER_HAS_LOWERLIMIT_SWITCH, false,
                RobotParams.CLIMBER_HAS_UPPERLIMIT_SWITCH, false,
                RobotParams.CLIMBER_CAL_POWER)
            .setPidParams(RobotParams.CLIMBER_KP, RobotParams.CLIMBER_KI, RobotParams.CLIMBER_KD,
                          RobotParams.CLIMBER_TOLERANCE)
            .setPosRange(RobotParams.CLIMBER_MIN_POS, RobotParams.CLIMBER_MAX_POS)
            .setScaleOffset(RobotParams.CLIMBER_INCHES_PER_COUNT, 0.0);
        climber = new TrcPidActuator("climber", climberMotor, climberLowerLimitSwitch, climberUpperLimitSwitch, climberParamaters);
    }   //Climber

    /**
     * This method creates and confiugres a climber motor.
     *
     * @param name specifies the name of the motor.
     * @param canID specifies the CAN ID of the motor.
     */
    private FrcCANTalon createClimberMotor(String name, int canID)
    {
        FrcCANTalon motor = new FrcCANTalon(name, canID);

        motor.motor.configFactoryDefault();
        motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.setBrakeModeEnabled(true);

        motor.motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        motor.motor.setSensorPhase(true);
        SensorCollection sensorCollection = motor.motor.getSensorCollection();
        sensorCollection.setPulseWidthPosition(0, 10); // reset index
        TrcUtil.sleep(50); // guarantee reset
        ErrorCode error = sensorCollection.syncQuadratureWithPulseWidth(0, 0, true, RobotParams.CLIMBER_ZERO, 10);
        if (error != ErrorCode.OK)
        {
            robot.globalTracer.traceErr(moduleName, "Failed to configure encoder (error=%s).", error.name());
        }
        TrcUtil.sleep(50); // guarantee reset

        robot.globalTracer.traceInfo(
            moduleName, "Tilter: zero=%d, pwmPos=%d, quadPos=%d, selectedPos=%f",
            RobotParams.TILTER_ZERO, sensorCollection.getPulseWidthPosition(),
            sensorCollection.getQuadraturePosition(), motor.motor.getSelectedSensorPosition());

        return motor;
    }   //createClimberMotor

    public void extendPneumatic()
    {
        climberPneumatic.extend();
    }   //extendPneumatic

    public void retractPneumatic()
    {
        climberPneumatic.retract();
    }   //retractPneumatic

    public void extendClimber(double target)
    {
        climber.setTarget(target);
    }   //extendClimber

    public void retractClimber(double target)
    {
        climber.setTarget(target);
    }   //retractClimber

    public void zeroClimber()
    {
        climber.zeroCalibrate();
    }   //zeroClimber

    /**
     * Before climbing, extends the climber all the way up so that the driver can drive forward into the bar
     */
    public void prepareClimb()
    {
        //Extend pneumatic to pull out the pin
        extendPneumatic();
        //No need to extend arm since it will be spring-loaded with slack in the string
    }   //prepareClimb

    /**
     * Climbing, goes from one bar to another
     * @param goToNext Whether the robot should go to the next bar or just hang on the current bar (false for traversal)
     */
    public void ascend(boolean goToNext)
    {
        //Retract pneumatic to line up for climb
        retractPneumatic();
        //Retract climber, pull robot up
        zeroClimber();
        if(goToNext)
        {
            //Deploy hook so that it locks on the current bar
            extendPneumatic();
            //Extend climber halfway to swing robot because of CG
            extendClimber(0.2);
            //Wait for the robot to swing back
            //wait();
            //Extend climber so that it hits the next bar
            extendClimber(1.0);
        }
    }   //ascend
}   //class Climber