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

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber
{
    private static final String moduleName = "Climber";
    private static final boolean debugEnabled = false;

    private final Robot robot;
    public final FrcCANFalcon climberMotor;
    public final FrcPneumatic climberPneumatic;
    public final FrcDigitalInput climberLowerLimitSwitch;
    public final TrcPidActuator climber;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for accessing other global objects.
     */
    public Climber(Robot robot)
    {
        this.robot = robot;
        climberMotor = createClimberMotor(moduleName + ".motor", RobotParams.CANID_CLIMBER);
        climberPneumatic = new FrcPneumatic(
            moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
            RobotParams.PNEUMATIC_CLIMBER_RETRACT, RobotParams.PNEUMATIC_CLIMBER_EXTEND);
        climberLowerLimitSwitch = new FrcDigitalInput(
            moduleName + ".lowerLimitSwitch", RobotParams.DIO_CLIMBER_LOWER_LIMIT_SWITCH);
        Parameters params = new Parameters()
            .setPidParams(
                RobotParams.CLIMBER_KP, RobotParams.CLIMBER_KI, RobotParams.CLIMBER_KD, RobotParams.CLIMBER_TOLERANCE)
            .setPosRange(RobotParams.CLIMBER_MIN_POS, RobotParams.CLIMBER_MAX_POS)
            .setScaleOffset(RobotParams.CLIMBER_INCHES_PER_COUNT, RobotParams.CLIMBER_OFFSET)
            // .setStallProtectionParams(
            //     RobotParams.CLIMBER_STALL_MIN_POWER, RobotParams.CLIMBER_STALL_TOLERANCE,
            //     RobotParams.CLIMBER_STALL_TIMEOUT, RobotParams.CLIMBER_RESET_TIMEOUT)
            .setZeroCalibratePower(RobotParams.CLIMBER_CAL_POWER);
        climber = new TrcPidActuator("climber", climberMotor, climberLowerLimitSwitch, null, params);
    }   //Climber

    /**
     * This method creates and confiugres a climber motor.
     *
     * @param name specifies the name of the motor.
     * @param canID specifies the CAN ID of the motor.
     */
    private FrcCANFalcon createClimberMotor(String name, int canID)
    {
        FrcCANFalcon motor = new FrcCANFalcon(name, canID);

        motor.motor.configFactoryDefault();
        motor.motor.configVoltageCompSaturation(RobotParams.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyClosed, 10);
        motor.motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, 10);
        motor.motor.setSensorPhase(true);
        motor.setBrakeModeEnabled(true);
        motor.setInverted(RobotParams.CLIMBER_MOTOR_INVERTED);

        return motor;
    }   //createClimberMotor

    public boolean isLowerLimitSwitchActive()
    {
        return climberLowerLimitSwitch.isActive();
    }   //isLowerLimitSwitchActive

    //
    // Climber PID Actuator methods.
    //

    public void setPower(double power)
    {
        climber.setPower(power);
    }   //setPower

    public void setPosition(double position)
    {
        climber.setTarget(position);
    }   //setPosition

    public void extendFull()
    {
        climber.setTarget(RobotParams.CLIMBER_MAX_POS);
    }   //extendFull

    public void retractFull()
    {
        climber.setTarget(RobotParams.CLIMBER_MIN_POS);
    }   //retractFull

    public void zeroCalibrateClimber()
    {
        final String funcName = "zeroClimber";

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "RetractClimber: currPos=%.1f", climber.getPosition());
        }

        climber.zeroCalibrate();
    }   //zeroCalibrateClimber

    //
    // Climber Pneumatic methods.
    //

    public void pushOut()
    {
        climberPneumatic.extend();
    }   //pushOut

    public void pullIn()
    {
        climberPneumatic.retract();
    }   //pullIn

    //
    // Climbing methods.
    //

    /**
     * Before climbing, extends the climber all the way up so that the driver can drive forward into the bar
     */
    public void prepareClimb()
    {
        // Stall protection is for zero calibration, turn it off for the climb.
        climber.setStallProtection(0.0, 0.0, 0.0, 0.0);
        //Extend pneumatic to pull out the pin
        pushOut();
        //No need to extend arm since it will be spring-loaded with slack in the string
    }   //prepareClimb

    /**
     * Climbing, goes from one bar to another
     * @param goToNext Whether the robot should go to the next bar or just hang on the current bar (false for traversal)
     */
    public void ascend(boolean goToNext)
    {
        //Retract pneumatic to line up for climb
        pullIn();
        //Retract climber, pull robot up
        // zeroCalibrateClimber(); //CodeReview: zero calibration does not have enough power to pull the robot up????
        retractFull();
        if(goToNext)
        {
            //Deploy hook so that it locks on the current bar
            pushOut();
            //Extend climber halfway to swing robot because of CG
            setPosition(45.0);
            //Wait for the robot to swing back
            //wait();
            //Extend climber so that it hits the next bar
            extendFull();
        }
    }   //ascend

}   //class Climber