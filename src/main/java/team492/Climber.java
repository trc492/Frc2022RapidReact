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

import TrcCommonLib.trclib.TrcPidActuator;
import TrcCommonLib.trclib.TrcPidActuator.Parameters;
import TrcFrcLib.frclib.FrcCANTalon;
import TrcFrcLib.frclib.FrcCANTalonLimitSwitch;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber
{
    public final FrcCANTalon climberMotor;
    public final FrcPneumatic climberPneumatic;
    public final FrcCANTalonLimitSwitch climberLowerLimitSwitch;
    public final FrcCANTalonLimitSwitch climberUpperLimitSwitch;
    public final Parameters climberParamaters;
    public final TrcPidActuator climber;

    /**
     * Constructor: Create an instance of the object.
     */
    public Climber()
    {
        climberMotor = new FrcCANTalon("climberMotor", RobotParams.CANID_CLIMBER);
        climberPneumatic = new FrcPneumatic("climberPneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.REVPH, RobotParams.PNEUMATIC_CLIMBER_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND);
        // Limit Switches may vary, not on robot yet
        climberLowerLimitSwitch = new FrcCANTalonLimitSwitch("climberLowerLimitSwitch", climberMotor, false);
        climberUpperLimitSwitch = new FrcCANTalonLimitSwitch("climberUpperLimitSwitch", climberMotor, true);
        climberParamaters = new Parameters();
        climberParamaters.setPidParams(RobotParams.CLIMBER_KP, RobotParams.CLIMBER_KI, RobotParams.CLIMBER_KD, RobotParams.CLIMBER_TOLERANCE);
        climber = new TrcPidActuator("climber", climberMotor, climberLowerLimitSwitch, climberUpperLimitSwitch, climberParamaters);
    }   //Climber

    public void extendPneumatic() {
        climberPneumatic.extend();
    }   //extendPneumatic

    public void retractPneumatic() {
        climberPneumatic.retract();
    }   //retractPneumatic

    public void extendClimber(double target) {
        climber.setTarget(target);
    }   //extendClimber

    public void retractClimber(double target) {
        climber.setTarget(target);
    }   //retractClimber

    public void zeroClimber() {
        climber.setTarget(0.0);
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