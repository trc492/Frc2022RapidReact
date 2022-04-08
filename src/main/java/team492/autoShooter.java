package team492;

public class autoShooter {
    public enum State{
        START,
        PREP_TO_SHOOT,
        
    }

    private void autoShootTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "autoShootTask";
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            double matchTime = TrcUtil.getModeElapsedTime();
            ShootParamTable.Params params = null;

            switch (state)
            {
                case START:
                    boolean ballAtEntrance = robot.conveyor.isEntranceSensorActive();
                    boolean ballAtExit = robot.conveyor.isExitSensorActive();

                    if (msgTracer != null)
                    {
                        msgTracer.traceInfo(funcName, "Entrance=%s, Exit=%s", ballAtEntrance, ballAtExit);
                    }

                    if (ballAtExit)
                    {
                        // Ball at the exit, shoot it.
                        sm.setState(State.PREP_TO_SHOOT);
                    }
                    else if (ballAtEntrance)
                    {
                        // No ball at the exit but there is a ball at the entrance, advance it.
                        sm.waitForSingleEvent(conveyorEvent, State.PREP_TO_SHOOT);
                        robot.conveyor.advance(currOwner, conveyorEvent);
                    }
                    else
                    {
                        // No more ball, we are done.
                        sm.setState(State.DONE);
                    }
                    break;

                case PREP_TO_SHOOT:
                    //
                    // Before we can shoot, we need to:
                    // - Spin the flywheels to the proper velocities.
                    // - Aim the shooter at the target.
                    // - Align the robot to the target.
                    //
                    // Flywheel velocities and shooter angle can be obtained from vision and interpolated from
                    // ShootParam table or provided by the caller in the form of a ShootParam table entry.
                    // Alignment angle can only be obtained by vision. If there is no vision, the caller is
                    // responsible for aligning the robot. In the scenario where vision is enabled but vision
                    // failed to find the target and the caller did not provide shoot params, we will attempt to
                    // calculate the params from robot odometry location.
                    //
                    double xPower = 0.0, yPower = 0.0, rotPower = 0.0;
                    boolean visionPidOnTarget;

                    targetAngle = null;
                    targetDistance = null;

                    // Use vision to determine shoot parameters.
                    if (usingVision && robot.vision != null)
                    {
                        if (robot.vision.targetAcquired())
                        {
                            targetAngle = robot.vision.getTargetHorizontalAngle();
                            targetDistance = robot.vision.getTargetDistance() + RobotParams.VISION_TARGET_RADIUS;

                            if (msgTracer != null)
                            {
                                msgTracer.traceInfo(
                                    funcName, "[%.3f] Vision: targetAngle=%.2f, targetDistance=%.2f",
                                    matchTime, targetAngle, targetDistance);
                            }
                        }
                    }

                    if (targetAngle == null)
                    {
                        // Either we are not using vision or vision did not detect target, use robot odometry instead.
                        double robotX = robot.robotDrive.driveBase.getXPosition();
                        double robotY = robot.robotDrive.driveBase.getYPosition();

                        targetAngle = robot.getAlignAngleFromOdometry(robotX, robotY) -
                                      robot.robotDrive.driveBase.getHeading();
                        targetDistance = TrcUtil.magnitude(robotX, robotY);

                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(
                                funcName, "[%.3f] Odometry: targetAngle=%.2f, targetDistance=%.2f",
                                matchTime, targetAngle, targetDistance);
                        }
                    }

                    if (providedParams != null)
                    {
                        // Caller provided shootParams, let's use it.
                        params = providedParams;
                    }
                    else
                    {
                        // Caller did not provide shootParams (e.g. full vision), use distance to lookup ShootParam
                        // table to interpolate/extrapolate shootParams.
                        params = robot.shootParamTable.get(targetDistance);
                    }
                    // Apply shoot parameters to flywheels and tilter.
                    // Don't need to wait for flywheel here. SHOOT_WHEN_READY will wait for it.
                    setFlywheelValue(
                        currOwner, params.lowerFlywheelVelocity, params.upperFlywheelVelocity, null);
                    // Pneumatic takes hardly any time, so fire and forget.
                    setTilterPosition(params.tilterAngle);

                    //fetch driver inputs if we are in teleOp or Test mode 
                    if (robot.isTeleop() || robot.isTest())
                    {
                        // In Teleop, we allow joystick control to drive the robot around before shooting.
                        // The joystick can control X and Y driving but vision is controlling the heading.
                        // Therefore, the robot is always aiming at the vision target. However, in case
                        // vision was wrong, we also allow the driver to override vision by controlling turn
                        // using joystick.
                        double[] inputs = robot.robotDrive.getDriveInputs();
                        xPower = inputs[0]*0.3;
                        yPower = inputs[1]*0.3;
                        rotPower = inputs[2]*0.3;
                    }

                    if (visionAlignEnabled && rotPower == 0.0)
                    {
                        // Vision alignment is enabled and driver is not overriding.
                        rotPower = alignPidCtrl.getOutput();
                        visionPidOnTarget = alignPidCtrl.isOnTarget();
                    }
                    else
                    {
                        // Vision alignment is disabled or driver is overriding. We'll say it's always ontarget.
                        visionPidOnTarget = true;
                    }
                    robot.robotDrive.driveBase.holonomicDrive(currOwner, xPower, yPower, rotPower);

                    if (RobotParams.Preferences.debugShooter)
                    {
                        robot.dashboard.displayPrintf(
                            11, "x=%.1f, y=%.1f, rot=%.1f, onTarget=%s", xPower, yPower, rotPower, visionPidOnTarget);
                    }
                    //
                    // In Teleop mode, let the operator controls when to shoot. In autonomous mode, only allows
                    // shooting when vision is onTarget.
                    //
                    if (robot.isTeleop() || visionPidOnTarget)
                    {
                        allowToShoot = true;
                        //if we are in autonomous this means visionPidOnTarget so we can go to SHOOT_WHEN_READY 
                        //if we are in teleOp readyToShoot means Tim has released joystick so we can go to SHOOT_WHEN_READY
                        if(robot.isAutonomous() || readyToShoot){
                                robot.robotDrive.driveBase.stop();
                                robot.robotDrive.setAntiDefenseEnabled(currOwner, true);
                                sm.setState(State.SHOOT_WHEN_READY);
                        }
                    }
                    break;

                case SHOOT_WHEN_READY:
                    ballAtEntrance = robot.conveyor.isEntranceSensorActive();
                    ballAtExit = robot.conveyor.isExitSensorActive();

                    if (ballAtExit)
                    {
                        // Ball at the exit, shoot it.
                        if (isFlywheelVelOnTarget())
                        {
                            sm.waitForSingleEvent(conveyorEvent, State.SHOOT_WHEN_READY);
                            robot.conveyor.advance(currOwner, conveyorEvent);
                            if (msgTracer != null)
                            {
                                msgTracer.traceInfo(
                                    funcName, "Shot a ball (lowerFlywheel=%.0f, upperFlywheel=%.0f).",
                                    getLowerFlywheelVelocity(), getUpperFlywheelVelocity());
                            }
                        }
                    }
                    else if (ballAtEntrance)
                    {
                        // No ball at the exit but there is a ball at the entrance, advance it.
                        sm.waitForSingleEvent(conveyorEvent, State.SHOOT_WHEN_READY);
                        robot.conveyor.advance(currOwner, conveyorEvent);
                        if (msgTracer != null)
                        {
                            msgTracer.traceInfo(funcName, "Advance ball to exit.");
                        }
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                default:
                    cancel();
                    break; 
            }
        }

        if (msgTracer != null)
        {
            msgTracer.traceStateInfo(
                state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive, robot.robotDrive.purePursuitDrive,
                null);
        }
    }   //autoShootTask

}
