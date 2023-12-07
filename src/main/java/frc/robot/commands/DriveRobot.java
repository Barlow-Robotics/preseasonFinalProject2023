// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveRobot extends CommandBase {

    /* Inputs */
    Drive driveSub;
    Joystick driverController;
    int controllerThrottleID;
    int controllerTurnID;

    /* Variables & Constants */
    private float yawMultiplier = 1.0f;
    private double rateLimit = 7;
    private double leftVelocity;
    private double rightVelocity;

    public PIDController pid;

    SlewRateLimiter xAxisInputRamp = new SlewRateLimiter(rateLimit); 


    public DriveRobot(Drive d, Joystick driverController, int throttleID, int turnID) {
        driveSub = d;
        addRequirements(driveSub);
        
        this.driverController = driverController;
        this.controllerThrottleID = throttleID;
        this.controllerTurnID = turnID;
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        
        double x = xAxisInputRamp.calculate(driverController.getRawAxis(controllerThrottleID));
        if (Math.abs(x) < 0.01) {
            x = 0.0;
        }

        NetworkTableInstance.getDefault().getEntry("driverController/xRawAxis")
                .setDouble(driverController.getRawAxis(controllerThrottleID));

        NetworkTableInstance.getDefault().getEntry("driverController/xRampedAxis")
                .setDouble(xAxisInputRamp.calculate(driverController.getRawAxis(controllerThrottleID)));
        
        // Yaw Stuff
        double yaw = -driverController.getRawAxis(controllerTurnID); 
        if (Math.abs(yaw) < 0.01) {
            yaw = 0.0;
        }
        NetworkTableInstance.getDefault().getEntry("driverController/yawRawAxis")
                .setDouble(driverController.getRawAxis(controllerTurnID));

        double speed = -x;
        yawMultiplier = 0.6f; // 0.5f
        yaw = Math.signum(yaw) * (yaw * yaw) * yawMultiplier;

        if (Math.abs(yaw) < 0.02f) {
            yaw = 0.0f;
        }

        NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed);
        NetworkTableInstance.getDefault().getEntry("drive/averageVelocity").setDouble((leftVelocity+rightVelocity)/2);
        NetworkTableInstance.getDefault().getEntry("drive/yaw").setDouble(yaw);

        driveSub.drive(-speed, yaw * 0.8, true);
    }

    @Override
    public void end(boolean interrupted) {
        driveSub.drive(0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}