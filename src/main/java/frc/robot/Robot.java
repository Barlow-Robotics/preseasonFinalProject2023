// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.sim.PhysicsSim;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;
    boolean autoActivated;
    private Field2d gameField;

    static long startTime = System.currentTimeMillis();

    static HashMap<Command, Long> startTimes = new HashMap();

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        DriverStation.silenceJoystickConnectionWarning(true);

        gameField = new Field2d();
        SmartDashboard.putData("Field", gameField);

        CommandScheduler.getInstance().onCommandInitialize(Robot::reportCommandStart);
        CommandScheduler.getInstance().onCommandFinish(Robot::reportCommandFinish);
        CommandScheduler.getInstance().onCommandInterrupt(this::handleInterrupted);

        Logger logger = Logger.getInstance();

        logger.recordMetadata("ProjectName", "WPI-Swerve-Prototype"); // Set a metadata value
    
        if (isReal()) {
            // Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            // setUseTiming(false); // Run as fast as possible
            // Stringf logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            logger.addDataReceiver(new WPILOGWriter(""));
            logger.addDataReceiver(new NT4Publisher());
        }
        
        // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    
    }

    @Override
    public void robotPeriodic() {
        
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(robotContainer.driveSub);

        // if (robotContainer.getCurrentTrajectory() != null) {
        //     gameField.getObject("traj").setTrajectory(robotContainer.getCurrentTrajectory());
        // }
        gameField.setRobotPose(robotContainer.driveSub.getPose());

        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        autoActivated = false;
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().clearComposedCommands();

        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
            System.out.println("Auto Command is " + autonomousCommand);
        } else {
            System.out.println("Auto Command is null");
        }
        autoActivated = true;
    }

    // @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        autoActivated = false;
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        autoActivated = false;
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        autoActivated = false;
        robotContainer.driveSub.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    public boolean ifAutonomous() {
        return autoActivated;
    }

    static public void reportCommandStart(Command c) {
        double deltaTime = ((double) System.currentTimeMillis() - startTime) / 1000.0;
        System.out.println(deltaTime + ": Started " + c.getName());
        startTimes.putIfAbsent(c, System.currentTimeMillis());
    }

    static public void reportCommandFinish(Command c) {
        if (startTimes.containsKey(c)) {
            long  currentTime = System.currentTimeMillis();
            double deltaTime = ((double) currentTime - startTime) / 1000.0;
            double elapsedTime = (double) (currentTime - startTimes.get(c)) / 1000.0;
            System.out.println(deltaTime + ": Finished (elapsed time " + elapsedTime + ")" + c.getName());
            startTimes.remove(c);
        }
    }

    private void handleInterrupted(Command c) {
        System.out.println("Commmand " + c + " named " + c.getName() + " was interrupted");
    }
}
