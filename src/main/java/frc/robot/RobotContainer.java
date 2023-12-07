// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.InstrumentedSequentialCommandGroup;
import frc.robot.subsystems.Drive;


public class RobotContainer {

        /* Subsystems */
        public final Drive driveSub = new Drive();

        /* Controllers */
        public Joystick driverController; // Joystick 1
        public Joystick operatorButtonController; // Joystick 2
        public Joystick operatorAxisController; // Joystick 3

        /* Buttons */
        public Trigger toggleTargetButton;
        public Trigger autoAlignButton;

        /* Drive & Arm Movement */
        public int throttleJoystickID;
        public int turnJoystickID;
        public int angleJoystickID;
        public int extensionJoystickID;

        /* Shuffleboard */
        SendableChooser<Command> autoChooser = new SendableChooser<Command>();
        final SendableChooser<String> stringChooser = new SendableChooser<String>();

        // PathPlannerTrajectory reversePath;
        // PathPlannerTrajectory engagePath;
        // PathPlannerTrajectory shortSideGamePiecePath1;
        // PathPlannerTrajectory shortSideGamePiecePath2;
        // PathPlannerTrajectory longSideGamePiecePath1;
        // PathPlannerTrajectory longSideGamePiecePath2;

        // PathPlannerTrajectory currentTrajectory = null;

        public RobotContainer() {
                configureButtonBindings();
                buildAutoOptions();
                driveSub.setDefaultCommand(
                                new DriveRobot(driveSub, driverController, throttleJoystickID, turnJoystickID));
        }


        private void configureButtonBindings() {
                driverController = new Joystick(1);
                operatorButtonController = new Joystick(2);
        }

        private void buildAutoOptions() {
                stringChooser.setDefaultOption("Engage Only", "engageOnly");
                SmartDashboard.putData("String Chooser", stringChooser);
        }

        /* * * * * * GRAB PIECE LONG COMMUNITY SIDE * * * * * */

        InstrumentedSequentialCommandGroup Example() {
                InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();

                // path = loadPath(
                //                 "autoPath", 1.0, 3.0, true);

                theCommand.addCommands(new InstantCommand(() -> theCommand.addCommands(new WaitCommand(0.5))));

                return theCommand;
        }
        // Need to fix
        // private void setupInstrumentation() {
        //         PPRamseteCommand.setLoggingCallbacks(
        //                         (PathPlannerTrajectory traj) -> {
        //                                 this.currentTrajectory = traj;
        //                         },
        //                         (Pose2d targetPose) -> {
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X")
        //                                                 .setDouble(targetPose.getX());
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y")
        //                                                 .setDouble(targetPose.getY());
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees")
        //                                                 .setDouble(targetPose.getRotation().getDegrees());
        //                         },
        //                         (ChassisSpeeds setpointSpeeds) -> {
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx")
        //                                                 .setDouble(setpointSpeeds.vxMetersPerSecond);
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy")
        //                                                 .setDouble(setpointSpeeds.vyMetersPerSecond);
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega")
        //                                                 .setDouble(setpointSpeeds.omegaRadiansPerSecond);
        //                         },
        //                         (Translation2d translationError, Rotation2d rotationError) -> {
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X")
        //                                                 .setDouble(translationError.getX());
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y")
        //                                                 .setDouble(translationError.getY());
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees")
        //                                                 .setDouble(rotationError.getDegrees());
        //                         });

        //         PathFromCurrentLocation.setLoggingCallbacks(
        //                         (PathPlannerTrajectory traj) -> {
        //                                 this.currentTrajectory = traj;
        //                         },
        //                         (Pose2d targetPose) -> {
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X")
        //                                                 .setDouble(targetPose.getX());
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y")
        //                                                 .setDouble(targetPose.getY());
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees")
        //                                                 .setDouble(targetPose.getRotation().getDegrees());
        //                         },
        //                         (ChassisSpeeds setpointSpeeds) -> {
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx")
        //                                                 .setDouble(setpointSpeeds.vxMetersPerSecond);
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy")
        //                                                 .setDouble(setpointSpeeds.vyMetersPerSecond);
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega")
        //                                                 .setDouble(setpointSpeeds.omegaRadiansPerSecond);
        //                         },
        //                         (Translation2d translationError, Rotation2d rotationError) -> {
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X")
        //                                                 .setDouble(translationError.getX());
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y")
        //                                                 .setDouble(translationError.getY());
        //                                 NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees")
        //                                                 .setDouble(rotationError.getDegrees());
        //                         });

        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X").setDouble(0.0);
        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y").setDouble(0.0);
        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees").setDouble(0.0);
        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx").setDouble(0.0);
        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy").setDouble(0.0);
        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega").setDouble(0.0);
        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X").setDouble(0.0);
        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y").setDouble(0.0);
        //         NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees").setDouble(0.0);
        // }

        public Command getAutonomousCommand() {
                // this.currentTrajectory = new PathPlannerTrajectory();

                // String choice = stringChooser.getSelected();
                // if (choice == "autoPath") {
                //         return null;
                // } else {
                //         return null;
                // }

                return null;
        }

        // public PathPlannerTrajectory getCurrentTrajectory() {
        //         return currentTrajectory;
        // }
}