// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.*;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;
import java.lang.Math;
import edu.wpi.first.math.util.Units;

public class Drive extends SubsystemBase {
    WPI_TalonFX driveMotorLeftLeader;
    WPI_TalonFX driveMotorLeftFollower;
    WPI_TalonFX driveMotorRightLeader;
    WPI_TalonFX driveMotorRightFollower;
    
    /* Constants */
    public static final double InchesToMeters = 0.0254;
    public static final double TrackWidth = Units.inchesToMeters(26.5);
    public static final double CircumferenceWithTrackWidth = TrackWidth * Math.PI;
    public static final double MaxSpeed = 3.8; 
    public static final double TalonFXEncoderResolution = 2048;
    public static final double DriveGearRatio = 9.8;
    public static final double CountsPerWheelRevolution = TalonFXEncoderResolution * DriveGearRatio;
    public static final double MaxVelocityChange = MaxSpeed * 0.4; // percentage of Acceleration, smoothing value

    public static final double WheelDiameter = Units.inchesToMeters(6.0);
    public static final double MetersPerRevolution = WheelDiameter * Math.PI;
    public static final double RevolutionsPerMeter = 1.0 / MetersPerRevolution;
    public static final double MetersPerSecondToCountsPerSecond = RevolutionsPerMeter * CountsPerWheelRevolution;

    public static final double kF = 0.048;
    public static final double kP = 0.005;
    public static final double kI = 0.0001; // 0.0001 original
    public static final double kD = 0.0;
    public static final int PID_id = 0;

    DifferentialDrive diffDrive;

    public final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    public final AHRS navX = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);

    public final DifferentialDriveOdometry odometry;

    boolean simulationInitialized = false;
    private double lastLeftDistance;
    private double lastRightDistance;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TrackWidth);
    ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

    // Gains are for example purposes only - must be determined for your own robot!
    // private final SimpleMotorFeedforward m_feedforward = new
    // SimpleMotorFeedforward(1, 3);

    public Drive() {

        driveMotorLeftLeader = new WPI_TalonFX(Constants.MotorIDs.driveFrontLeft);
        driveMotorLeftFollower = new WPI_TalonFX(Constants.MotorIDs.driveBackLeft);
        driveMotorRightLeader = new WPI_TalonFX(Constants.MotorIDs.driveFrontRight);
        driveMotorRightFollower = new WPI_TalonFX(Constants.MotorIDs.driveBackRight);

        driveMotorLeftLeader.configFactoryDefault();
        driveMotorLeftFollower.configFactoryDefault();

        driveMotorRightLeader.configFactoryDefault();
        driveMotorRightFollower.configFactoryDefault();

        setMotorConfig(driveMotorLeftLeader);
        setMotorConfig(driveMotorRightLeader);
        setMotorConfig(driveMotorLeftFollower);
        setMotorConfig(driveMotorRightFollower);

        driveMotorLeftFollower.follow(driveMotorLeftLeader);
        driveMotorRightFollower.follow(driveMotorRightLeader);
        driveMotorRightLeader.setInverted(InvertType.InvertMotorOutput);
        driveMotorRightFollower.setInverted(InvertType.FollowMaster);
        driveMotorLeftFollower.setInverted(InvertType.FollowMaster);

        driveMotorLeftLeader.setSensorPhase(true);
        driveMotorRightLeader.setSensorPhase(true);

        diffDrive = new DifferentialDrive(driveMotorLeftLeader, driveMotorRightLeader);
        diffDrive.setSafetyEnabled(false);

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftDistance(), getRightDistance());

        setDefaultNeutralMode();

        CreateNetworkTableEntries();
        SmartDashboard.putData("Nav X", navX);

    }

    public void periodic() {

        odometry.update(
                gyro.getRotation2d(),
                getLeftDistance(),
                getRightDistance());

        NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(getLeftDistance());
        NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(getRightDistance());
        NetworkTableInstance.getDefault().getEntry("drive/left_encoder_count")
                .setDouble(driveMotorLeftLeader.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/right_encoder_count")
                .setDouble(driveMotorRightLeader.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/odometry/X").setDouble(odometry.getPoseMeters().getX());
        NetworkTableInstance.getDefault().getEntry("drive/odometry/Y").setDouble(odometry.getPoseMeters().getY());
        NetworkTableInstance.getDefault().getEntry("drive/odometry/heading")
                .setDouble(odometry.getPoseMeters().getRotation().getDegrees());

        NetworkTableInstance.getDefault().getEntry("drive/closedLoopErrorLeft")
                .setDouble(driveMotorLeftLeader.getClosedLoopError());
        NetworkTableInstance.getDefault().getEntry("drive/closedLoopErrorRight")
                .setDouble(driveMotorRightLeader.getClosedLoopError());

        NetworkTableInstance.getDefault().getEntry("drive/pitch").setDouble(getPitch());
    }

    private void setMotorConfig(WPI_TalonFX motor) {
        // motor.configClosedloopRamp(ClosedVoltageRampingConstant);
        // motor.configOpenloopRamp(ManualVoltageRampingConstant);
        motor.config_kF(PID_id, kF);
        motor.config_kP(PID_id, kP);
        motor.config_kI(PID_id, kI);
        motor.config_kD(PID_id, kD);

        /* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }

    public void setDefaultNeutralMode() {
        driveMotorLeftLeader.setNeutralMode(NeutralMode.Brake);
        driveMotorRightLeader.setNeutralMode(NeutralMode.Brake);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        setSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
    }

    public void stopMoving() {
        driveMotorLeftLeader.set(TalonFXControlMode.PercentOutput, 0.0);
        driveMotorRightLeader.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void setSpeeds(double leftSpeed, double rightSpeed) {
        driveMotorLeftLeader.set(TalonFXControlMode.Velocity,
                (leftSpeed * MetersPerSecondToCountsPerSecond / 10.0));
        driveMotorRightLeader.set(TalonFXControlMode.Velocity,
                (rightSpeed * MetersPerSecondToCountsPerSecond / 10.0));

        NetworkTableInstance.getDefault().getEntry("drive/left_speed").setDouble(leftSpeed);
        NetworkTableInstance.getDefault().getEntry("drive/right_speed").setDouble(rightSpeed);
    }

    public void setSpeedsWithFF(double leftSpeed, double rightSpeed, double leftFF, double rightFF) {
        driveMotorLeftLeader.set(TalonFXControlMode.Velocity,
                (leftSpeed * MetersPerSecondToCountsPerSecond / 10.0),
                DemandType.ArbitraryFeedForward, leftFF);
        driveMotorRightLeader.set(TalonFXControlMode.Velocity,
                (rightSpeed * MetersPerSecondToCountsPerSecond / 10.0),
                DemandType.ArbitraryFeedForward, rightFF);

        NetworkTableInstance.getDefault().getEntry("drive/left_speed").setDouble(leftSpeed);
        NetworkTableInstance.getDefault().getEntry("drive/right_speed").setDouble(rightSpeed);
    }

    private double getLeftSpeed() {
        double s = driveMotorLeftLeader.getSelectedSensorVelocity() * 10.0
                * (1.0 / MetersPerSecondToCountsPerSecond);
        return (s);
    }

    private double getRightSpeed() {
        double s = driveMotorRightLeader.getSelectedSensorVelocity() * 10.0
                * (1.0 / MetersPerSecondToCountsPerSecond);
        return (s);
    }

    public double getLeftDistance() {
        double d = (driveMotorLeftLeader.getSelectedSensorPosition()
                / CountsPerWheelRevolution)
                * MetersPerRevolution;
        return (d);
    }

    public double getRightDistance() {
        double d = (driveMotorRightLeader.getSelectedSensorPosition()
                / CountsPerWheelRevolution)
                * MetersPerRevolution;
        return (d);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed       Linear velocity in m/s.
     * @param rot          Angular velocity in rad/s.
     * @param squareInputs Decreases input sensitivity at low speeds.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot, boolean squareInputs) {
        DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(xSpeed, rot, squareInputs);
        // *** need to reduce max speed when arm is extended??

        double desiredLeftSpeed = speeds.left * MaxSpeed;
        double desiredRightSpeed = speeds.right * MaxSpeed;
        double deltaLeft = (desiredLeftSpeed) - getLeftSpeed();
        double deltaRight = (desiredRightSpeed) - getRightSpeed();

        // finds desired speed to get to MaxSpeed

        double leftSpeed = getLeftSpeed() + (Math.min(Math.abs(deltaLeft), MaxVelocityChange))
                * Math.signum(deltaLeft);
        double rightSpeed = getRightSpeed()
                + (Math.min(Math.abs(deltaRight), MaxVelocityChange))
                        * Math.signum(deltaRight);

        setSpeeds(leftSpeed, rightSpeed);

        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(xSpeed);
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(rot);
        NetworkTableInstance.getDefault().getEntry("drive/ik_left_speed").setDouble(speeds.left);
        NetworkTableInstance.getDefault().getEntry("drive/ik_right_speed").setDouble(speeds.right);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
        // odometry.resetPosition(gyro.getRotation2d(), 0.0, 0.0, pose);
    }

    public void resetEncoders() {
        driveMotorLeftLeader.setSelectedSensorPosition(0, 0, 30);
        driveMotorRightLeader.setSelectedSensorPosition(0, 0, 30);
    }

    public void setMaxOutput(double maxOutput) {
        diffDrive.setMaxOutput(maxOutput);
    }

    public double getGyroHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getGyroHeading());
    }

    public void resetHeading() {
        gyro.reset();
    }

    public double getPitch() {
        return navX.getRoll();
    }

    /* Simulation */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(driveMotorLeftLeader, 0.25, 21777, false);
        PhysicsSim.getInstance().addTalonFX(driveMotorLeftFollower, 0.25, 21777, false);
        PhysicsSim.getInstance().addTalonFX(driveMotorRightLeader, 0.25, 21777, false);
        PhysicsSim.getInstance().addTalonFX(driveMotorRightFollower, 0.25, 21777, false);
    }

    @Override
    public void simulationPeriodic() {
        Twist2d twist = kinematics.toTwist2d(this.getLeftDistance() - lastLeftDistance,
                this.getRightDistance() - lastRightDistance);
        NetworkTableInstance.getDefault().getEntry("drive/twist_angle").setDouble(Units.radiansToDegrees(twist.dtheta));
        gyroSim.setAngle(gyro.getAngle() - Units.radiansToDegrees(twist.dtheta));
        lastLeftDistance = this.getLeftDistance();
        lastRightDistance = this.getRightDistance();

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Pitch"));
        angle.set(5.0);
    }

    /* SmartDashboard */

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("Drive Subsystem");

        builder.addDoubleProperty("Left Distance", this::getLeftDistance, null);
        builder.addDoubleProperty("Right Distance", this::getRightDistance, null);

        builder.addDoubleProperty("Left Speed", this::getLeftSpeed, null);
        builder.addDoubleProperty("Right Speed", this::getRightSpeed, null);

        builder.addDoubleProperty("Pitch", this::getPitch, null);
    }

    /* Network Tables */

    private void CreateNetworkTableEntries() {
        NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rotation").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftVolts").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightVolts").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/pitch").setDouble(0.0);
    }
}