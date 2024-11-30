// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.TRANSLATION_P;
import static edu.wpi.first.units.Units.Volts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.dashboard.DashboardUtil;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.lib.wpilib.MecanumDrivePoseEstimator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a mecanum drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 1.5; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final MotorEx m_frontLeftMotor;
    private final MotorEx m_frontRightMotor;
    private final MotorEx m_backLeftMotor;
    private final MotorEx m_backRightMotor;

    private final OTOS m_otos;

    private final double robotLength = 0.195 / 2.0;
    private final double robotWidth = 0.240 / 2.0;

    private final Translation2d m_frontLeftLocation = new Translation2d(robotLength, robotWidth);
    private final Translation2d m_frontRightLocation = new Translation2d(robotLength, -robotWidth);
    private final Translation2d m_backLeftLocation = new Translation2d(-robotLength, robotWidth);
    private final Translation2d m_backRightLocation = new Translation2d(-robotLength, -robotWidth);

    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

    private final MecanumDriveKinematics m_kinematics;

    private final MecanumDrivePoseEstimator m_poseEstimator;

    private final MecanumDriveWheelPositions m_wheelPositions;
    private final MecanumDriveWheelSpeeds m_wheelSpeeds;
    private MecanumDriveWheelSpeeds m_desiredWheelSpeeds;

    private ChassisSpeeds m_desiredChassisSpeeds;

    // Gains are for example purposes only - must be determined for your own robot!
    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    private final PIDController m_frontLeftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController m_frontRightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController m_backLeftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController m_backRightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

    private final PIDController m_turnPIDController = new PIDController(DriveConstants.HEADING_P, DriveConstants.HEADING_I, DriveConstants.HEADING_D);
    private Rotation2d desiredHeading;
    private boolean m_turnControllerActive = false;

    private final FtcDashboard dashboard;

    private final Telemetry telemetry;

    private final Timer m_timer;

    /** Constructs a MecanumDrive and resets the gyro. */
    public DrivetrainSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        m_frontLeftMotor = new MotorEx(hwMap, "FL");
        m_frontRightMotor = new MotorEx(hwMap, "FR");
        m_backLeftMotor = new MotorEx(hwMap, "BL");
        m_backRightMotor = new MotorEx(hwMap, "BR");

        m_otos = new OTOS(hwMap);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_frontLeftMotor.setInverted(true);
        m_backLeftMotor.setInverted(true);

        m_frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        m_frontRightMotor.stopAndResetEncoder();
        m_frontLeftMotor.stopAndResetEncoder();
        m_backRightMotor.stopAndResetEncoder();
        m_backLeftMotor.stopAndResetEncoder();

        m_kinematics = new MecanumDriveKinematics(
                m_frontLeftLocation,
                m_frontRightLocation,
                m_backLeftLocation,
                m_backRightLocation);

        m_wheelPositions = new MecanumDriveWheelPositions();
        m_wheelSpeeds = new MecanumDriveWheelSpeeds();
        m_desiredWheelSpeeds = new MecanumDriveWheelSpeeds();

        m_desiredChassisSpeeds = new ChassisSpeeds();
        m_poseEstimator = new MecanumDrivePoseEstimator(m_kinematics, getHeading(), getWheelPositions(), new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::forceOdometry,
                this::getRobotRelativeSpeeds,
                (ChassisSpeeds speeds) -> {drive(speeds, false);},
                new HolonomicPathFollowerConfig(
                        new PIDConstants(DriveConstants.TRANSLATION_P, DriveConstants.TRANSLATION_I, DriveConstants.TRANSLATION_D),
                        new PIDConstants(DriveConstants.AUTO_HEADING_P, DriveConstants.AUTO_HEADING_I, DriveConstants.AUTO_HEADING_D),
                        1.0,
                        0.3,
                        new ReplanningConfig()),
                () -> false,
                this,
                hwMap);

        dashboard = FtcDashboard.getInstance();

        dashboard.setTelemetryTransmissionInterval(25);

        this.telemetry = telemetry;

        m_timer = new Timer();
        m_timer.start();

        forceOdometry(new Pose2d(new Translation2d(0.15, 1.66), new Rotation2d()));

        m_turnPIDController.enableContinuousInput(-180, 180);
        m_turnPIDController.setTolerance(10);
        desiredHeading = getHeading();
    }

    @Override
    public void periodic() {
        m_otos.update();

        updateWheelPositions();
        updateWheelSpeeds();

        m_poseEstimator.updateWithTime(m_timer.get(), getHeading(), m_wheelPositions);

        m_poseEstimator.addVisionMeasurement(m_otos.getPose(), m_timer.get());

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, getPose());

        DashboardUtil.drawRobot(fieldOverlay, m_otos.getPose());

        packet.put("x", getPose().getX());
        packet.put("y", getPose().getY());
        packet.put("heading (deg)", getPose().getRotation().getDegrees());

        packet.put("Front Left Speed", getWheelSpeeds().frontLeftMetersPerSecond);
        packet.put("Front Right Speed", getWheelSpeeds().frontRightMetersPerSecond);
        packet.put("Rear Left Speed", getWheelSpeeds().rearLeftMetersPerSecond);
        packet.put("Rear Right Speed", getWheelSpeeds().rearRightMetersPerSecond);

        packet.put("Desired Front Left Speed", m_desiredWheelSpeeds.frontLeftMetersPerSecond);
        packet.put("Desired Front Right Speed", m_desiredWheelSpeeds.frontRightMetersPerSecond);
        packet.put("Desired Rear Left Speed", m_desiredWheelSpeeds.rearLeftMetersPerSecond);
        packet.put("Desired Rear Right Speed", m_desiredWheelSpeeds.rearRightMetersPerSecond);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("Drivetrain");
        telemetry.addData("Pose", getPose().toString());
        telemetry.addData("Wheel Positions", getWheelPositions().toString());
        telemetry.addData("Wheel Speeds", getWheelSpeeds().toString());
        telemetry.addData("Desired Heading", desiredHeading.getDegrees());

        setSpeeds(m_desiredChassisSpeeds);
        updateGains();
        m_otos.updateScalars();


    }

    /**
     * Set the desired speeds for each wheel.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(ChassisSpeeds speeds) {
        m_desiredWheelSpeeds = m_kinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading()));

//        m_desiredWheelSpeeds.desaturate(kMaxSpeed);

        final double frontLeftFeedforward = m_feedforward.calculate(m_desiredWheelSpeeds.frontLeftMetersPerSecond);
        final double frontRightFeedforward = m_feedforward.calculate(m_desiredWheelSpeeds.frontRightMetersPerSecond);
        final double backLeftFeedforward = m_feedforward.calculate(m_desiredWheelSpeeds.rearLeftMetersPerSecond);
        final double backRightFeedforward = m_feedforward.calculate(m_desiredWheelSpeeds.rearRightMetersPerSecond);

        final double frontLeftOutput =
                m_frontLeftPIDController.calculate(
                        getWheelSpeeds().frontLeftMetersPerSecond, m_desiredWheelSpeeds.frontLeftMetersPerSecond);
        final double frontRightOutput =
                m_frontRightPIDController.calculate(
                        getWheelSpeeds().frontRightMetersPerSecond, m_desiredWheelSpeeds.frontRightMetersPerSecond);
        final double backLeftOutput =
                m_backLeftPIDController.calculate(
                        getWheelSpeeds().rearLeftMetersPerSecond, m_desiredWheelSpeeds.rearLeftMetersPerSecond);
        final double backRightOutput =
                m_backRightPIDController.calculate(
                        getWheelSpeeds().rearRightMetersPerSecond, m_desiredWheelSpeeds.rearRightMetersPerSecond);

        m_frontLeftMotor.setVoltage(frontLeftFeedforward + frontLeftOutput);
        m_frontRightMotor.setVoltage(frontRightFeedforward + frontRightOutput);
        m_backLeftMotor.setVoltage(backLeftFeedforward + backLeftOutput);
        m_backRightMotor.setVoltage(backRightFeedforward + backRightOutput);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param speeds Chassis Speeds
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        robotRelativeSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_otos.getRotation2d()) : speeds;

        m_desiredChassisSpeeds = speeds;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
//        return m_otos.getPose();
    }

    public Rotation2d getHeading() {
        return m_otos.getRotation2d();
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        return m_wheelPositions;
    }
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return m_wheelSpeeds;
    }

    public void updateWheelPositions() {
        m_wheelPositions.frontLeftMeters = m_frontLeftMotor.getCurrentPosition() * DriveConstants.GEAR_RATIO;
        m_wheelPositions.frontRightMeters = m_frontRightMotor.getCurrentPosition() * DriveConstants.GEAR_RATIO;
        m_wheelPositions.rearLeftMeters = m_backLeftMotor.getCurrentPosition() * DriveConstants.GEAR_RATIO;
        m_wheelPositions.rearRightMeters = m_backRightMotor.getCurrentPosition() * DriveConstants.GEAR_RATIO;
    }

    public void updateWheelSpeeds() {
        m_wheelSpeeds.frontLeftMetersPerSecond = m_frontLeftMotor.getVelocity() * DriveConstants.GEAR_RATIO;
        m_wheelSpeeds.frontRightMetersPerSecond = m_frontRightMotor.getVelocity() * DriveConstants.GEAR_RATIO;
        m_wheelSpeeds.rearLeftMetersPerSecond = m_backLeftMotor.getVelocity() * DriveConstants.GEAR_RATIO;
        m_wheelSpeeds.rearRightMetersPerSecond = m_backRightMotor.getVelocity() * DriveConstants.GEAR_RATIO;
    }

    public void forceOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getHeading(), getWheelPositions(), pose);
        m_otos.setPosition(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return robotRelativeSpeeds;
    }

    public MotorEx[] getMotors() {
        return new MotorEx[]{m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor};
    }

    public void setMotorVoltage(Measure<Voltage> volts) {
        m_frontLeftMotor.setVoltage(volts.in(Volts));
        m_frontRightMotor.setVoltage(volts.in(Volts));
        m_backLeftMotor.setVoltage(volts.in(Volts));
        m_backRightMotor.setVoltage(volts.in(Volts));
    }

    private void updateGains() {
        m_frontLeftPIDController.setPID(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        m_frontRightPIDController.setPID(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        m_backLeftPIDController.setPID(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        m_backRightPIDController.setPID(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

        m_turnPIDController.setPID(DriveConstants.HEADING_P, DriveConstants.HEADING_I, DriveConstants.HEADING_D);

        if (m_feedforward.ks != DriveConstants.kS || m_feedforward.kv != DriveConstants.kV || m_feedforward.ka != DriveConstants.kA) {
            m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kS);
        }
    }

    public void setHeading(double heading) {
        m_turnControllerActive = true;
        desiredHeading = Rotation2d.fromDegrees(heading);
    }

    public boolean headingIsFinished() {
        if (m_turnPIDController.atSetpoint()) {
            m_turnControllerActive = false;
        }
        return m_turnPIDController.atSetpoint();
    }
}