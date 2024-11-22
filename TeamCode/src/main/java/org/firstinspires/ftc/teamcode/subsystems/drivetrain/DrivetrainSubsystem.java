// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.dashboard.DashboardUtil;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.lib.pathplannerlib.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.lib.wpilib_command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

/** Represents a mecanum drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final MotorEx m_frontLeftMotor;
    private final MotorEx m_frontRightMotor;
    private final MotorEx m_backLeftMotor;
    private final MotorEx m_backRightMotor;
    private final OTOS m_otos;

    private final Translation2d m_frontLeftLocation = new Translation2d(0.195, 0.240);
    private final Translation2d m_frontRightLocation = new Translation2d(0.195, -0.240);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.195, 0.240);
    private final Translation2d m_backRightLocation = new Translation2d(-0.195, -0.240);

    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

    private final MecanumDriveKinematics m_kinematics;

    private final MecanumDrivePoseEstimator m_poseEstimator;

    private final MecanumDriveWheelPositions m_wheelPositions;

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.01, 6);

    private final FtcDashboard dashboard;

    /** Constructs a MecanumDrive and resets the gyro. */
    public DrivetrainSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        m_frontLeftMotor = new MotorEx(hwMap, "FL");
        m_frontRightMotor = new MotorEx(hwMap, "FR");
        m_backLeftMotor = new MotorEx(hwMap, "BL");
        m_backRightMotor = new MotorEx(hwMap, "BR");

        m_otos = new OTOS(hwMap, telemetry);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_frontRightMotor.setInverted(true);
        m_backRightMotor.setInverted(true);

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

        m_poseEstimator = new MecanumDrivePoseEstimator(m_kinematics, getHeading(), getWheelPositions(), new Pose2d());

        m_wheelPositions = new MecanumDriveWheelPositions();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::forceOdometry,
                this::getRobotRelativeSpeeds,
                (ChassisSpeeds speeds) -> {
                    drive(speeds, false);
                },
                DriveConstants.CONFIG,
                () -> false
                ,
                this,
                hwMap);

        dashboard = FtcDashboard.getInstance();

        dashboard.setTelemetryTransmissionInterval(25);
    }

    @Override
    public void periodic() {
        m_otos.update();

        updateWheelPositions();

        m_poseEstimator.update(getHeading(), m_wheelPositions);

        m_poseEstimator.addVisionMeasurement(m_otos.getPose(), 0.02);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, getPose());

        packet.put("x", getPose().getX() * 39.37);
        packet.put("y", getPose().getY() * 39.37);
        packet.put("heading (deg)", getPose().getRotation().getDegrees());

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Set the desired speeds for each wheel.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
        final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
        final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
        final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
        final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

        m_frontLeftMotor.setVoltage(frontLeftFeedforward);
        m_frontRightMotor.setVoltage(frontRightFeedforward);
        m_backLeftMotor.setVoltage(backLeftFeedforward);
        m_backRightMotor.setVoltage(backRightFeedforward);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param speeds Chassis Speeds
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        robotRelativeSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_otos.getRotation2d()) : speeds;

        var mecanumDriveWheelSpeeds =
                m_kinematics.toWheelSpeeds(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_otos.getRotation2d()) : speeds);

        mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
        setSpeeds(mecanumDriveWheelSpeeds);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return m_otos.getRotation2d();
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        return m_wheelPositions;
    }

    public void updateWheelPositions() {
        m_wheelPositions.frontLeftMeters = m_frontLeftMotor.getCurrentPosition();
        m_wheelPositions.frontRightMeters = m_frontRightMotor.getCurrentPosition();
        m_wheelPositions.rearLeftMeters = m_backLeftMotor.getCurrentPosition();
        m_wheelPositions.rearRightMeters = m_backRightMotor.getCurrentPosition();
    }

    public void forceOdometry(Pose2d pose) {
        m_otos.setPosition(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return robotRelativeSpeeds;
    }
}