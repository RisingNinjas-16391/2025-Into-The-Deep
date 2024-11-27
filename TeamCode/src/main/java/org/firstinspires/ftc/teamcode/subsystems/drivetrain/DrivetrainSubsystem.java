// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Volts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pathplanner.lib.auto.AutoBuilder;
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
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final MotorEx m_frontLeftMotor;
    private final MotorEx m_frontRightMotor;
    private final MotorEx m_backLeftMotor;
    private final MotorEx m_backRightMotor;

    private final OTOS m_otos;

    private final double robotLength = 0.240;
    private final double robotWidth = 0.195;
    private final Translation2d m_frontLeftLocation = new Translation2d(robotLength, robotWidth);
    private final Translation2d m_frontRightLocation = new Translation2d(robotLength, -robotWidth);
    private final Translation2d m_backLeftLocation = new Translation2d(-robotLength, robotWidth);
    private final Translation2d m_backRightLocation = new Translation2d(-robotLength, -robotWidth);

    private final double m_gearRatio = 1.0 / 1152.006;

    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

    private final MecanumDriveKinematics m_kinematics;

    private final MecanumDrivePoseEstimator m_poseEstimator;

    private final MecanumDriveWheelPositions m_wheelPositions;
    private final MecanumDriveWheelSpeeds m_wheelSpeeds;

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.01, 6);

    private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
    private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);

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

        m_poseEstimator = new MecanumDrivePoseEstimator(m_kinematics, getHeading(), getWheelPositions(), new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::forceOdometry,
                this::getRobotRelativeSpeeds,
                (ChassisSpeeds speeds) -> {
                    drive(speeds, false);
                },
                DriveConstants.CONFIG,
                () -> false,
                this,
                hwMap);

        dashboard = FtcDashboard.getInstance();

        dashboard.setTelemetryTransmissionInterval(25);

        this.telemetry = telemetry;

        m_timer = new Timer();
        m_timer.start();
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

        packet.put("x", getPose().getY() * 39.37);
        packet.put("y", getPose().getX() * 39.37);
        packet.put("heading (deg)", getPose().getRotation().getDegrees() + 90);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("Drivetrain");
        telemetry.addData("Pose", getPose().toString());
        telemetry.addData("Wheel Positions", getWheelPositions().toString());
        telemetry.addData("Wheel Speeds", getWheelSpeeds().toString());
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

        final double frontLeftOutput =
                m_frontLeftPIDController.calculate(
                        getWheelSpeeds().frontLeftMetersPerSecond, speeds.frontLeftMetersPerSecond);
        final double frontRightOutput =
                m_frontRightPIDController.calculate(
                        getWheelSpeeds().frontRightMetersPerSecond, speeds.frontRightMetersPerSecond);
        final double backLeftOutput =
                m_backLeftPIDController.calculate(
                        getWheelSpeeds().rearLeftMetersPerSecond, speeds.rearLeftMetersPerSecond);
        final double backRightOutput =
                m_backRightPIDController.calculate(
                        getWheelSpeeds().rearRightMetersPerSecond, speeds.rearRightMetersPerSecond);

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
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return m_wheelSpeeds;
    }

    public void updateWheelPositions() {
        m_wheelPositions.frontLeftMeters = m_frontLeftMotor.getCurrentPosition() * m_gearRatio;
        m_wheelPositions.frontRightMeters = m_frontRightMotor.getCurrentPosition() * m_gearRatio;
        m_wheelPositions.rearLeftMeters = m_backLeftMotor.getCurrentPosition() * m_gearRatio;
        m_wheelPositions.rearRightMeters = m_backRightMotor.getCurrentPosition() * m_gearRatio;
    }

    public void updateWheelSpeeds() {
        m_wheelSpeeds.frontLeftMetersPerSecond = m_frontLeftMotor.getVelocity() * m_gearRatio;
        m_wheelSpeeds.frontRightMetersPerSecond = m_frontRightMotor.getVelocity() * m_gearRatio;
        m_wheelSpeeds.rearLeftMetersPerSecond = m_backLeftMotor.getVelocity() * m_gearRatio;
        m_wheelSpeeds.rearRightMetersPerSecond = m_backRightMotor.getVelocity() * m_gearRatio;
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
}