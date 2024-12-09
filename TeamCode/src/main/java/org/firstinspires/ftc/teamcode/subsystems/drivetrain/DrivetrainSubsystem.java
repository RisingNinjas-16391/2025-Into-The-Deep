// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants.TRANSLATION_P;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.dashboard.DashboardUtil;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.lib.wpilib.MecanumDrivePoseEstimator;

import java.util.Arrays;

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
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a mecanum drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
    private final MotorEx m_frontLeftMotor;
    private final MotorEx m_frontRightMotor;
    private final MotorEx m_backLeftMotor;
    private final MotorEx m_backRightMotor;

    private final OTOS m_otos;

    private final MecanumDriveKinematics m_kinematics;

    private final MecanumDrivePoseEstimator m_poseEstimator;

    private final MecanumDriveWheelPositions m_wheelPositions;
    private final MecanumDriveWheelSpeeds m_wheelSpeeds;
    private MecanumDriveWheelSpeeds m_desiredWheelSpeeds;

    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    private final PIDController m_frontLeftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController m_frontRightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController m_backLeftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController m_backRightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

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
                DriveConstants.FRONT_LEFT_LOCATION,
                DriveConstants.FRONT_RIGHT_LOCATION,
                DriveConstants.REAR_LEFT_LOCATION,
                DriveConstants.REAR_RIGHT_LOCATION);

        m_wheelPositions = new MecanumDriveWheelPositions();
        m_wheelSpeeds = new MecanumDriveWheelSpeeds();
        m_desiredWheelSpeeds = new MecanumDriveWheelSpeeds();

        m_poseEstimator = new MecanumDrivePoseEstimator(m_kinematics, getHeading(), getWheelPositions(), new Pose2d());

        try{
            RobotConfig config = RobotConfig.fromGUISettings(hwMap);

            // Configure AutoBuilder
            AutoBuilder.configure(
                    this::getPose,
                    this::forceOdometry,
                    this::getRobotRelativeSpeeds,
                    this::driveRobotRelative,
                    new PPHolonomicDriveController(
                            new PIDConstants(DriveConstants.TRANSLATION_P, DriveConstants.TRANSLATION_I, DriveConstants.TRANSLATION_D),
                            new PIDConstants(DriveConstants.HEADING_P, DriveConstants.HEADING_I, DriveConstants.HEADING_D)
                    ),
                    config,
                    () -> {
//                        // Boolean supplier that controls when the path will be mirrored for the red alliance
//                        // This will flip the path being followed to the red side of the field.
//                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
//
//                        var alliance = DriverStation.getAlliance();
//                        if (alliance.isPresent()) {
//                            return alliance.get() == DriverStation.Alliance.Red;
//                        }
//                        return false;
                        return false;
                    },
                    hwMap,
                    this
            );
        }catch(Exception e){
            throw new AutoBuilderException(e.toString() + Arrays.toString(e.getStackTrace()));
        }

        this.telemetry = telemetry;

        m_timer = new Timer();
        m_timer.start();

        //forceOdometry(new Pose2d(new Translation2d(0.15, 1.66), Rotation2d.fromDegrees(180)));
    }

    @Override
    public void periodic() {
        m_otos.update();

        updateWheelPositions();
        updateWheelSpeeds();

        m_poseEstimator.updateWithTime(m_timer.get(), getHeading(), m_wheelPositions);

        m_poseEstimator.addVisionMeasurement(m_otos.getPose(), m_timer.get());

        telemetry.addLine("Drivetrain");
        telemetry.addData("Pose", getPose().toString());
        telemetry.addData("Wheel Positions", getWheelPositions().toString());
        telemetry.addData("Wheel Speeds", getWheelSpeeds().toString());

        updateGains();
        m_otos.updateScalars();
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        this.robotRelativeSpeeds = robotRelativeSpeeds;

        m_desiredWheelSpeeds = m_kinematics.toWheelSpeeds(robotRelativeSpeeds);

        final Voltage frontLeftFeedforward = m_feedforward.calculate(MetersPerSecond.of(m_desiredWheelSpeeds.frontLeftMetersPerSecond));
        final Voltage frontRightFeedforward = m_feedforward.calculate(MetersPerSecond.of(m_desiredWheelSpeeds.frontRightMetersPerSecond));
        final Voltage backLeftFeedforward = m_feedforward.calculate(MetersPerSecond.of(m_desiredWheelSpeeds.rearLeftMetersPerSecond));
        final Voltage backRightFeedforward = m_feedforward.calculate(MetersPerSecond.of(m_desiredWheelSpeeds.rearRightMetersPerSecond));

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

        m_frontLeftMotor.setVoltage(frontLeftFeedforward.magnitude() + frontLeftOutput);
        m_frontRightMotor.setVoltage(frontRightFeedforward.magnitude() + frontRightOutput);
        m_backLeftMotor.setVoltage(backLeftFeedforward.magnitude() + backLeftOutput);
        m_backRightMotor.setVoltage(backRightFeedforward.magnitude() + backRightOutput);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeading());
        driveRobotRelative(fieldRelativeSpeeds);
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

    public void resetHeading() {
        forceOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return robotRelativeSpeeds;
    }

    public MotorEx[] getMotors() {
        return new MotorEx[]{m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor};
    }

    public void setMotorVoltage(Voltage volts) {
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

        if (m_feedforward.getKs() != DriveConstants.kS || m_feedforward.getKv() != DriveConstants.kV || m_feedforward.getKa() != DriveConstants.kA) {
            m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kS);
        }
    }
}