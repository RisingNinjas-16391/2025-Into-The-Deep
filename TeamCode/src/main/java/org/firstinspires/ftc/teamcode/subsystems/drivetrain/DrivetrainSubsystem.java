// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.lib.wpilib_command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
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

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final MecanumDriveKinematics m_kinematics =
            new MecanumDriveKinematics(
                    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    /** Constructs a MecanumDrive and resets the gyro. */
    public DrivetrainSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        m_frontLeftMotor = new MotorEx(hwMap, "frontLeft");
        m_frontRightMotor = new MotorEx(hwMap, "frontRight");
        m_backLeftMotor = new MotorEx(hwMap, "rearLeft");
        m_backRightMotor = new MotorEx(hwMap, "rearRight");

        m_otos = new OTOS(hwMap, telemetry);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_frontRightMotor.setInverted(true);
        m_backRightMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        m_otos.update();
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
    public void drive(
            ChassisSpeeds speeds, boolean fieldRelative) {
        var mecanumDriveWheelSpeeds =
                m_kinematics.toWheelSpeeds(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_otos.getRotation2d()) : speeds);

        mecanumDriveWheelSpeeds.desaturate(kMaxSpeed);
        setSpeeds(mecanumDriveWheelSpeeds);
    }
}