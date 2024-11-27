package org.firstinspires.ftc.teamcode.commands.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.MutableMeasure.mutable;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class DrivetrainSysIDCommand {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance> m_distanceDrive = mutable(Meters.of(0));

    private final MutableMeasure<Angle> m_distanceAngle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocityDrive = mutable(MetersPerSecond.of(0));

    private final MutableMeasure<Velocity<Angle>> m_velocityAngle = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine;

    public DrivetrainSysIDCommand(DrivetrainSubsystem drivetrain) {
        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        (Measure<Voltage> volts) -> {
                            drivetrain.setMotorVoltage(volts);
                        },
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        log -> {
                            drivetrain.periodic();

                            log.motor("Front Left Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getMotors()[0].getVoltage(), Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(
                                                    drivetrain.getMotors()[0].getCurrentPosition(),
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(
                                                    drivetrain.getMotors()[0].getVelocity(),
                                                    MetersPerSecond));
                            log.motor("Front Right Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getMotors()[1].getVoltage(), Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(
                                                    drivetrain.getMotors()[1].getCurrentPosition(),
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(
                                                    drivetrain.getMotors()[1].getVelocity(),
                                                    MetersPerSecond));
                            log.motor("Rear Left Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getMotors()[2].getVoltage(), Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(
                                                    drivetrain.getMotors()[2].getCurrentPosition(),
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(
                                                    drivetrain.getMotors()[2].getVelocity(),
                                                    MetersPerSecond));
                            log.motor("Rear Right Drive")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    drivetrain.getMotors()[3].getVoltage(), Volts))
                                    .linearPosition(
                                            m_distanceDrive.mut_replace(
                                                    drivetrain.getMotors()[3].getCurrentPosition(),
                                                    Meters))
                                    .linearVelocity(
                                            m_velocityDrive.mut_replace(
                                                    drivetrain.getMotors()[3].getVelocity(),
                                                    MetersPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in
                        // WPILog with this subsystem's name
                        drivetrain));
    }

    public Command getQuasiStatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command getDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
