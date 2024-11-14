package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    private final Telemetry telemetry;

    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean isFieldCentric) {
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
    }

}
