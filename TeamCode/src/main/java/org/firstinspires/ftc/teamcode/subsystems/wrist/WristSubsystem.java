package org.firstinspires.ftc.teamcode.subsystems.wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.SimpleServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private final SimpleServo m_claw;
    private final Telemetry m_telemetry;

    public WristSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        m_claw = new SimpleServo(hwMap, "wrist", -180,
                180, AngleUnit.DEGREES);

        m_telemetry = telemetry;
        turnToAngle(100);
    }

    @Override
    public void periodic() {
        m_telemetry.addLine("Wrist");
        m_telemetry.addData("Angle", m_claw.getAngle());
    }

    public void turnToAngle(double angle) {
        m_claw.turnToAngle(angle);

    }

}