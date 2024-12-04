package org.firstinspires.ftc.teamcode.subsystems.claws;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.SimpleServo;

import edu.wpi.first.wpilibj2.command.wSubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private final SimpleServo m_claw;
    private final String m_name;
    private final Telemetry m_telemetry;

    public ClawSubsystem(HardwareMap hwMap, Telemetry telemetry, String name, double initialPos) {
        m_claw = new SimpleServo(hwMap, name, 0,
                180, AngleUnit.DEGREES);
        m_name = name;

        m_telemetry = telemetry;

        turnToAngle(initialPos);
    }

    @Override
    public void periodic() {
        m_telemetry.addLine(m_name);
        m_telemetry.addData("Angle", m_claw.getAngle());
    }

    public void turnToAngle(double angle) {
        m_claw.turnToAngle(angle);

    }

}