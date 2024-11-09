package org.firstinspires.ftc.teamcode.subsystems.claws;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem extends SubsystemBase {
    private final SimpleServo m_claw;
    private final String m_name;

    public ClawSubsystem(HardwareMap hwMap, String name, double initialPos) {
        m_claw = new SimpleServo(hwMap, name, 0,
                180, AngleUnit.DEGREES);
        m_name = name;

        turnToAngle(initialPos);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine(m_name);
        telemetry.addData("Angle", m_claw.getAngle());
    }

    public void turnToAngle(double angle) {
        m_claw.turnToAngle(angle);

    }

}