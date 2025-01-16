package org.firstinspires.ftc.teamcode.subsystems.wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class WristSubsystem extends SubsystemBase {
    private final SimpleServo m_claw;

    public WristSubsystem(HardwareMap hwMap) {
        m_claw = new SimpleServo(hwMap, "wrist", 0,
                355, AngleUnit.DEGREES);

        turnToAngle(150);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Wrist");
        telemetry.addData("Angle", m_claw.getAngle());
    }

    public void turnToAngle(double angle) {
        m_claw.turnToAngle(angle);

    }

}