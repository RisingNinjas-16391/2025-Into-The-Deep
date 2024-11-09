package org.firstinspires.ftc.teamcode.subsystems.pivot;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakePivotSubsystem extends SubsystemBase {
    private final SimpleServo m_leftMotor;
    private final SimpleServo m_rightMotor;

    public IntakePivotSubsystem(HardwareMap hwMap) {
        m_leftMotor = new SimpleServo(hwMap, "leftIntakePivot", 0,
                70, AngleUnit.DEGREES);
        m_rightMotor = new SimpleServo(hwMap, "rightIntakePivot", 0,
                70, AngleUnit.DEGREES);

        turnToAngle(20);
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Intake Pivot");
        telemetry.addData("Left Motor Angle", m_leftMotor.getAngle());
        telemetry.addData("Right Motor Angle", m_rightMotor.getAngle());
    }

    public void turnToAngle(double angle) {
        m_leftMotor.turnToAngle(angle);
        m_rightMotor.turnToAngle(angle);

    }

}