package org.firstinspires.ftc.teamcode.subsystems.pivot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.SimpleServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakePivotSubsystem extends SubsystemBase {
    private final SimpleServo m_leftMotor;
    private final SimpleServo m_rightMotor;

    private final Telemetry m_telemetry;

    public OuttakePivotSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        m_leftMotor = new SimpleServo(hwMap, "leftDepositPivot", 0,
                355, AngleUnit.DEGREES);
        m_rightMotor = new SimpleServo(hwMap, "rightDepositPivot", 0,
                355, AngleUnit.DEGREES);

        m_telemetry = telemetry;

    }

    @Override
    public void periodic() {
        m_telemetry.addLine("Outtake Pivot");
        m_telemetry.addData("Left Motor Angle", m_leftMotor.getAngle());
        m_telemetry.addData("Right Motor Angle", m_rightMotor.getAngle());
    }

    public void turnToAngle(double angle) {
        m_leftMotor.turnToAngle(angle);
        m_rightMotor.turnToAngle(angle);

    }

}