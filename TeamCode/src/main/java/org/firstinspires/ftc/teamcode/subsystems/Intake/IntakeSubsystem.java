package org.firstinspires.ftc.teamcode.subsystems.Intake;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx m_intake;

    public IntakeSubsystem(@NonNull HardwareMap hardwareMap){
        m_intake = hardwareMap.get(DcMotorEx.class, "intake");
        m_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_intake.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("Intake");
        telemetry.addData("Power", m_intake.getPower());

    }

    public void setPower(double power) {
        m_intake.setPower(power);
    }

    public double getPower() {
        return m_intake.getPower();
    }

    public boolean isBusy() {
        return m_intake.isBusy();
    }

}