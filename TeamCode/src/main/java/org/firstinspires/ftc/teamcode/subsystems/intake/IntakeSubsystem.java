package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx m_intake;
    private final Telemetry m_telemetry;

    public IntakeSubsystem(@NonNull HardwareMap hardwareMap, Telemetry telemetry){
        m_intake = hardwareMap.get(DcMotorEx.class, "intake");
        m_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_intake.setDirection(DcMotorSimple.Direction.FORWARD);

        m_telemetry = telemetry;

    }

    @Override
    public void periodic() {
        m_telemetry.addLine("Intake");
        m_telemetry.addData("Power", m_intake.getPower());
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