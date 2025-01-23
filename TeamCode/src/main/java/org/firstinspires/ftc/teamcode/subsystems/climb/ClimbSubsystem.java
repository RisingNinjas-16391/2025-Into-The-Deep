package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.CRServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final CRServo m_rightClimb;
    private final CRServo m_leftClimb;
    private final Telemetry m_telemetry;

    public ClimbSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        m_rightClimb = new CRServo(hwMap, "rightClimb");
        m_leftClimb = new CRServo(hwMap, "leftClimb");

        m_telemetry = telemetry;
    }

    @Override
    public void periodic() {
        m_telemetry.addLine("Climb");
        m_telemetry.addData("ClimbPower",m_rightClimb.get());
    }

    public void setPower(double power) {
        m_rightClimb.set(power);
        m_leftClimb.set(power);

    }
    }