package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors.CRServo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final CRServo m_rightClimb;
    private final CRServo m_leftClimb;
    private final String m_name;
    private final Telemetry m_telemetry;

    public ClimbSubsystem(HardwareMap hwMap, Telemetry telemetry, String name, double initialPos) {
        m_rightClimb = new CRServo(hwMap, name);
        m_leftClimb = new CRServo(hwMap, name);
        m_name = name;

        m_telemetry = telemetry;
    }

    @Override
    public void periodic() {
        m_telemetry.addLine(m_name);
    }

    public void setPower(double power) {
        m_rightClimb.set(power);
        m_leftClimb.set(power);

    }
    }