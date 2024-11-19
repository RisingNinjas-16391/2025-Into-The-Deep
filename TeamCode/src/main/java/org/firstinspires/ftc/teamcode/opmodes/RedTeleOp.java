package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "REDTeleOp", group = "Teleop")
public class RedTeleOp extends CommandOpMode {
    private Telemetry m_telemetry;
    private RobotContainer m_container;
    @Override
    public void initialize() {
        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        m_container = new RobotContainer(hardwareMap, m_telemetry, gamepad1, gamepad2, 0); //Uses heavily modified untested hardware

        waitForStart();

        OperatorPresets.IsRed = true;

    }

    @Override
    public void run() {
        super.run();
        m_container.updateTelemetry(m_telemetry);
    }



}