package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

@Autonomous(name = "BlueAuto", group = "Auto")
public class BlueAuto extends CommandOpMode {
    private Telemetry m_telemetry;
    private RobotContainer m_container;
    @Override
    public void initialize() {
        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        m_container = new RobotContainer(hardwareMap, m_telemetry, gamepad1, gamepad2, 1); //Uses heavily modified untested hardware

        waitForStart();

    }

    @Override
    public void run() {
        super.run();
        m_container.updateTelemetry(m_telemetry);
    }

}