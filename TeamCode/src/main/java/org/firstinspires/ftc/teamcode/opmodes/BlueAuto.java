package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pathplanner.lib.auto.AutoBuilder;
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class BlueAuto extends CommandOpMode {
    private Telemetry m_telemetry;
    private RobotContainer m_robotContainer;

    private String auto;

    public BlueAuto(String auto) {
        this.auto = auto;
    }

    @Override
    public void robotInit() {
        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m_robotContainer = new RobotContainer(hardwareMap, m_telemetry, gamepad1, gamepad2, 1); //Uses heavily modified untested hardware
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        m_telemetry.update();
    }

    @Override
    public void enabledInit() {
        AutoBuilder.buildAuto(auto).schedule();
    }

    @OpModeRegistrar
    public static void register(AnnotatedOpModeManager opModeManager) {
        String[] names = {"ScoreCenter", "Frontal6Specimen"};
        for (String name : names) {
            opModeManager.register(
                    new OpModeMeta.Builder()
                            .setName(name)
                            .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                            .build(),
                    new BlueAuto(name));
        }
    }


}