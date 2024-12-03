package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pathplanner.lib.auto.AutoBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "InitNT")
public class InitNT extends OpMode {
    @Override
    public void init() {
        NetworkTableInstance.getDefault().startServer();

//        try {
//            hardwareMap.appContext.getAssets().open("pathplanner/" + "navgrid.json");
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
//        new File(AppUtil.ROBOT_DATA_DIR, "");
    }

    @Override
    public void loop() {

    }
}
