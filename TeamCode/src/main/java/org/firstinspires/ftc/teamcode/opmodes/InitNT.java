package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "InitNT")
public class InitNT extends OpMode {
    @Override
    public void init() {
        NetworkTableInstance.getDefault().startServer();
    }

    @Override
    public void loop() {

    }
}
