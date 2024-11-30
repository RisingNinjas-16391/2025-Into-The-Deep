package org.firstinspires.ftc.teamcode.lib.ftclib.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import edu.wpi.first.networktables.NetworkTableInstance;

// An OpMode that is has methods similar to WPILib's Robot class
public abstract class TimedRobotOpMode extends OpMode {
    @Override
    public final void init() {
        NetworkTableInstance.getDefault().startServer();

        NetworkTableInstance.getDefault().getEntry("topic").setBoolean(true);

        Robot.disable();
        robotInit();
    }
    @Override
    public final void init_loop() {
        robotPeriodic();
    }

    @Override
    public final void start() {
        Robot.enable();
        enabledInit();
    }

    @Override
    public final void loop() {
        robotPeriodic();
        enabledPeriodic();
    }

    @Override
    public final void stop() {
        disabledInit();
    }

    abstract public void robotInit();

    abstract public void robotPeriodic();

    abstract public void enabledInit();

    abstract public void enabledPeriodic();

    abstract public void disabledInit();
}
