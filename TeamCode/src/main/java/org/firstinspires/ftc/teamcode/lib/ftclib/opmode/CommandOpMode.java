package org.firstinspires.ftc.teamcode.lib.ftclib.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.robot.Robot;

/**
 * As opposed to the general WPILib-style Robot paradigm, FTCLib also offers a command opmode
 * for individual opmodes.
 *
 * @author Jackson
 */
public class CommandOpMode extends TimedRobotOpMode {

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void enabledInit() {

    }

    @Override
    public void enabledPeriodic() {

    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().reset();
    }
}