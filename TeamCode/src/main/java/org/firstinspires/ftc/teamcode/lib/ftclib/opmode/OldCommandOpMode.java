package org.firstinspires.ftc.teamcode.lib.ftclib.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * As opposed to the general WPILib-style Robot paradigm, FTCLib also offers a command opmode
 * for individual opmodes.
 *
 * @author Jackson
 */
public abstract class OldCommandOpMode extends LinearOpMode {

    /**
     * Cancels all previous commands
     */
    public void reset() {
//        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link CommandScheduler} instance
     */
    public void run() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }

    public abstract void initialize();

    public static void disable() {
        Robot.disable();
    }

    public static void enable() {
        Robot.enable();
    }


}