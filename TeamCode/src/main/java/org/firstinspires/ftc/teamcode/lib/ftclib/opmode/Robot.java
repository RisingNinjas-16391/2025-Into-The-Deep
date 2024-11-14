package org.firstinspires.ftc.teamcode.lib.ftclib.opmode;

import org.firstinspires.ftc.teamcode.lib.wpilib_command.CommandScheduler;

/**
 * This is the Robot class. This will make your command-based robot code a lot smoother
 * and easier to understand.
 */
public class Robot {

    private static boolean isDisabled = false;

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link CommandScheduler} instance
     */
    public void run() {
        CommandScheduler.getInstance().run();
    }

    public static void disable() {
        isDisabled = true;
    }

    public static void enable() {
        isDisabled = false;
    }

    public static boolean isDisabled() {
        return isDisabled;
    }

}
