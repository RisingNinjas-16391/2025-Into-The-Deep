package org.firstinspires.ftc.teamcode.lib.ftclib.opmode;

/**
 * This is the Robot class. This will make your command-based robot code a lot smoother
 * and easier to understand.
 */
public class Robot {

    private static boolean isDisabled = true;

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