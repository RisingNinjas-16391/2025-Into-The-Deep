package org.firstinspires.ftc.teamcode.lib.ftclib.button;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.lib.wpilib_command.button.Trigger;

/**
 * A {@link Trigger} that gets its state from a {@link org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadEx}.
 *
 * @author Jackson
 */
public class GamepadButton extends Trigger {
    /**
     * Creates a gamepad button for triggering commands.
     *
     * @param gamepad the gamepad with the buttons
     * @param button the specified buttons
     */
    public GamepadButton(GamepadEx gamepad, @NonNull GamepadKeys.Button button) {
        super(() -> gamepad.getButton(button));
        requireNonNullParam(gamepad, "gamepad", "GamepadButton");
    }
}
