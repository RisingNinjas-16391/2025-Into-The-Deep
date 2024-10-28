package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.auto.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.TeleOpHeadingCommand;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;
    private final GamepadButton m_resetHeading;
    private final GamepadButton m_headingN;
    private final GamepadButton m_headingS;
    private final GamepadButton m_headingE;
    private final GamepadButton m_headingW;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, telemetry, true);
        m_driverController = new GamepadEx(gamepad1);
        m_operatorController = new GamepadEx(gamepad2);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);

        m_headingN = new GamepadButton(m_driverController, GamepadKeys.Button.X);
        m_headingS = new GamepadButton(m_driverController, GamepadKeys.Button.B);
        m_headingE = new GamepadButton(m_driverController, GamepadKeys.Button.Y);
        m_headingW = new GamepadButton(m_driverController, GamepadKeys.Button.A);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new MecanumDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));
    }

    public void configureButtonBindings() {
        m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));

        m_headingN.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 0));
        m_headingS.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 180));
        m_headingE.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> -90));
        m_headingW.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 90));
    }

    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new BlueAutoCommand(m_driveSubsystem).schedule();
                break;
            case 2:
                new RedAutoCommand(m_driveSubsystem).schedule();
                break;
        }

    }
}