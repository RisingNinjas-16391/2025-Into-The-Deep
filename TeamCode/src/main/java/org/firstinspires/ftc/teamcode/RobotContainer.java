package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;

    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;

    private final GamepadButton m_resetHeading;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, telemetry, true);

        m_driverController = new GamepadEx(gamepad1);
        m_operatorController = new GamepadEx(gamepad2);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        } else {
            setAutoCommands(autoNum);
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.update();
    }
    public void setDefaultCommands(){

    }

    public void configureButtonBindings() {

    }
    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
        }

    }
}