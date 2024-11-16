package org.firstinspires.ftc.teamcode;

import android.content.res.loader.AssetsProvider;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveCommand;
import org.firstinspires.ftc.teamcode.lib.ftclib.button.GamepadButton;
import org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.lib.pathplannerlib.commands.PathPlannerAuto;
import org.firstinspires.ftc.teamcode.lib.pathplannerlib.path.PathPlannerPath;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;

    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;

    private final GamepadButton m_resetHeading;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, telemetry);
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
        m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem,
                () -> new ChassisSpeeds(
                MathUtil.applyDeadband(m_driverController.getLeftY(),
                        .1),
                MathUtil.applyDeadband(m_driverController.getLeftX(),
                        .1),
                Math.toRadians(100 * MathUtil
                        .applyDeadband(m_driverController.getRightX(), 0.1))),
                () -> true));
    }

    public void configureButtonBindings() {

    }
    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                PathPlannerPath.fromPathFile("Sample Path");
                break;
            case 2:
                break;
            case 3:
                break;
        }

    }
}