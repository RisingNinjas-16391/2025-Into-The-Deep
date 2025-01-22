package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.auto.BlueAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.RedAutoCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.MecanumDriveCommand;
<<<<<<< Updated upstream
=======
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorVelocityCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoVelocityCommand;
import org.firstinspires.ftc.teamcode.commands.wrist.WristPositionCommand;
import org.firstinspires.ftc.teamcode.constants.Global;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
>>>>>>> Stashed changes
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;


public class RobotContainer {
    private final Telemetry telemetry;

    private final DrivetrainSubsystem m_driveSubsystem;
    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;
    private final GamepadButton m_resetHeading;



<<<<<<< Updated upstream
    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum){
        this.telemetry = telemetry;
=======
    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {

>>>>>>> Stashed changes

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

    public void periodic() {
        telemetry.update();
    }

    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new MecanumDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX));
    }

    public void configureButtonBindings() {
<<<<<<< Updated upstream
//        m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));
=======


        //Driver Controls
        m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));

//
        new GamepadButton(m_driverController, GamepadKeys.Button.X).whenPressed(new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 65),
                new WaitCommand(500),
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> 355)
        ));

        m_elevatorUp.whenPressed(new InstantCommand(()->{Global.ScoringState= Global.ScoringType.HIGHBAR;}));

        // Score on bar
        new GamepadButton(m_driverController, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> Global.ScoreSpecimen)
        ));

        new GamepadButton(m_driverController, GamepadKeys.Button.LEFT_BUMPER).whenPressed(new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90)
        ));
        new GamepadButton(m_driverController, GamepadKeys.Button.B).whenPressed(new SequentialCommandGroup(
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> Global.IntakeSpecimen),
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90).withTimeout(800),
                new ElevatorPositionCommand(m_elevatorSubsystem, () -> 0)
        ));

        new GamepadButton(m_driverController, GamepadKeys.Button.X).whenPressed(new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 65),
                new WaitCommand(500),
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> 355)
        ));

//        m_headingN.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 0));
//        m_headingS.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 180));
//        m_headingE.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> -90));
//        m_headingW.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 90));

        new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_DOWN).whenPressed(new SequentialCommandGroup(
                new TransferCommand(
                        m_intakePivotSubsystem,
                        m_intakeClawSubsystem,
                        m_wristSubsystem,
                        m_outtakeClawSubsystem,
                        m_elevatorSubsystem,
                        m_extendoSubsystem,
                        m_outtakePivotSubsystem)
        ));

        // Rotate Claw
        new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_UP).toggleWhenPressed(
                new WristPositionCommand(m_wristSubsystem, () -> 355),
                new WristPositionCommand(m_wristSubsystem, () -> 150));

        // Get ready to intake
        new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_RIGHT).whenPressed(new SequentialCommandGroup(
                new ClawPositionCommand(m_intakeClawSubsystem, () -> 90),
                new IntakePivotPositionCommand(m_intakePivotSubsystem, 5)
        ));

        // Get ready to score
        new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_LEFT).whenPressed(new SequentialCommandGroup(
                new ExtendoPositionCommand(m_extendoSubsystem, () -> 40
                )
        ));

        new GamepadButton(m_driverController, GamepadKeys.Button.BACK).or(new GamepadButton(m_operatorController, GamepadKeys.Button.BACK)).whenActive(new TopTransferCommand(m_outtakeClawSubsystem, m_elevatorSubsystem, m_outtakePivotSubsystem));





        //Operator Controls


        new GamepadButton(m_driverController, GamepadKeys.Button.Y).whenPressed(new ExtendoPositionCommand(m_extendoSubsystem, () -> 20));
        new GamepadButton(m_driverController, GamepadKeys.Button.A).whenPressed(new ExtendoPositionCommand(m_extendoSubsystem, () -> 0));

        new GamepadButton(m_operatorController, GamepadKeys.Button.Y).whenPressed(new ElevatorPositionCommand(m_elevatorSubsystem, () -> Global.HighBar));
        new GamepadButton(m_operatorController, GamepadKeys.Button.A).whenPressed(new ElevatorPositionCommand(m_elevatorSubsystem, () -> Global.LowBar));

        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_UP).whenPressed(new ElevatorPositionCommand(m_elevatorSubsystem, () -> Global.HighBucket));
        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_DOWN).whenPressed(new ElevatorPositionCommand(m_elevatorSubsystem, () -> Global.LowBucket));


        new GamepadButton(m_operatorController, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DropIntakeCommand(
                m_intakePivotSubsystem,
                m_intakeClawSubsystem,
                m_wristSubsystem,
                m_outtakeClawSubsystem,
                m_elevatorSubsystem,
                m_extendoSubsystem,
                m_outtakePivotSubsystem));

        new GamepadButton(m_operatorController, GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DropDropCommand(
                m_intakePivotSubsystem,
                m_intakeClawSubsystem,
                m_wristSubsystem,
                m_outtakeClawSubsystem,
                m_elevatorSubsystem,
                m_extendoSubsystem,
                m_outtakePivotSubsystem));


        new GamepadButton(m_operatorController, GamepadKeys.Button.START).whenHeld(new ElevatorVelocityCommand(m_elevatorSubsystem, () -> -50).whenFinished(m_elevatorSubsystem::resetPosition));
        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_LEFT).whenPressed(new ExtendoVelocityCommand(m_extendoSubsystem, () -> -50).whenFinished(m_extendoSubsystem::resetPosition));
>>>>>>> Stashed changes
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