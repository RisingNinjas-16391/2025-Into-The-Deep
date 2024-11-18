package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.auto.Specimen2PartnerBackCommand;
import org.firstinspires.ftc.teamcode.commands.auto.SpecimenPreLoadandPark;
import org.firstinspires.ftc.teamcode.commands.auto.Specimen2PartnerParkCommand;
import org.firstinspires.ftc.teamcode.commands.automation.DropDropCommand;
import org.firstinspires.ftc.teamcode.commands.automation.DropIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.automation.PickUpSPCommand;
import org.firstinspires.ftc.teamcode.commands.automation.TopTransferCommand;
import org.firstinspires.ftc.teamcode.commands.automation.TransferCommand;
import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorVelocityCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoVelocityCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.Intake.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;


public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final ExtendoSubsystem m_extendoSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final IntakePivotSubsystem m_intakePivotSubsystem;
    private final OuttakePivotSubsystem m_outtakePivotSubsystem;
    private final ClawSubsystem m_outtakeClawSubsystem;
    private final IntakeSubsystem m_intakesubsystem;
    private final ColorSubsystem m_colorsensor;

    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;

    private final GamepadButton m_resetHeading;
//    private final GamepadButton m_headingN;
//    private final GamepadButton m_headingS;
//    private final GamepadButton m_headingE;
//    private final GamepadButton m_headingW;







    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, telemetry, true);
        m_extendoSubsystem = new ExtendoSubsystem(hwMap);
        m_elevatorSubsystem = new ElevatorSubsystem(hwMap);
        m_intakePivotSubsystem = new IntakePivotSubsystem(hwMap);
        m_outtakePivotSubsystem = new OuttakePivotSubsystem(hwMap);
        m_intakesubsystem = new IntakeSubsystem(hwMap);
        m_colorsensor = new ColorSubsystem(hwMap);
        m_outtakeClawSubsystem = new ClawSubsystem(hwMap, "depositClaw", 65);

        m_driverController = new GamepadEx(gamepad1);
        m_operatorController = new GamepadEx(gamepad2);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);

//        m_headingN = new GamepadButton(m_driverController, GamepadKeys.Button.X);
//        m_headingS = new GamepadButton(m_driverController, GamepadKeys.Button.B);
//        m_headingE = new GamepadButton(m_driverController, GamepadKeys.Button.Y);
//        m_headingW = new GamepadButton(m_driverController, GamepadKeys.Button.A);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
            //new OperatorPresets.IsRed==true;
        }
        if (autoNum ==10){
            setDefaultCommands();
            configureButtonBindings();
            //blue
        }
        else {
            setAutoCommands(autoNum);
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        //m_extendoSubsystem.updateTelemetry(telemetry);
        //m_elevatorSubsystem.updateTelemetry(telemetry);
        //m_outtakePivotSubsystem.updateTelemetry(telemetry);
        //m_intakePivotSubsystem.updateTelemetry(telemetry);]
        m_colorsensor.updateTelemetry(telemetry);
        m_intakesubsystem.updateTelemetry(telemetry);
        telemetry.update();
    }
    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new MecanumDriveCommand(
                m_driveSubsystem, m_driverController::getLeftY,
                m_driverController::getLeftX, m_driverController::getRightX, () -> (.6 + m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) * 1));

        m_extendoSubsystem.setDefaultCommand(new ExtendoVelocityCommand(m_extendoSubsystem, () -> 0));

        m_intakesubsystem.setDefaultCommand(new IntakeCommand(m_intakesubsystem, () -> m_driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - m_driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-.2 ));
    }

    public void configureButtonBindings() {

        //Driver Controls
        m_resetHeading.whenPressed(new InstantCommand(m_driveSubsystem::resetHeading));

        // Score on bar
        new GamepadButton(m_driverController, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SequentialCommandGroup(
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> OperatorPresets.ScoreSpecimenBack),
                new ElevatorPositionCommand(m_elevatorSubsystem, () -> OperatorPresets.HighBarBackScore)
        ));

        //Release Specimen/Sample
        new GamepadButton(m_driverController, GamepadKeys.Button.LEFT_BUMPER).whenPressed(new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90).withTimeout(300)

        ));

        new GamepadButton(m_driverController, GamepadKeys.Button.B).whenPressed(new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 60).withTimeout(300),
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> OperatorPresets.IntakeSpecimen).withTimeout(800),
                new ElevatorPositionCommand(m_elevatorSubsystem, () -> 0),
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90)
        ));

        //Old Claw INtake
       /*new GamepadButton(m_driverController, GamepadKeys.Button.X).whenPressed(new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 65),
                new WaitCommand(500),
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> 355)
        ));*/
        new GamepadButton(m_driverController, GamepadKeys.Button.X).whenPressed(new SequentialCommandGroup(
                new PickUpSPCommand(
                        m_outtakeClawSubsystem,
                        m_elevatorSubsystem,
                        m_outtakePivotSubsystem)
        ));

//        m_headingN.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 0));
//        m_headingS.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 180));
//        m_headingE.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> -90));
//        m_headingW.whenPressed(new TeleOpHeadingCommand(m_driveSubsystem, () -> 90));


        new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_DOWN).whenPressed(new SequentialCommandGroup(
                new TransferCommand(
                        m_intakePivotSubsystem,
                        m_intakesubsystem,
                        m_outtakeClawSubsystem,
                        m_elevatorSubsystem,
                        m_extendoSubsystem,
                        m_outtakePivotSubsystem)
        ));

        // Get ready to score

        new GamepadButton(m_driverController, GamepadKeys.Button.BACK).or(new GamepadButton(m_operatorController, GamepadKeys.Button.BACK)).whenActive(new TopTransferCommand(m_outtakeClawSubsystem, m_elevatorSubsystem, m_outtakePivotSubsystem));


        //Claw

        new GamepadButton(m_driverController, GamepadKeys.Button.Y).whenPressed(new ExtendoPositionCommand(m_extendoSubsystem, () -> 20));
        new GamepadButton(m_driverController, GamepadKeys.Button.A).whenPressed(new ExtendoPositionCommand(m_extendoSubsystem, () -> 0));
        new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_LEFT).whenPressed(new SequentialCommandGroup(
                new ExtendoPositionCommand(m_extendoSubsystem, () -> 40
                )
        ));

        //Operator Controls


        new GamepadButton(m_operatorController, GamepadKeys.Button.Y).whenPressed(new ElevatorPositionCommand(m_elevatorSubsystem, () -> OperatorPresets.HighBar));
        new GamepadButton(m_operatorController, GamepadKeys.Button.A).whenPressed(new ElevatorPositionCommand(m_elevatorSubsystem, () -> OperatorPresets.LowBar));

        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_UP).whenPressed(new ElevatorPositionCommand(m_elevatorSubsystem, () -> OperatorPresets.HighBucket));
        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_DOWN).whenPressed(new ElevatorPositionCommand(m_elevatorSubsystem, () -> OperatorPresets.LowBucket));


        new GamepadButton(m_operatorController, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new DropIntakeCommand(
                m_intakePivotSubsystem,
                m_intakesubsystem,
                m_outtakeClawSubsystem,
                m_elevatorSubsystem,
                m_extendoSubsystem,
                m_outtakePivotSubsystem));

        new GamepadButton(m_operatorController, GamepadKeys.Button.LEFT_BUMPER).whenPressed(new DropDropCommand(
                m_intakePivotSubsystem,
                m_intakesubsystem,
                m_outtakeClawSubsystem,
                m_elevatorSubsystem,
                m_extendoSubsystem,
                m_outtakePivotSubsystem));


        new GamepadButton(m_operatorController, GamepadKeys.Button.START).whenHeld(new ElevatorVelocityCommand(m_elevatorSubsystem, () -> -50).whenFinished(m_elevatorSubsystem::resetPosition));
        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_LEFT).whenPressed(new ExtendoVelocityCommand(m_extendoSubsystem, () -> -50).whenFinished(m_extendoSubsystem::resetPosition));
    }
    private void setAutoCommands(int chooser) {
        switch (chooser) {
            case 1:
                new SpecimenPreLoadandPark(m_driveSubsystem, m_elevatorSubsystem, m_outtakePivotSubsystem, m_outtakeClawSubsystem).schedule();
                break;
            case 2:
                new Specimen2PartnerParkCommand(m_driveSubsystem, m_elevatorSubsystem, m_outtakePivotSubsystem, m_outtakeClawSubsystem).schedule();
                break;
            case 3:
                new Specimen2PartnerBackCommand(m_driveSubsystem, m_elevatorSubsystem, m_outtakePivotSubsystem, m_outtakeClawSubsystem).schedule();
                break;



























































































































































































        }

    }
}