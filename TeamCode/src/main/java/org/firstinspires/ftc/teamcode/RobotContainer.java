package org.firstinspires.ftc.teamcode;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commands.automation.DropDropCommand;
import org.firstinspires.ftc.teamcode.commands.automation.DropIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.automation.OuttakePieceCommand;
import org.firstinspires.ftc.teamcode.commands.automation.PickUpSPCommand;
import org.firstinspires.ftc.teamcode.commands.automation.TopTransferCommand;
import org.firstinspires.ftc.teamcode.commands.automation.TransferCommand;
import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.intake.FullRecursiveIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intake.PartialRecursiveIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorVelocityCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoVelocityCommand;
import org.firstinspires.ftc.teamcode.commands.wrist.WristPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.lib.ftclib.button.GamepadButton;
import org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.lib.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.opmodes.BlueAuto;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final ExtendoSubsystem m_extendoSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final IntakePivotSubsystem m_intakePivotSubsystem;
    private final OuttakePivotSubsystem m_outtakePivotSubsystem;
    private final WristSubsystem m_outtakeWristSubsystem;
    private final ClawSubsystem m_outtakeClawSubsystem;
    private final IntakeSubsystem m_intakesubsystem;
    private final ColorSubsystem m_colorsensor;

    private final GamepadEx m_driverController;
    private final GamepadEx m_operatorController;

    private final GamepadButton m_resetHeading;

    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, int autoNum) {
        m_driveSubsystem = new DrivetrainSubsystem(hwMap, telemetry);
        m_extendoSubsystem = new ExtendoSubsystem(hwMap, telemetry);
        m_elevatorSubsystem = new ElevatorSubsystem(hwMap, telemetry);
        m_intakePivotSubsystem = new IntakePivotSubsystem(hwMap, telemetry);
        m_outtakePivotSubsystem = new OuttakePivotSubsystem(hwMap, telemetry);
        m_outtakeWristSubsystem = new WristSubsystem(hwMap, telemetry);
        m_intakesubsystem = new IntakeSubsystem(hwMap, telemetry);
        m_colorsensor = new ColorSubsystem(hwMap, telemetry);
        m_outtakeClawSubsystem = new ClawSubsystem(hwMap, telemetry, "depositClaw", 50);

        m_driverController = new GamepadEx(gamepad1);
        m_operatorController = new GamepadEx(gamepad2);

        m_resetHeading = new GamepadButton(m_driverController, GamepadKeys.Button.START);

        if (autoNum == 0) {
            setDefaultCommands();
            configureButtonBindings();
        }

        registerAutoNamedCommands();
    }

    public void setDefaultCommands(){
        m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem,
                () -> new ChassisSpeeds(
                MathUtil.applyDeadband(m_driverController.getLeftY(),
                        .1),
                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                        .1),
                -Math.toRadians(100 * MathUtil
                        .applyDeadband(m_driverController.getRightX(), 0.1))),
                () -> true));

        m_extendoSubsystem.setDefaultCommand(new ExtendoVelocityCommand(m_extendoSubsystem, () -> 0));

        m_intakesubsystem.setDefaultCommand(new IntakeCommand(
                m_intakesubsystem,
                () -> m_operatorController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - m_operatorController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-.2 ));
    }

    public void configureButtonBindings() {
        //Driver Controls
        m_resetHeading.onTrue(new InstantCommand(m_driveSubsystem::resetHeading));


        //Feeding Controls
        new GamepadButton(m_driverController, GamepadKeys.Button.A).onTrue(new ParallelCommandGroup(
                new ExtendoPositionCommand(m_extendoSubsystem, () -> 5),
                new FullRecursiveIntakeCommand(
                        m_intakePivotSubsystem,
                        m_intakesubsystem,
                        m_outtakeClawSubsystem,
                        m_elevatorSubsystem,
                        m_extendoSubsystem,
                        m_outtakePivotSubsystem,
                        m_colorsensor,
                        m_outtakeWristSubsystem)

        ));
        new GamepadButton(m_driverController, GamepadKeys.Button.Y).onTrue(new ParallelCommandGroup(
                new ExtendoPositionCommand(m_extendoSubsystem, () -> 40),
                new FullRecursiveIntakeCommand(
                        m_intakePivotSubsystem,
                        m_intakesubsystem,
                        m_outtakeClawSubsystem,
                        m_elevatorSubsystem,
                        m_extendoSubsystem,
                        m_outtakePivotSubsystem,
                        m_colorsensor,
                        m_outtakeWristSubsystem)
        ));



        //Transfer Back
        new GamepadButton(m_driverController, GamepadKeys.Button.DPAD_DOWN).onTrue(new SequentialCommandGroup(
                new TransferCommand(
                        m_outtakeWristSubsystem,
                        m_intakePivotSubsystem,
                        m_intakesubsystem,
                        m_outtakeClawSubsystem,
                        m_elevatorSubsystem,
                        m_extendoSubsystem,
                        m_outtakePivotSubsystem)
        ));

        // Release Piece

        new GamepadButton(m_driverController, GamepadKeys.Button.RIGHT_BUMPER).onTrue(new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90).withTimeout(300)

        ));

        //WallFeed
        new GamepadButton(m_driverController, GamepadKeys.Button.B).onTrue(new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 60),
                new WaitCommand(.2),
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> OperatorPresets.IntakeSpecimen),
                new WaitCommand(.5),
                new ElevatorPositionCommand(m_elevatorSubsystem, () -> 0),
                new WristPositionCommand(m_outtakeWristSubsystem,()-> OperatorPresets.IntakeSpecimenWrist),
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90)
        ));

        new GamepadButton(m_driverController, GamepadKeys.Button.X).onTrue(new SequentialCommandGroup(
                new PickUpSPCommand(
                        m_outtakeClawSubsystem,
                        m_elevatorSubsystem,
                        m_outtakePivotSubsystem,
                        m_outtakeWristSubsystem
                )
        ));


        new GamepadButton(m_driverController, GamepadKeys.Button.BACK).or(new GamepadButton(m_operatorController, GamepadKeys.Button.BACK)).onTrue(new TopTransferCommand(m_outtakeClawSubsystem, m_elevatorSubsystem, m_outtakePivotSubsystem));


        //Operator Controls
        new GamepadButton(m_operatorController,GamepadKeys.Button.A).onTrue(new IntakePivotPositionCommand(m_intakePivotSubsystem, OperatorPresets.Feeding));
        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_UP).onTrue(new ElevatorPositionCommand(m_elevatorSubsystem, () -> OperatorPresets.HighBucket));
        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_DOWN).onTrue(new ElevatorPositionCommand(m_elevatorSubsystem, () -> OperatorPresets.LowBucket));

    /*
        new GamepadButton(m_operatorController, GamepadKeys.Button.RIGHT_BUMPER).onTrue(new DropIntakeCommand(
                m_intakePivotSubsystem,
                m_intakesubsystem,
                m_outtakeClawSubsystem,
                m_elevatorSubsystem,
                m_extendoSubsystem,
                m_outtakePivotSubsystem));

        new GamepadButton(m_operatorController, GamepadKeys.Button.LEFT_BUMPER).onTrue(new DropDropCommand(
                m_intakePivotSubsystem,
                m_intakesubsystem,
                m_outtakeClawSubsystem,
                m_elevatorSubsystem,
                m_extendoSubsystem,
                m_outtakePivotSubsystem));
    */


        new GamepadButton(m_operatorController, GamepadKeys.Button.START).whileTrue(new ElevatorVelocityCommand(m_elevatorSubsystem, () -> -50).andThen(m_elevatorSubsystem::resetPosition));
        new GamepadButton(m_operatorController, GamepadKeys.Button.DPAD_LEFT).whileTrue(new ExtendoVelocityCommand(m_extendoSubsystem, () -> -50).andThen(m_extendoSubsystem::resetPosition));

//        m_resetHeading.onTrue(new TurnCommand(m_driveSubsystem, () -> 90));
    }

    private void registerAutoNamedCommands() {
        //General Commands

        NamedCommands.registerCommand("ClawRelease", new SequentialCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90)
        ));

        //BucketCommands

        NamedCommands.registerCommand("HighBucket",
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(m_elevatorSubsystem, () -> OperatorPresets.HighBucket),
                        new ExtendoPositionCommand(m_extendoSubsystem, () -> 0),
                        new OuttakePivotPositionCommand(m_outtakePivotSubsystem, 355)
                        ));

        NamedCommands.registerCommand("BucketReset",
                new SequentialCommandGroup(
                        new ElevatorPositionCommand(m_elevatorSubsystem, ()-> 15),
                        new OuttakePivotPositionCommand(m_outtakePivotSubsystem, 149),
                        new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90),
                        new OuttakePivotPositionCommand(m_outtakePivotSubsystem, 355),
                        new IntakePivotPositionCommand(m_intakePivotSubsystem, OperatorPresets.Feeding)
                ));

        NamedCommands.registerCommand("OuttakePiece",
                new OuttakePieceCommand(m_extendoSubsystem, m_intakePivotSubsystem, m_intakesubsystem));

        NamedCommands.registerCommand("LowerFeeder",
                new IntakePivotPositionCommand(m_intakePivotSubsystem, OperatorPresets.Vertical));

        NamedCommands.registerCommand("DeployFeeder",
                new IntakePivotPositionCommand(m_intakePivotSubsystem, OperatorPresets.Feeding));

        NamedCommands.registerCommand("Feed1",
                new ParallelCommandGroup(
                        new ExtendoPositionCommand(m_extendoSubsystem, () -> 20),
                        new PartialRecursiveIntakeCommand(
                                m_intakePivotSubsystem,
                                m_intakesubsystem,
                                m_outtakeClawSubsystem,
                                m_elevatorSubsystem,
                                m_extendoSubsystem,
                                m_outtakePivotSubsystem,
                                m_colorsensor)
                ));

        NamedCommands.registerCommand("Feed2",
                new ParallelCommandGroup(
                        new ExtendoPositionCommand(m_extendoSubsystem, () -> 30),
                        new PartialRecursiveIntakeCommand(
                                m_intakePivotSubsystem,
                                m_intakesubsystem,
                                m_outtakeClawSubsystem,
                                m_elevatorSubsystem,
                                m_extendoSubsystem,
                                m_outtakePivotSubsystem,
                                m_colorsensor)
                ));

        NamedCommands.registerCommand("Feed3",
                new ParallelCommandGroup(
                        new ExtendoPositionCommand(m_extendoSubsystem, () -> 40),
                        new PartialRecursiveIntakeCommand(
                                m_intakePivotSubsystem,
                                m_intakesubsystem,
                                m_outtakeClawSubsystem,
                                m_elevatorSubsystem,
                                m_extendoSubsystem,
                                m_outtakePivotSubsystem,
                                m_colorsensor)
                ));


        //Specimen Commands
        NamedCommands.registerCommand("HighBar",
                new SequentialCommandGroup(
                        new ClawPositionCommand(m_outtakeClawSubsystem, () -> 30).withTimeout(0.6),
                        new ElevatorPositionCommand(m_elevatorSubsystem, () -> 44).withTimeout(0.1),
                        new WristPositionCommand(m_outtakeWristSubsystem, () -> 15),
                        new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> 285)));

            // This is for wall intaking
        NamedCommands.registerCommand("ReadyFeed", new ParallelCommandGroup(
                new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90),
                new ElevatorPositionCommand(m_elevatorSubsystem, () -> 0),
                new OuttakePivotPositionCommand(m_outtakePivotSubsystem, () -> OperatorPresets.IntakeSpecimen),
                new WristPositionCommand(m_outtakeWristSubsystem, () ->110)


        ).withTimeout(.1));


        NamedCommands.registerCommand("Turn-130",
                new TurnCommand(m_driveSubsystem, () -> -130));

        NamedCommands.registerCommand("PullDownScore",
                new SequentialCommandGroup(
                        new ElevatorPositionCommand(m_elevatorSubsystem, () -> 20),
                        new ClawPositionCommand(m_outtakeClawSubsystem, () -> 90)
                ));


    }
    public Command getAutoCommand(int chooser) {
        switch (chooser) {
            case 1:
                return null;
            case 2:
                return null;
            case 3:
                return null;
        }
        return null;
    }
}