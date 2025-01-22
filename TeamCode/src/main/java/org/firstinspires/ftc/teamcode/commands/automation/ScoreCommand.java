package org.firstinspires.ftc.teamcode.commands.automation;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.commands.wrist.WristPositionCommand;
import org.firstinspires.ftc.teamcode.constants.Global;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

public class ScoreCommand extends CommandBase {


    public ScoreCommand(IntakePivotSubsystem intakePivotSubsystem, ClawSubsystem intakeClawSubsystem, WristSubsystem wristSubsystem, ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {

    }
    @Override
    public void initialize(){
        if (Global.ScoringState== Global.ScoringType.HIGHBAR) {
            new SequentialCommandGroup(
                    new TransferCommand()
            ).schedule();

        } else if (Global.ScoringState== Global.ScoringType.LOWBAR) {



        } else if (Global.ScoringState== Global.ScoringType.HIGHBUCKET) {

        }
        else if (Global.ScoringState== Global.ScoringType.HIGHBUCKET) {}
    }
}
