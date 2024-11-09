package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;

/**
 * A subsystem that uses the {@link MecanumDrive} class.
 * This periodically calls {@link MecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class DrivetrainSubsystem extends SubsystemBase {
    private final Telemetry telemetry;

    private final Localizer localizer;

    private final MecanumDrive drive;
    private final boolean fieldCentric;

    private final PIDFController kHeadingPID;
    private double desiredHeading = 0;
    public static double omegaSpeed = 0.5;
    private double m_headingOffset = 0;

    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean isFieldCentric) {
        this.telemetry = telemetry;

        localizer = new OTOSLocalizer(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, localizer);
        fieldCentric = isFieldCentric;

        kHeadingPID = new PIDFController(DriveConstants.TELEOP_HEADING_PID);
        kHeadingPID.setInputBounds(0, 2 * Math.PI);
    }

    @Override
    public void periodic() {
        drive.updatePoseEstimate();
        telemetry.addLine("Drivetrain");
        telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
        telemetry.addData("Desired Heading", desiredHeading);
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
                fieldCentric ? -poseEstimate.getHeading() : 0
        );

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightX
                )
        );
    }

    public void driveHeadingPID(double leftY, double leftX, double rightX) {
        drive(leftY, leftX, calculatePID());

        if (Math.abs(rightX) > 0.1) {
            setHeading(getHeading() + rightX * omegaSpeed);
        }
    }

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectorySequence(TrajectorySequence trajectory) {
        drive.followTrajectorySequenceAsync(trajectory);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    public void setHeading(double heading) {
        desiredHeading = heading;
    }

    public double getHeading() {
        return drive.getHeading() - m_headingOffset;
    }

    public double calculatePID() {
        kHeadingPID.setTargetPosition(desiredHeading);
        return kHeadingPID.update(getHeading());
    }

    public boolean isFinishedHeadingPID() {
        return (desiredHeading - getHeading()) < 0.2;
    }

    public void resetHeading() {
        m_headingOffset = getHeading();
    }

}
