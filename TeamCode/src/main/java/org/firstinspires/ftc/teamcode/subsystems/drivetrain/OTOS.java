package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class OTOS {
    private SparkFunOTOS myOtos;
    private Pose2d pose = new Pose2d();
    private Pose2d poseVel = new Pose2d();

    private Transform2d offset = new Transform2d(0, 0, Rotation2d.fromDegrees(0));

    public OTOS(HardwareMap hardwareMap) {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        configureOtos();
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.120, 0.060, Math.toRadians(90));
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose2d getPoseVelocity() {
        return poseVel;
    }

    public Rotation2d getRotation2d() {
        return pose.getRotation();
    }
    public void update() {
        pose = OTOSPose2dtoPose2d(myOtos.getPosition()).transformBy(offset);
        poseVel = OTOSPose2dtoPose2d(myOtos.getVelocity()).transformBy(offset);
    }

    public void setPosition(Pose2d pose) {
        offset = new Transform2d(getPose(), pose);
//        myOtos.setPosition(Pose2dtoOTOSPose2d(pose));
    }

    public static Pose2d OTOSPose2dtoPose2d(SparkFunOTOS.Pose2D otosPose) {
        return new Pose2d(otosPose.x, otosPose.y, new Rotation2d(otosPose.h));
    }

    public static SparkFunOTOS.Pose2D Pose2dtoOTOSPose2d(Pose2d pose) {
        return new SparkFunOTOS.Pose2D(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }
}