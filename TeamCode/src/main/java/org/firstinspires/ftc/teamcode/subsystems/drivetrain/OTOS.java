package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class OTOS {
    Telemetry telemetry;
    SparkFunOTOS myOtos;
    Pose2d pose = new Pose2d();
    Pose2d poseVel = new Pose2d();

    Pose2d offset = new Pose2d(0, 0, Rotation2d.fromDegrees(90));

    public OTOS(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        configureOtos();
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");

        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(0));
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);


        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
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
        pose = OTOSPose2dtoPose2d(myOtos.getPosition());
        poseVel = OTOSPose2dtoPose2d(myOtos.getVelocity());
    }

    public static Pose2d OTOSPose2dtoPose2d(SparkFunOTOS.Pose2D otosPose) {
        return new Pose2d(otosPose.x, otosPose.y, new Rotation2d(otosPose.h));
    }
}