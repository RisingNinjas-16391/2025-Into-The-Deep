package org.firstinspires.ftc.teamcode.lib.dashboard;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 6; // in


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, PathPlannerPath path, double resolution) {
        int samples = path.numPoints();
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];

        for (int i = 0; i < samples; i++) {
            Pose2d pose = path.getPathPoses().get(samples);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

//    public static void drawSampledPath(Canvas canvas, Path path) {
//        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
//    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        pose = new Pose2d(new Translation2d(-pose.getY() * 39.37 + 72, pose.getX() * 39.37 - 72), pose.getRotation().rotateBy(Rotation2d.fromDegrees(90)));

        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Rotation2d rot = pose.getRotation();
        double x1 = pose.getX() + (rot.getCos() * ROBOT_RADIUS) / 2, y1 = pose.getY() + (rot.getSin() * ROBOT_RADIUS) / 2;
        double x2 = pose.getX() + (rot.getCos() * ROBOT_RADIUS), y2 = pose.getY() + (rot.getSin() * ROBOT_RADIUS);
        canvas.strokeLine(x1, y1, x2, y2);
    }
}