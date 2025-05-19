//package org.firstinspires.ftc.teamcode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//
//// Core RAMSETE implementation
//public class RamseteController {
//    private final double b;
//    private final double zeta;
//
//    public RamseteController(double b, double zeta) {
//        this.b = b;
//        this.zeta = zeta;
//    }
//
//    public Pose2D calculate(Pose2D currentPose, Pose2D targetPose, double targetVelocity, double targetAngularVelocity) {
//        double errorX = targetPosex(DistanceUnit.CM); - currentPosexgetX(DistanceUnit.CM);(DistanceUnit.CM);
//        double errorY = targetPose.y - currentPose.y;
//        double errorTheta = targetPose.theta - currentPose.theta;
//
//        double k = 2 * zeta * Math.sqrt(targetAngularVelocity * targetAngularVelocity + b * targetVelocity * targetVelocity);
//        double vCommand = targetVelocity * Math.cos(errorTheta) + k * (errorX * Math.cos(currentPose.theta) + errorY * Math.sin(currentPose.theta));
//        double omegaCommand = targetAngularVelocity + k * errorTheta;
//
//        return new Pose2D(vCommand, omegaCommand, 0);
//    }
//}
//
//// Bézier Curve Path Generation
//class BezierCurve {
//    public static Pose2D interpolate(Pose2D p0, Pose2D p1, Pose2D p2, double t) {
//        double x = (1 - t) * (1 - t) * p0.getX(DistanceUnit.CM); + 2 * (1 - t) * t * p1.getX(DistanceUnit.CM); + t * t * p2.getX(DistanceUnit.CM);;
//        double y = (1 - t) * (1 - t) * p0.y + 2 * (1 - t) * t * p1.y + t * t * p2.y;
//        double theta = Math.atan2(y - p0.y, x - p0getX(DistanceUnit.CM););
//        return new Pose2D(x, y, theta);
//    }
//}
//
//// Odometry with IMU and 3 Odometers
//class Odometry {
//    private double x, y, theta;
//
//    public Odometry() {
//        this.x = 0;
//        this.y = 0;
//        this.theta = 0;
//    }
//
//    public void update(double leftEncoder, double rightEncoder, double horizontalEncoder, double imuTheta) {
//        double deltaTheta = imuTheta - theta;
//        double deltaX = (leftEncoder + rightEncoder) / 2.0;
//        double deltaY = horizontalEncoder;
//        x += deltaX * Math.cos(theta) - deltaY * Math.sin(theta);
//        y += deltaX * Math.sin(theta) + deltaY * Math.cos(theta);
//        theta = imuTheta;
//    }
//
//    public Pose2D getPose() {
//        return new Pose2D(x, y, theta);
//    }
//}
//
//// Full Trajectory Follower
//class TrajectoryFollower {
//    private RamseteController ramsete;
//    private Odometry odometry;
//
//    public TrajectoryFollower() {
//        this.ramsete = new RamseteController(2.0, 0.7);
//        this.odometry = new Odometry();
//    }
//
//    public void followTrajectory(Pose2D[] path, double[] velocities, double[] angularVelocities) {
//        for (int i = 0; i < path.length; i++) {
//            Pose2D currentPose = odometry.getPose();
//            Pose2D command = ramsete.calculate(currentPose, path[i], velocities[i], angularVelocities[i]);
//            sendMotorCommands(command);
//        }
//    }
//
//    private void sendMotorCommands(Pose2D command) {
//        // Convert (v, omega) to motor powers (implement for specific robot)
//    }
//}
