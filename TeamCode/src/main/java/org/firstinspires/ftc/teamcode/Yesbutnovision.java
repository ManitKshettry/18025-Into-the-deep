package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;
import android.util.Size;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;
import org.opencv.core.Point;
import org.opencv.video.KalmanFilter;  // OpenCV Kalman Filter
import java.util.List;
@Disabled
@Autonomous(name = "FullTrackingSystemK", group = "Concept")
public class Yesbutnovision extends LinearOpMode {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotorEx angleMotor, slideMotor;
    private IMU imu;
    private VisionPortal portal;
    private List<ColorBlobLocatorProcessor.Blob> blobs;
    private ColorBlobLocatorProcessor colorLocator;

    double x, y, tt, rawDistance, filteredDistance;
    boolean objectDetected = false;

    // ✅ Kalman Filter for Distance Estimation
    private KalmanFilter kalman;
    private org.opencv.core.Mat state, meas, processNoiseCov, measurementNoiseCov, errorCovPost;

    @Override
    public void runOpMode() {

        // ✅ Initialize Drivetrain
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ✅ Initialize Arm System (No control logic)
        angleMotor = hardwareMap.get(DcMotorEx.class, "AM");
        slideMotor = hardwareMap.get(DcMotorEx.class, "SM");
        // ✅ Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        // ✅ Setup Vision Processing
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asImageCoordinates(0, 0, 320, 240))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // ✅ Initialize Kalman Filter
        initKalmanFilter();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        waitForStart();

        while (opModeIsActive()) {
            updateVisionTracking();
            controlMovement();
            telemetry.update();
        }
    }

    // ✅ Vision-Based Object Tracking
    public void updateVisionTracking() {
        blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

        if (!blobs.isEmpty()) {
            objectDetected = true;
            ColorBlobLocatorProcessor.Blob b = blobs.get(0);
            RotatedRect boxFit = b.getBoxFit();
            Point[] pt = new Point[4];
            boxFit.points(pt);
            x = pt[3].x;
            y = pt[3].y;

            double frame_center_x = 320 / 2.0;
            double horizontal_FOV = 45.22;
            tt = ((x - frame_center_x) / 320.0) * horizontal_FOV;

            double objectSize = Math.abs(pt[3].x - pt[0].x);
            rawDistance = 618 / (objectSize + 0.2); // Tune scale factor

            telemetry.addData("size",objectSize);
            telemetry.addData("dis",rawDistance);
            telemetry.addData("better dis",filteredDistance);
            telemetry.update();

            filteredDistance = applyKalmanFilter(rawDistance);
        } else {
            objectDetected = false;
        }
    }

    public void controlMovement() {
        if (objectDetected) {
            double turnPower = 0.025 * tt;
//            double movePower = 0.02 * (filteredDistance - 5); // Move if farther than 5cm
            double movePower = (filteredDistance > 5) ? 0.02 * (filteredDistance - 5) : 0;

            // **Apply power for movement**
            frontLeftMotor.setPower(movePower + turnPower);
            backLeftMotor.setPower(movePower + turnPower);
            frontRightMotor.setPower(movePower - turnPower);
            backRightMotor.setPower(movePower - turnPower);
        } else {
            stopMotors();
        }
    }

    public void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    // ✅ Kalman Filter Initialization
    private void initKalmanFilter() {
        kalman = new KalmanFilter(2, 1, 0); // 2 state variables, 1 measurement, 0 control

        state = new org.opencv.core.Mat(2, 1, org.opencv.core.CvType.CV_32F); // x (distance), dx (velocity)
        meas = new org.opencv.core.Mat(1, 1, org.opencv.core.CvType.CV_32F); // measurement (distance)

        kalman.set_transitionMatrix(org.opencv.core.Mat.eye(2, 2, org.opencv.core.CvType.CV_32F));
        kalman.set_measurementMatrix(org.opencv.core.Mat.eye(1, 2, org.opencv.core.CvType.CV_32F));
        processNoiseCov = org.opencv.core.Mat.eye(2, 2, org.opencv.core.CvType.CV_32F);
        measurementNoiseCov = org.opencv.core.Mat.eye(1, 1, org.opencv.core.CvType.CV_32F);
        errorCovPost = org.opencv.core.Mat.eye(2, 2, org.opencv.core.CvType.CV_32F);

        processNoiseCov.put(0, 0, 1e-4);
        measurementNoiseCov.put(0, 0, 1e-1);
        errorCovPost.put(0, 0, 0.1);
        errorCovPost.put(1, 1, 0.1);
    }

    // ✅ Apply Kalman Filter
    private double applyKalmanFilter(double measuredDistance) {
        meas.put(0, 0, measuredDistance);
        kalman.correct(meas);
        org.opencv.core.Mat prediction = kalman.predict();
        return prediction.get(0, 0)[0]; // Smoothed distance
    }
}
