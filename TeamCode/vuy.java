public class vuy {
}
// /*
//  * Copyright (c) 2024 Phil Malone
//  *
//  * Permission is hereby granted, free of charge, to any person obtaining a copy
//  * of this software and associated documentation files (the "Software"), to deal
//  * in the Software without restriction, including without limitation the rights
//  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  * copies of the Software, and to permit persons to whom the Software is
//  * furnished to do so, subject to the following conditions:
//  *
//  * The above copyright notice and this permission notice shall be included in all
//  * copies or substantial portions of the Software.
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  * SOFTWARE.
//  */

// package org.firstinspires.ftc.teamcode.pedroPathing.examples;

// import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
// import android.util.Config;
// import static java.lang.Thread.sleep;

// import android.util.Size;
// import org.firstinspires.ftc.vision.opencv.ImageRegion;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.hardware.lynx.LynxModule;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.DcMotor;
// // import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.Servo;
// import org.firstinspires.ftc.vision.opencv.ColorSpace;

// //import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
// import org.firstinspires.ftc.vision.opencv.ColorRange;
// import org.firstinspires.ftc.vision.opencv.ImageRegion;
// import org.opencv.core.RotatedRect;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;

// import java.util.List;


// //@Disabled
// @Config
// @Autonomous(name = "inrishitbhaiyawetrust", group = "Concept")
// public class WallVisionnew extends LinearOpMode {

//     double x;
//     double y;
//     double avgx = 0;
//     double sumx = 0;
//     double avgy = 0;
//     double sumy = 0;

//     private DcMotor backRightMotor;
//     private DcMotor backLeftMotor;
//     private DcMotor frontRightMotor;
//     private DcMotor frontLeftMotor;
//     private IMU imu;


//     boolean arm = true;
//     String mode = "Field";


//     //intake
//     private DcMotorEx angleMotor;
//     private DcMotorEx slideMotor;
//     private CRServo R_Roll;
//     private CRServo L_Roll;

//     double angleBrakePower;
//     double slideBrakePower = 0;
//     double servoBusCurrent;
//     double power = 0.8;
//     // Inside the WallVisionnew class

//     List<ColorBlobLocatorProcessor.Blob> blobs;
//     ColorBlobLocatorProcessor colorLocator;


//     @Override
//     public void runOpMode() {
//         //wheelbase
//         frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//         backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//         frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//         backRightMotor = hardwareMap.dcMotor.get("rightBack");


//         //intake
//         angleMotor = hardwareMap.get(DcMotorEx.class, "AM");
//         slideMotor = hardwareMap.get(DcMotorEx.class, "SM");
//         R_Roll = hardwareMap.get(CRServo.class, "R_Roll");
//         L_Roll = hardwareMap.get(CRServo.class, "L_Roll");
//         slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//         backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//         // Retrieve the IMU from the hardware map
//         imu = hardwareMap.get(IMU.class, "imu");
//         // Adjust the orientation parameters to match your robot
//         IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                 RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                 RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
//         // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//         imu.initialize(parameters);


//         colorLocator = new ColorBlobLocatorProcessor.Builder()
//                 .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
//                 .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//                 .setRoi(ImageRegion.asImageCoordinates(0, 0, 320, 240))  // search central 1/4 of camera view
//                 .setDrawContours(true)                        // Show contours on the Stream Preview
//                 .setBlurSize(5)                               // Smooth the transitions between different colors in image
//                 .build();

//         // ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
//         //         .setTargetColorRange(ColorRange.RED)         // use a predefined color match
//         //         .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
//         //         .setRoi(ImageRegion.asImageCoordinates(0, 0,  320, 240))  // search central 1/4 of camera view
//         //         .setDrawContours(true)                        // Show contours on the Stream Preview
//         //         .setBlurSize(5)                               // Smooth the transitions between different colors in image
//         //         .build();


//         VisionPortal portal = new VisionPortal.Builder()
//                 .addProcessor(colorLocator)
//                 .setCameraResolution(new Size(320, 240))
//                 .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                 .build();

//         telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
//         telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

//         blobs = colorLocator.getBlobs();
//         ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

//         waitForStart();

//         while (opModeIsActive()) {
//             double vertical = 0;
//             double horizontal = 0;
//             float pivot;
//             if(x>110){
//                 horizontal = 0.20f;
//             }
//             else if(x<90)
//                 horizontal=-0.20f;
//             else
//                 horizontal = 0;




//                 if (gamepad1.left_trigger > 0) {
//                     pivot = -0;
//                 } else if (gamepad1.right_trigger > 0) {
//                     pivot = +gamepad1.right_trigger;
//                 } else {
//                     pivot = 0;
//                 }

//             frontRightMotor.setPower((-pivot + (vertical - horizontal)));
//             backRightMotor.setPower((-pivot + vertical + horizontal));
//             frontLeftMotor.setPower((pivot + vertical + horizontal));
//             backLeftMotor.setPower((pivot + (vertical - horizontal)));
//             telemetry.addData("x", x);
//             telemetry.addData("y", y);
//             telemetry.addData("avgx", avgx);
//             telemetry.addData("avgy", avgy);
//             telemetry.addData("blobs", blobs.size());
//             telemetry.addData("vert", vertical);
//             telemetry.addData("hor", horizontal);
//             // telemetry.addData("is align", align);
//             telemetry.update();


//             //     sumx = 0;
//             //     sumy = 0;

//             //     for (int i = 0; i <= 99; i++) {

//             blobs = colorLocator.getBlobs();
//             ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

//             ColorBlobLocatorProcessor.Blob b = blobs.get(0);
//             RotatedRect boxFit = b.getBoxFit();

//             Point pt[] = new Point[4];
//             boxFit.points(pt);
//             x = pt[3].x;//bottom left corner
//             y = pt[3].y;

//             //         telemetry.addData("x", x);
//             //         telemetry.addData("y", y);
//             //         telemetry.addData("avgx", avgx);
//             //         telemetry.addData("avgy", avgy);
//             //         telemetry.addData("blobs", blobs.size());
//             //         telemetry.addData("vert", vertical);
//             //         telemetry.addData("hor", horizontal);
//             //         // telemetry.addData("is align", align);
//             //         telemetry.update();
//             //         sleep(20);

//             //         sumx += x;
//             //         sumy += y;


//             //     }
//             //     if (!blobs.isEmpty()) {

//             //     }

//             //     avgx = sumx / 100;
//             //     avgy = sumy / 100;


//             //     telemetry.addData("x", x);
//             //     telemetry.addData("y", y);
//             //     telemetry.addData("avgx", avgx);
//             //     telemetry.addData("avgy", avgy);
//             //     telemetry.addData("blobs", blobs.size());
//             //     telemetry.addData("vert", vertical);
//             //     telemetry.addData("hor", horizontal);
//             //     // telemetry.addData("is align", align);
//             //     telemetry.update();
//             //     sleep(50);
//             //     moveRobot(avgx,avgy);
//             // }

//         }
//         // private void moveRobot(double avgx, double avgy) {


// //            double targetX = 100; // Target X coordinate
// //            double targetY = 100; // Target Y coordinate
// //
// //            // Calculate errors
// //            double xError = targetX - avgx;
// //            double yError = targetY - avgy;
// //
// //            // Update error sums (integral term)
// //            xErrorSum += xError;
// //            yErrorSum += yError;
// //
// //            // Calculate rate of change of error (derivative term)
// //            double xErrorRate = xError - lastXError;
// //            double yErrorRate = yError - lastYError;
// //
// //            // Calculate PID output for X and Y
// //            double xCorrection = kP * xError + kI * xErrorSum + kD * xErrorRate;
// //            double yCorrection = kP * yError + kI * yErrorSum + kD * yErrorRate;
// //
// //            // Update last error for next derivative calculation
// //            lastXError = xError;
// //            lastYError = yError;
// //
// //            // Assign corrections to motors for field-centric movement
// //            double frontLeftPower = yCorrection + xCorrection;
// //            double frontRightPower = yCorrection - xCorrection;
// //            double backLeftPower = yCorrection - xCorrection;
// //            double backRightPower = yCorrection + xCorrection;
// //
// //            // Normalize powers to ensure values are within [-1, 1]
// //            double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
// //                    Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));
// //
// //            frontLeftPower /= maxPower;
// //            frontRightPower /= maxPower;
// //            backLeftPower /= maxPower;
// //            backRightPower /= maxPower;
// //
// //            // Set motor powers
// //            frontLeftMotor.setPower(frontLeftPower);
// //            frontRightMotor.setPower(frontRightPower);
// //            backLeftMotor.setPower(backLeftPower);
// //            backRightMotor.setPower(backRightPower);
// //
// //            telemetry.addData("Target X", targetX);
// //            telemetry.addData("Target Y", targetY);
// //            telemetry.addData("Current X", avgx);
// //            telemetry.addData("Current Y", avgy);
// //            telemetry.addData("X Error", xError);
// //            telemetry.addData("Y Error", yError);
// //            telemetry.addData("X Correction", xCorrection);
// //            telemetry.addData("Y Correction", yCorrection);
// //            telemetry.update();
//     }
// }