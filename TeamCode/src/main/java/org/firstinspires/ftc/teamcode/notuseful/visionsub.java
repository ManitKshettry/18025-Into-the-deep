//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.opencv.core.Point;
//import org.opencv.core.RotatedRect;
//import org.opencv.core.Size;
//
//import com.arcrobotics.ftclib.vision.ColorBlobLocatorProcessor;
//import com.arcrobotics.ftclib.vision.VisionPortal;
//
//@TeleOp(name = "VisionPortal Example", group = "Examples")
//public class visionsub extends LinearOpMode {
//
//    private VisionPortal visionPortal;
//    private ColorBlobLocatorProcessor colorLocator;
//
//    @Override
//    public void runOpMode() {
//        // Initialize VisionPortal and Processor
//        initializeVisionPortal();
//
//        telemetry.addLine("Ready to start");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            processVision();
//        }
//
//        // Close VisionPortal when done
//        if (visionPortal != null) {
//            visionPortal.close();
//        }
//    }
//
//    private void initializeVisionPortal() {
//        // Create ColorBlobLocatorProcessor for blob detection
//        colorLocator = new ColorBlobLocatorProcessor();
//
//        // Build VisionPortal with correct camera resolution (using org.opencv.core.Size)
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(colorLocator)
//                .setCameraResolution(new Size(320, 240)) // Use OpenCV Size
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();
//    }
//
//    private void processVision() {
//        // Retrieve blobs from the processor
//        var blobs = colorLocator.getBlobs();
//
//        if (!blobs.isEmpty()) {
//            // Process the first blob
//            ColorBlobLocatorProcessor.Blob blob = blobs.get(0);
//            RotatedRect boxFit = blob.getBoxFit();
//            Point[] points = new Point[4];
//            boxFit.points(points);
//
//            // Display information on telemetry
//            telemetry.addData("Blob Center X", boxFit.center.x);
//            telemetry.addData("Blob Center Y", boxFit.center.y);
//            telemetry.addData("Blob Width", boxFit.size.width);
//            telemetry.addData("Blob Height", boxFit.size.height);
//            telemetry.addLine("Blob Detected!");
//        } else {
//            telemetry.addLine("No blobs detected.");
//        }
//
//        telemetry.update();
//    }
//}
