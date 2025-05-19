//package org.firstinspires.ftc.teamcode.notuseful;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.*;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.opencv.core.Mat;
//import org.opencv.imgproc.Imgproc;
//import org.tensorflow.lite.Interpreter;
//import org.tensorflow.lite.support.common.FileUtil;
//import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;
//
//import java.nio.ByteBuffer;
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp(name = "ML + EasyOpenCV (LiteRT Correct Syntax)")
//public class MLEasyOpenCVLiteRT extends LinearOpMode {
//    OpenCvCamera webcam;
//    Interpreter tfliteInterpreter;
//
//    @Override
//    public void runOpMode() {
//        // Initialize OpenCV Camera
//        int cameraMonitorViewId = hardwareMap.appContext.getResources()
//                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        // Set OpenCV processing pipeline
//        webcam.setPipeline(new SamplePipeline());
//
//        // Load the TensorFlow Lite model
//        try {
//            tfliteInterpreter = new Interpreter(FileUtil.loadMappedFile(hardwareMap.appContext, "model.tflite"));
//        } catch (Exception e) {
//            telemetry.addData("Error loading model", e.getMessage());
//            telemetry.update();
//            return;
//        }
//
//        // Open the camera and start streaming
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error", errorCode);
//            }
//        });
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Get the current frame from OpenCV
//            Mat frame = webcam.();
//
//            // Prepare the frame data to be passed to the model (convert to byte buffer)
//            ByteBuffer byteBuffer = convertMatToByteBuffer(frame);
//
//            // Run inference using LiteRT
//            TensorBuffer outputBuffer = TensorBuffer.createFixedSize(new int[]{1, 10}, org.tensorflow.lite.DataType.FLOAT32);  // Change dimensions based on model
//            tfliteInterpreter.run(byteBuffer, outputBuffer.getBuffer());
//
//            // Process the model's output
//            List<Float> results = processInferenceOutput(outputBuffer);
//
//            // Display the results
//            telemetry.addData("Model Results", results.toString());
//            telemetry.update();
//        }
//    }
//
//    // Convert OpenCV Mat to ByteBuffer
//    private ByteBuffer convertMatToByteBuffer(Mat mat) {
//        int size = mat.rows() * mat.cols() * mat.channels();
//        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(size);
//        byteBuffer.rewind();
//        mat.get(0, 0, byteBuffer.array());
//        return byteBuffer;
//    }
//
//    // Process the inference output (example: get top prediction)
//    private List<Float> processInferenceOutput(TensorBuffer outputBuffer) {
//        // Process the output buffer as needed
//        // For simplicity, returning the raw output as a list of floats
//        float[] resultArray = outputBuffer.getFloatArray();
//        List<Float> results = new ArrayList<>();
//        for (float res : resultArray) {
//            results.add(res);
//        }
//        return results;
//    }
//
//    // Custom OpenCV Processing Pipeline
//    static class SamplePipeline extends OpenCvPipeline {
//        @Override
//        public Mat processFrame(Mat input) {
//            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);  // Example preprocessing
//            return input;
//        }
//    }
//}
