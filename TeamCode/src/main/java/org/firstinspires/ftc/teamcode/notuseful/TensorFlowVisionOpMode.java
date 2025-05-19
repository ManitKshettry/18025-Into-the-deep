//package org.firstinspires.ftc.teamcode.notuseful;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.tensorflow.lite.Interpreter;
//import org.tensorflow.lite.support.common.FileUtil;
//import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;
//import org.opencv.core.Mat;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import java.nio.ByteBuffer;
//import java.io.IOException;
//@Autonomous(name = "blah",group = "newnew")
//public class TensorFlowVisionOpMode extends LinearOpMode {
//
//    private OpenCvCamera webcam;
//    private Interpreter interpreter;
//    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize motors
//        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        backRightMotor = hardwareMap.dcMotor.get("rightBack");
//
//        // Initialize TensorFlow Lite model
//        try {
//            interpreter = new Interpreter(FileUtil.loadMappedFile(hardwareMap.appContext, "asd.tflite"));
//        } catch (IOException e) {
//            telemetry.addData("Error", "Failed to load model: " + e.getMessage());
//            telemetry.update();
//            return;
//        }
//
//        // Initialize webcam
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        webcam.setPipeline(new OpenCvPipeline() {
//            @Override
//            public Mat processFrame(Mat input) {
//                // Preprocess the frame (resize, convert to RGB)
//                Mat processedFrame = preprocessImage(input);
//
//                // Run TensorFlow Lite inference
//                TensorBuffer outputBuffer = TensorBuffer.createFixedSize(new int[]{1, 10}, org.tensorflow.lite.DataType.FLOAT32); // Adjust size based on your model
//                interpreter.run(getByteBufferFromMat(processedFrame), outputBuffer.getBuffer());
//
//                // Process the inference results (example)
//                processInferenceResults(outputBuffer);
//
//                // Return the frame (for visualization, optional)
//                return processedFrame;
//            }
//        });
//
//        // Open the webcam
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error", errorCode);
//                telemetry.update();
//            }
//        });
//
//        waitForStart();
//
//        // Main loop
//        while (opModeIsActive()) {
//            // Add any other robot logic here if necessary
//            telemetry.update();
//        }
//    }
//
//    // Method to preprocess the image before passing to TensorFlow Lite
//    private Mat preprocessImage(Mat input) {
//        // Resize image to the size expected by the model (e.g., 224x224)
//        Size targetSize = new Size(224, 224); // Adjust according to your model's input size
//        Imgproc.resize(input, input, targetSize);
//
//        // Convert the image to RGB (TensorFlow Lite models expect RGB format)
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2RGB);
//
//        return input;
//    }
//
//    // Convert OpenCV Mat to ByteBuffer (required for TensorFlow Lite input)
//    private ByteBuffer getByteBufferFromMat(Mat mat) {
//        int bufferSize = mat.rows() * mat.cols() * mat.channels();
//        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(bufferSize);
//        byteBuffer.rewind();
//        mat.get(0, 0, byteBuffer.array());
//        return byteBuffer;
//    }
//
//    // Process the TensorFlow Lite output (example processing)
//    private void processInferenceResults(TensorBuffer outputBuffer) {
//        float[] output = outputBuffer.getFloatArray();
//
//        // Example: Print the model's output to telemetry
//        telemetry.addData("Inference Results", output);
//
//        // Add your logic here to process the results, for example:
//        // if (output[0] > 0.5) {
//        //     // Perform some robot action
//        // }
//
//        telemetry.update();
//    }
//}
