//package org.firstinspires.ftc.teamcode;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.lynx.LynxNackException;
//import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
//import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@TeleOp(name="FullIntake+fieldCetric",group = "newnewnew")
//public class FULLINTAKE2024 extends OpMode {
//
//    //wheelbase
//    private DcMotor backRightMotor;
//    private DcMotor backLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor frontLeftMotor;
//    private IMU imu;
//
//    //intake
//    private DcMotor angleMotor;
//    private DcMotor slideMotor;
//    private CRServo R_Roll;
//    private CRServo L_Roll;
//
//    //datalog
//    private LynxModule myRevHub;
//    private LynxGetADCCommand.Channel servoChannel;
//    private LynxGetADCCommand servoCommand;
//    private LynxGetADCResponse servoResponse;
//
//    //datalog and optimization
//    double angleBrakePower;
//    double slideBrakePower = 0;
//    double power=0.75;
//    String mode = "Field";
//    boolean arm = true;
//
//
//    @Override
//    public void init() {
//
//        //wheelbase
//         frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//         backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//         frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//         backRightMotor = hardwareMap.dcMotor.get("rightBack");
//
//         //intake
//        angleMotor = hardwareMap.get(DcMotor.class, "AngleMotor");
//        slideMotor = hardwareMap.get(DcMotor.class, "SlideMotor");
//        R_Roll = hardwareMap.get(CRServo.class, "R_Roll");
//        L_Roll = hardwareMap.get(CRServo.class, "L_Roll");
//        myRevHub = hardwareMap.get(LynxModule.class, "Control Hub");
//
//        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Retrieve the IMU from the hardware map
//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//    }
//
//
//        @Override
//        public void loop() {
//            //WHEELBASE  FIELD
//            if(arm) {
//                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
//                double x = gamepad2.left_stick_x;
//                double rx = gamepad2.right_stick_x;
//                if (gamepad2.start) {
//                    imu.resetYaw();
//                }
//                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//                rotX = rotX * 1.1280827;  // Counteract imperfect strafing
//                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//                double frontLeftPower = (rotY + rotX + rx) / denominator;
//                double backLeftPower = (rotY - rotX + rx) / denominator;
//                double frontRightPower = (rotY - rotX - rx) / denominator;
//                double backRightPower = (rotY + rotX - rx) / denominator;
//                frontLeftMotor.setPower(frontLeftPower);
//                backLeftMotor.setPower(backLeftPower);
//                frontRightMotor.setPower(frontRightPower);
//                backRightMotor.setPower(backRightPower);
//            }
//            else {
//                float vertical;
//                float horizontal;
//                float pivot;
//
//                vertical = -gamepad1.left_stick_y;
//                horizontal = gamepad1.left_stick_x;
//
//                if (gamepad1.left_trigger>0)
//                {
//                    pivot = -gamepad1.left_trigger;
//                }
//                else if (gamepad1.right_trigger>0)
//                {
//                    pivot = +gamepad1.right_trigger;
//                }
//                else
//                {
//                    pivot = 0;
//                }
//
//                frontRightMotor.setPower ((-pivot + (vertical - horizontal)) * power);
//                backRightMotor.setPower ((-pivot + vertical + horizontal) * power);
//                frontLeftMotor.setPower ((pivot + vertical + horizontal) * power);
//                backLeftMotor.setPower ((pivot + (vertical - horizontal)) * power);
//            }
//
//            //MODE SWITCHING
//            if(gamepad1.dpad_up){
//                arm = false;
//                mode = "Tank";
//            }
//            if(gamepad1.dpad_down){
//                arm=true;
//                mode="Field";
//            }
//            if (gamepad1.left_bumper)
//            {
//                power = 0.3;
//            }
//            if (gamepad1.right_bumper)
//            {
//                power = 0.75;
//            }
//
//            //INTAKE
//
//
//            //Update info
//            telemetry.addData("Mode", mode);
//            telemetry.addData("Slide Motor Enc", slideMotor.getCurrentPosition());
//            telemetry.addData("Angle Motor Enc", angleMotor.getCurrentPosition());
//            telemetry.addData("Angle Motor Brake Power", angleBrakePower);
//            telemetry.addData("Wheelbase Power", power);
//            telemetry.update();
//
//            //TELEOP OPTIMIZATION
//
//            //linear braking
//            angleBrakePower = 0.00005 * slideMotor.getCurrentPosition();
//
//            //DATALOGGING
////            if (getServoBusCurrent() > 4.5) {
////                Roll.setPower(0);
////                currentThreshold = true;
////            }
//
//            //INTAKE
//            if(gamepad1.a|| gamepad2.a)
//            {
//                R_Roll.setPower(1);
//                L_Roll.setPower(-1);
//            }
//            else if(gamepad1.b|| gamepad2.b)
//            {
//                R_Roll.setPower(-1);
//                L_Roll.setPower(1);
//            }
//            else
//            {
//                R_Roll.setPower(0);
//                L_Roll.setPower(0);
//            }
//
//
//            //ANGLE MOTOR CONTROLS
//            if (gamepad2.dpad_up)
//            {
//                angleMotor.setPower(-0.5);
//            }
//            else if (gamepad2.dpad_down)
//            {
//                angleMotor.setPower(0.5);
//            }
//            else
//            {
//                angleMotor.setPower(angleBrakePower);
//            }
//
//            //SLIDE MOTOR CONTROLS
//            if (gamepad2.right_bumper)
//            {
//                slideMotor.setPower(-0.6);
//            }
//            else if (gamepad2.left_bumper)
//            {
//                slideMotor.setPower(0.4);
//            }
//            else
//            {
//                slideMotor.setPower(slideBrakePower);
//            }
//
//            //SLIDE MOTOR BRAKING
//            if (gamepad2.right_stick_button)
//            {
//                slideBrakePower = -0.2;
//            }
//            if (gamepad2.left_stick_button)
//            {
//                slideBrakePower = 0;
//            }
//
//        //sleep
////    void sona(int time) {
////        try {
////            Thread.sleep(time);
////        } catch (InterruptedException e) {
////            Thread.currentThread().interrupt();
////        }
////    }
//    //CURRENT FUNCTION
////    double getServoBusCurrent()
////    {
////        servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
////        servoCommand = new LynxGetADCCommand(myRevHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
////        try
////        {
////            servoResponse = servoCommand.sendReceive();
////            return servoResponse.getValue() / 1000.0;    // return value in Amps
////        }
////        catch (InterruptedException | RuntimeException | LynxNackException e)
////        {
////        }
////        return 999;
////    }
//}
//}
//
