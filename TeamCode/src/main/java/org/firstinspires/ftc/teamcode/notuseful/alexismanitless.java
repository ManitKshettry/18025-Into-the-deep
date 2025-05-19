//package org.firstinspires.ftc.teamcode;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
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
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//
//@TeleOp(name="servodatalog",group = "Final")
//public class alexismanitless extends OpMode
//{
//    //Wheelbase
//    private DcMotor BR;
//    private DcMotor BL;
//    private DcMotor FR;
//    private DcMotor FL;
//
//    //Intake
//    private DcMotor angleMotor;
//    private DcMotor slideMotor;
//    private CRServo R_Roll;
//    private CRServo L_Roll;
//
//    //Servo current monitoring
//    private LynxModule myRevHub;
//    private LynxGetADCCommand.Channel servoChannel;
//    private LynxGetADCCommand servoCommand;
//    private LynxGetADCResponse servoResponse;
//    boolean currentThreshold = false;
//
//    //Graphing
//    private FtcDashboard dashboard;
//
//    //Optimization
//    double angleBrakePower;
//    double slideBrakePower = 0;
//    double power_factor = 0.7;
//
//    @Override
//    public void init()
//    {
//        //Graphing
//        dashboard = FtcDashboard.getInstance();
//
//        //Wheelbase
//        FL = hardwareMap.dcMotor.get("leftFront");
//        BL = hardwareMap.dcMotor.get("leftBack");
//        FR = hardwareMap.dcMotor.get("rightFront");
//        BR = hardwareMap.dcMotor.get("rightBack");
//
//
//
//        //Intake
//        angleMotor = hardwareMap.get(DcMotor.class, "AM");
//        slideMotor = hardwareMap.get(DcMotor.class, "SM");
//        R_Roll = hardwareMap.get(CRServo.class, "R_Roll");
//        L_Roll = hardwareMap.get(CRServo.class, "L_Roll");
//        myRevHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
//
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        FL.setDirection(DcMotorSimple.Direction.REVERSE);
//        BL.setDirection(DcMotorSimple.Direction.REVERSE);
//    }
//
//
//    @Override
//    public void loop()
//    {
//        float vertical;
//        float horizontal;
//        float pivot;
//
//        vertical = -gamepad1.left_stick_y;
//        horizontal = gamepad1.left_stick_x;
//
//        if (gamepad1.left_trigger>0)
//        {
//            pivot = -gamepad1.left_trigger;
//        }
//        else if (gamepad1.right_trigger>0)
//        {
//            pivot = +gamepad1.right_trigger;
//        }
//        else
//        {
//            pivot = 0;
//        }
//
//        FR.setPower ((-pivot + (vertical - horizontal)) * power_factor);
//        BR.setPower ((-pivot + vertical + horizontal) * power_factor);
//        FL.setPower ((pivot + vertical + horizontal) * power_factor);
//        BL.setPower ((pivot + (vertical - horizontal)) * power_factor);
//
//        if (gamepad1.left_bumper)
//        {
//            power_factor = 0.3;
//        }
//        if (gamepad1.right_bumper)
//        {
//            power_factor = 0.7;
//        }
//
//        //Update info
//        telemetry.addData("Current Threshold", currentThreshold);
//        telemetry.addData("Servo Bus Current", getServoBusCurrent());
//        telemetry.addData("Slide Motor Enc", slideMotor.getCurrentPosition());
//        telemetry.addData("Angle Motor Enc", angleMotor.getCurrentPosition());
//        telemetry.addData("Angle Motor Brake Power", angleBrakePower);
//        telemetry.addData("Wheelbase Power", power_factor);
//        telemetry.update();
//
//        //Graphing on FTC Dashboard
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Servo Bus Current", getServoBusCurrent());
//        dashboard.sendTelemetryPacket(packet);
//
//        //Linear braking
//        if (angleMotor.getCurrentPosition() > -2700)
//        {
//            angleBrakePower = 0.00005 * slideMotor.getCurrentPosition();
//        }
//        else
//        {
//            angleBrakePower = 0;
//        }
//
//        //ANGLE MOTOR CONTROLS
//        if (gamepad2.dpad_up)
//        {
//            angleMotor.setPower(-0.5);
//        }
//        else if (gamepad2.dpad_down)
//        {
//            angleMotor.setPower(0.5);
//        }
//        else
//        {
//            angleMotor.setPower(angleBrakePower);
//        }
//
//        //SLIDE MOTOR CONTROLS
//        if (gamepad2.right_bumper)
//        {
//            slideMotor.setPower(-0.6);
//        }
//        else if (gamepad2.left_bumper)
//        {
//            slideMotor.setPower(0.4);
//        }
//        else
//        {
//            slideMotor.setPower(slideBrakePower);
//        }
//
//        //SLIDE MOTOR BRAKING
//        if (gamepad2.right_stick_button)
//        {
//            slideBrakePower = -0.2;
//        }
//        if (gamepad2.left_stick_button)
//        {
//            slideBrakePower = 0;
//        }
//
//        //ROLLER CONTROLS
//        if(gamepad1.a)
//        {
//            R_Roll.setPower(1);
//            L_Roll.setPower(-1);
//        }
//        else if(gamepad1.b)
//        {
//            R_Roll.setPower(-1);
//            L_Roll.setPower(1);
//        }
//        else
//        {
//            R_Roll.setPower(0);
//            L_Roll.setPower(0);
//        }
//    }
//
//    //sleep
//    // void sona(int time) {
//    //     try {
//    //         Thread.sleep(time);
//    //     } catch (InterruptedException e) {
//    //         Thread.currentThread().interrupt();
//    //     }
//    // }
//
//    //CURRENT FUNCTION
//    double getServoBusCurrent()
//    {
//        servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
//        servoCommand = new LynxGetADCCommand(myRevHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
//        try
//        {
//            servoResponse = servoCommand.sendReceive();
//            return servoResponse.getValue() / 1000.0;    // return value in Amps
//        }
//        catch (InterruptedException | RuntimeException | LynxNackException e)
//        {
//        }
//        return 999;
//    }
//}
//
