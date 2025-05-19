package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="TELEOP_THE_MANITLESS",group = "test")
public class FINAL_TELEOP extends OpMode
{
    //Wheelbase
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor FL;
    private IMU imu;

    //Intake and Delivery
    private DcMotor angleMotor;
    private DcMotor slideMotor;
    private CRServo R_Roll;
    private CRServo L_Roll;
    private Servo Pushy;
    private Servo Claw;
    private Servo Adj;

    //Hanging
    private DcMotor RPull;
    private DcMotor LPull;
    private Servo L_Hanger;
    private Servo R_Hanger;

    //Servo Current Monitoring
    private LynxModule myRevHub;
    private LynxGetADCCommand.Channel servoChannel;
    private LynxGetADCCommand servoCommand;
    private LynxGetADCResponse servoResponse;

    //Optimization
    double angleBrakePower;
    double slideBrakePower = 0;
    double slidePower = 1;
    double power_factor = 0.907;
    boolean currentThreshold = false;
    boolean arm = true;
    boolean winderToggle = false;
    String driveMode = "Field";

    @Override
    public void init()
    {
        //Wheelbase
        FL = hardwareMap.dcMotor.get("leftFront");
        BL = hardwareMap.dcMotor.get("leftBack");
        FR = hardwareMap.dcMotor.get("rightFront");
        BR = hardwareMap.dcMotor.get("rightBack");

        //Intake and Delivery
        angleMotor = hardwareMap.get(DcMotor.class, "AM");
        slideMotor = hardwareMap.get(DcMotor.class, "SM");
        LPull = hardwareMap.get(DcMotor.class, "L_Pull");
        RPull = hardwareMap.get(DcMotor.class, "R_Pull");
        R_Roll = hardwareMap.get(CRServo.class, "R_Roll");
        // L_Roll = hardwareMap.get(CRServo.class, "L_Roll");
        Pushy = hardwareMap.get(Servo.class, "push");
        Claw = hardwareMap.get(Servo.class, "Claw");

        //Hanging
        // Back_Wind = hardwareMap.get(DcMotor.class, "Back_Wind");
        // Front_Wind = hardwareMap.get(DcMotor.class, "Front_Wind");
        L_Hanger = hardwareMap.get(Servo.class, "L_Hanger");
        R_Hanger = hardwareMap.get(Servo.class, "R_Hanger");
        Adj = hardwareMap.get(Servo.class, "Adj");

        //Servo Current Monitoring
        myRevHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //DC Motor Braking
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RPull.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LPull.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);//192.168.43.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/FINAL_TELEOP.javairection.REVERSE);
        Adj.setPosition(0.15);

        //Setting all motors to start position
        angleMotor.setPower(-0.5);
        slideMotor.setPower(1);
        sona(1000);
        angleMotor.setPower(1);
        sona(1500);

        //Initializing Servos
        Pushy.setPosition(1);

        L_Hanger.setPosition(0.97);
        R_Hanger.setPosition(0.15);

        //Disabling Encoders
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");


        //Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters
                (
                        new RevHubOrientationOnRobot
                                (
                                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                                )
                );

        //Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    @Override
    public void init_loop(){}

    @Override
    public void loop()
    {
        //Field Centric Drive
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.start)
        {
            imu.resetYaw();
        }

        //Resetting encoders just in case
        if(gamepad1.a && gamepad1.b && gamepad1.x)
        {
            angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1078;  // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double FL_Power = ((rotY + rotX + rx) / denominator)*power_factor;
        double BL_Power = ((rotY - rotX + rx) / denominator)*power_factor;
        double FR_Power = ((rotY - rotX - rx) / denominator)*power_factor;
        double BR_Power = ((rotY + rotX - rx) / denominator)*power_factor;

        FL.setPower(FL_Power);
        BL.setPower(BL_Power);
        FR.setPower(FR_Power);
        BR.setPower(BR_Power);

        //Slow Mode
        if (gamepad1.left_bumper)
        {
            power_factor = 0.3;
        }
        if (gamepad1.right_bumper)
        {
            power_factor = 0.907;
        }

        //Slide Slow Motor
        // if (gamepad2.x)
        //     slidePower = 0.8;
        // if (gamepad2.y)
        //     slidePower = 1;

        //Update Info
        // telemetry.addData("Drive Mode", driveMode);
        telemetry.addData("Slide Motor Enc", slideMotor.getCurrentPosition());
        telemetry.addData("Angle Motor Enc", angleMotor.getCurrentPosition());
        telemetry.addData("Angle Motor Brake Power", angleBrakePower);
        telemetry.addData("Claw Position", Claw.getPosition());
        telemetry.addData("Adj Position", Adj.getPosition());
        telemetry.addData("Wheelbase Power", power_factor);
        telemetry.addData("Bot Heading",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.update();

        //Linear Braking
        if (angleMotor.getCurrentPosition() > -1900)
            angleBrakePower = -(0.0001 * slideMotor.getCurrentPosition());
        else if (angleMotor.getCurrentPosition() < -2200)
            angleBrakePower = 0.0001 * slideMotor.getCurrentPosition();
        else
            angleBrakePower = 0;

        //Claw Intake
        if (gamepad2.a || gamepad1.a)
        {
            Claw.setPosition(0.1);
        }
        if (gamepad2.b || gamepad1.b)
        {
            Claw.setPosition(0.47);
        }

        // //Gecko Wheel Intake
        // if(gamepad1.a|| gamepad2.a)
        // {
        //     R_Roll.setPower(1);
        //     L_Roll.setPower(-1);
        // }
        // else if(gamepad1.b|| gamepad2.b)
        // {
        //     R_Roll.setPower(-1);
        //     L_Roll.setPower(1);
        // }
        // else
        // {
        //     R_Roll.setPower(0);
        //     L_Roll.setPower(0);
        // }

        //Angle Motor
        if (gamepad2.right_trigger > 0.05)
        {
            angleMotor.setPower(-gamepad2.right_trigger/1.75);
        }
        else if (gamepad2.left_trigger > 0.05)
        {
            angleMotor.setPower(gamepad2.left_trigger/1.75);
        }
        else
        {
            angleMotor.setPower(-angleBrakePower);
        }

        //Slide Motor
        if (slideMotor.getCurrentPosition() < -3100)
        {
            if (gamepad2.left_stick_y > 0.1)
            {
                slideMotor.setPower(gamepad2.left_stick_y * slidePower);
            }
            else
            {
                slideMotor.setPower(slideBrakePower);
            }
        }
        else if (slideMotor.getCurrentPosition() > -10)
        {
            if (gamepad2.left_stick_y < -0.1)
            {
                slideMotor.setPower(gamepad2.left_stick_y * slidePower);
            }
            else
            {
                slideMotor.setPower(slideBrakePower);
            }
        }
        else
        {
            if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1)
            {
                slideMotor.setPower(gamepad2.left_stick_y * slidePower);
            }
            else
            {
                slideMotor.setPower(slideBrakePower);
            }
        }

        //Slide Motor Braking
        if (gamepad2.right_stick_button)
        {
            slideBrakePower = -0.2;
        }
        if (gamepad2.left_stick_button)
        {
            slideBrakePower = 0;
        }

        //Hanger Servos
        // if (gamepad2.dpad_up)
        // {
        //     L_Hanger.setPosition(0.5);
        //     R_Hanger.setPosition(0.5);
        // }
        // if (gamepad2.dpad_down)
        // {
        //     L_Hanger.setPosition(0.97);
        //     R_Hanger.setPosition(0.15);
        // }


        //Winder Motors
        if (gamepad2.right_bumper)
        {
            Adj.setPosition(1);
        }
        if (gamepad2.left_bumper)
        {
            // Adj.setPosition(Adj.getPosition()-0.1);
            Adj.setPosition(0.35);
        }
        if (gamepad2.left_bumper&&gamepad2.right_bumper)
        {
            Adj.setPosition(0.7);
        }
        if (gamepad1.x&&gamepad1.y)
        {
            arm=false;
            LPull.setPower(-0.5);
            RPull.setPower(0.5);
        }



        // if (gamepad1.right_trigger > 0.2)
        // {
        //     Back_Wind.setPower(1);
        // }
        // else if (gamepad1.left_trigger > 0.2)
        // {
        //     Back_Wind.setPower(-1);
        // }
        // else
        // {
        //     Back_Wind.setPower(0);
        // }

        if (gamepad2.dpad_up)
        {
            RPull.setPower(-1);
            LPull.setPower(1);
        }
        else if (gamepad2.dpad_down)
        {
            RPull.setPower(1);
            LPull.setPower(-1);

        }
        else if(arm){LPull.setPower(0);
            RPull.setPower(-0);}
        // if (gamepad2.left_bumper&&gamepad2.right_bumper)
        // {
        //     Adj.setPosition(0.7);
        // }
    }

    //DELAY FUNCTION
    void sona(int time)
    {
        try
        {
            Thread.sleep(time);
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
    }

    // //CURRENT FUNCTION
    // double getServoBusCurrent()
    // {
    //     servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
    //     servoCommand = new LynxGetADCCommand(myRevHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
    //     try
    //     {
    //         servoResponse = servoCommand.sendReceive();
    //         return servoResponse.getValue() / 1000.0;    // return value in Amps
    //     }
    //     catch (InterruptedException | RuntimeException | LynxNackException e)
    //     {
    //     }
    //     return 999;
    // }
}

