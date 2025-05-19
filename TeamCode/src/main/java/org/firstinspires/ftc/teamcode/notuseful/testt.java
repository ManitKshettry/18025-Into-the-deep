//package org.firstinspires.ftc.teamcode.notuseful;
//
//import android.annotation.SuppressLint;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous(name = "test°", group = "Final")
//public class testt extends LinearOpMode
//{
//    double x,y;
//    MecanumDrive drive;
//    Servo push;
//    static Servo claw;
//
//    //Intake Motors
//    private static DcMotorEx angleMotor;
//    private static DcMotorEx slideMotor;
//
////    private DcMotorEx boreThroughEnc;
//
//    //Wheelbase Motors
//    private DcMotor backRightMotor;
//    private DcMotor backLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor frontLeftMotor;
//
//    //Actions
//    Action scoreprelload;
//    Action strafespike;
//    Action obzone1;
//    Action obzone1back;
//    Action obzone2;
//    Action obzone2back;
//    Action obzone3;
//    Action pickup1;
//    Action test,test2,test3;
//    Action scoreee;
//    Action scoreee1;
//    Servo Claw;
//    //Linear Braking
//    double angleBrakePower = 0;
//    double slideBrakePower = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        drive = new MecanumDrive(hardwareMap, new Pose2d(-70.35, 0, 0)); //Initialize in init()
//
//        Pose2d pick = new Pose2d(13.20-70.35, -7.3,-129.3);
//        Pose2d deliver = new Pose2d(14.68-70.350, -1.4+1.5, Math.PI);
//        Pose2d deliver1 = new Pose2d(14.68-70.350, -1.4+2.5+3, Math.PI);
//        Pose2d deliver2 = new Pose2d(14.68-70.350, -1.4+2.5+6, Math.PI);
//
//        angleMotor = hardwareMap.get(DcMotorEx.class, "AM");
////        double slideBrakePower = 0;
//        Claw = hardwareMap.get(Servo.class, "Claw");
//        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        backRightMotor = hardwareMap.dcMotor.get("rightBack");
//        slideMotor = hardwareMap.get(DcMotorEx.class, "SM");
////        boreThroughEnc = hardwareMap.get(DcMotorEx.class, "Back_Wind");
//        push = hardwareMap.get(Servo.class, "push");
//        claw = hardwareMap.get(Servo.class, "Claw");
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        boreThroughEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        boreThroughEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        angleMotor.setTargetPositionTolerance(1);
//        claw.setPosition(-0);
//        slideMotor.setTargetPositionTolerance(1);
//
//        scoreprelload = drive.actionBuilder(drive.pose)
//                .setTangent(0)
//                .strafeToConstantHeading(new Vector2d(12 - 70.35, 0))
//                .build();
//
//        strafespike = drive.actionBuilder(new Pose2d(12 - 70.35, 0, 0))
//                .setTangent(0)
//                .stopAndAdd(new pushy(push))
//                .strafeToConstantHeading(new Vector2d(28 + 4 + 2 + 1 + 2 - 70.35, -30.1))
//                .build();
//
//        obzone1 = drive.actionBuilder(new Pose2d(28 + 4 + 2 + 1 + 2 - 70.35, -30.1, 0))
//                .setTangent(0)
////                .waitSeconds(0.54444444476476764767)
//                .strafeTo(new Vector2d(3 - 70.35, -34))
//                .build();
//
//        obzone1back = drive.actionBuilder(new Pose2d(3 - 70.35, -34, 0))
//                .setTangent(0)
////                .waitSeconds(0.1)
//                .stopAndAdd(new pushyup(push))
//                .strafeTo(new Vector2d(30 + 6 + 2 + 1 + 3 + 2 - 70.35, -40))
//                .build();
//        obzone2 = drive.actionBuilder(new Pose2d(30 + 6 + 3 + 2 + 2 + 1 - 70.35, -40, 0))
//                .setTangent(0)
//
////                .waitSeconds(0.4)
//                .lineToX(3 - 70.35)
//                .build();
//        obzone2back = drive.actionBuilder(new Pose2d(3 - 70.35, -40, 0))
//                .setTangent(0)
////                .waitSeconds(0.4)
//                .strafeTo(new Vector2d(30 + 6 + 2 + 1 + 3 - 70.35, -40 - 9.5))
////                .waitSeconds(0.1)
//                .build();
//        obzone3 = drive.actionBuilder(new Pose2d(30 + 6 + 2 + 3 + 1 - 70.35, -50.5, 0))
//                .setTangent(0)
//                .lineToX(3 - 70.35)
//                .build();
//        pickup1 = drive.actionBuilder(new Pose2d(3 - 70.35, -50.5, 0))
//                .setTangent(0)
////                .waitSeconds(0.1)
//                .strafeToLinearHeading(new Vector2d(-62.55, -36.5), Math.PI)
//                .build();
//
////        testy = drive.actionBuilder(new Pose2d(-70.35, 0, 0))
////                .setTangent(0)
//////                .waitSeconds(0.1)
////                .strafeToLinearHeading(new Vector2d(-62.55, -36.5), Math.PI)
////                .build();
//
////        scoreee = drive.actionBuilder(new Pose2d(new Vector2d(-62.55, -36.5), Math.PI))
////                .setTangent(0)
////                .stopAndAdd(new clawy(claw))
////                .strafeToLinearHeading(deliver180.component1(), Math.toRadians(0))
////                .build();
//        test = drive.actionBuilder(new Pose2d(new Vector2d(3 - 70.35, -50.5), 0))
//                .waitSeconds(15)
//                .setTangent(0)
//                .strafeToLinearHeading(pick.component1(), -Math.toRadians(129.3))
////                .waitSeconds(0.5)
////                .stopAndAdd(new pick ())
////                .waitSeconds(0.2)
////                .stopAndAdd(new clawose(claw))
////                .waitSeconds(1)
//                .waitSeconds(15)
//                .setTangent(0)
//                .strafeToLinearHeading(deliver.component1(),Math.PI)
////                .waitSeconds(1)
////                .stopAndAdd(new deliver())
////                .waitSeconds(2)
//
//
//                .build();
//        test2 = drive.actionBuilder(deliver)
//                .waitSeconds(15)
//                .setTangent(0)
//
//                .strafeToLinearHeading(pick.component1(), -Math.toRadians(129.3))
////                .waitSeconds(0.1)
////                .stopAndAdd(new pick1 ())
////                .waitSeconds(0.1)
////                .stopAndAdd(new clawose(claw))
////                .waitSeconds(1)
//                .waitSeconds(15)
//                .setTangent(0)
//                .strafeToLinearHeading(deliver1.component1(),Math.PI)
////                .waitSeconds(1)
////                .stopAndAdd(new deliver())
////                .waitSeconds(2)
//
//
//                .build();
//        test3 = drive.actionBuilder(deliver1)
//                .waitSeconds(15)
//                .setTangent(0)
////                .waitSeconds(15)
//                .strafeToLinearHeading(pick.component1(), -Math.toRadians(129.3))
////                .waitSeconds(0.1)
////                .stopAndAdd(new pick1 ())
////                .waitSeconds(0.1)
////                .stopAndAdd(new clawose(claw))
////                .waitSeconds(1)
//                .waitSeconds(15)
//                .setTangent(0)
////                .waitSeconds(15)
//                .strafeToLinearHeading(deliver2.component1(),Math.PI)
////                .waitSeconds(1)
////                .stopAndAdd(new deliver())
////                .waitSeconds(2)
//
//
//                .build();
//        waitForStart();
//        Actions.runBlocking(
//                new ParallelAction(new SequentialAction(
////                        new ParallelAction(scoreprelload)
//////                                , new preloaddhruv()),new clawose(push),new clawy(push),
////                        strafespike,
////                        obzone1,
////                        obzone1back,new pushy(push),
////                        obzone2,
////                        obzone2back,
////                        obzone3,
//////                        pickup1
//                        test,
//                        test2,
//                        test3
//                ),new bla())
//        );
//    }
//
//
//    public class bla implements Action{
//        public bla (){}
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (angleMotor.getCurrentPosition() > -500)
//                angleBrakePower = -(0.00005 * slideMotor.getCurrentPosition());
//            else if (angleMotor.getCurrentPosition() < -1500)
//                angleBrakePower = 0.00005 * slideMotor.getCurrentPosition();
//            else
//                angleBrakePower = 0;
//
//
//            if (gamepad2.a || gamepad1.a)
//            {
//                Claw.setPosition(0);
//            }
//            if (gamepad2.b || gamepad1.b)
//            {
//                Claw.setPosition(0.6);
//            }
//
//            // //Gecko Wheel Intake
//            // if(gamepad1.a|| gamepad2.a)
//            // {
//            //     R_Roll.setPower(1);
//            //     L_Roll.setPower(-1);
//            // }
//            // else if(gamepad1.b|| gamepad2.b)
//            // {
//            //     R_Roll.setPower(-1);
//            //     L_Roll.setPower(1);
//            // }
//            // else
//            // {
//            //     R_Roll.setPower(0);
//            //     L_Roll.setPower(0);
//            // }
//
//            //Angle Motor
//            if (gamepad2.right_trigger > 0.05)
//            {
//                angleMotor.setPower(-gamepad2.right_trigger/1.5);
//            }
//            else if (gamepad2.left_trigger > 0.05)
//            {
//                angleMotor.setPower(gamepad2.left_trigger/1.5);
//            }
//            else
//            {
//                angleMotor.setPower(-angleBrakePower);
//            }
//
//            //Slide Motor
//            if (slideMotor.getCurrentPosition() < -3100)
//            {
//                if (gamepad2.left_stick_y > 0.1)
//                {
//                    slideMotor.setPower(gamepad2.left_stick_y);
//                }
//                else
//                {
//                    slideMotor.setPower(slideBrakePower);
//                }
//            }
//            else if (slideMotor.getCurrentPosition() > -10)
//            {
//                if (gamepad2.left_stick_y < -0.1)
//                {
//                    slideMotor.setPower(gamepad2.left_stick_y);
//                }
//                else
//                {
//                    slideMotor.setPower(slideBrakePower);
//                }
//            }
//            else
//            {
//                if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1)
//                {
//                    slideMotor.setPower(gamepad2.left_stick_y);
//                }
//                else
//                {
//                    slideMotor.setPower(slideBrakePower);
//                }
//            }
//
//            //Slide Motor Braking
//            if (gamepad2.right_stick_button)
//            {
//                slideBrakePower = -0.2;
//            }
//            if (gamepad2.left_stick_button)
//            {
//                slideBrakePower = 0;
//            }
//            telemetry.addData("angle",angleMotor.getCurrentPosition());
//            telemetry.addData("slide",slideMotor.getCurrentPosition());
////            telemetry.addData("bore",boreThroughEnc.getCurrentPosition());
////        telemetry.addData("slide",slideMotor.getCurrentPosition());
//
//            telemetry.update();
//            return true;
//        }
//    }
//
//    @Config
//
//    public static class pick implements Action{
//        ElapsedTime timer;
//        public static int anglet=725-3
////                ,anglet2=-2817
//                ,slidet=-1518-14-3;
//        //                ,slidet2=-428;
//        double sp=-1,ap=1;
//
//        public pick() {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//
//
//        }
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(timer==  null){timer = new ElapsedTime();}
//
//
//
//            if (timer.seconds()<1) {
//                angleMotor.setTargetPosition(anglet);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(ap-0.2);
//                slideMotor.setTargetPosition(slidet);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(sp);
//            }
//            if (slideMotor.getCurrentPosition()>(slideMotor.getTargetPosition()-5)&&slideMotor.getCurrentPosition()<(slideMotor.getTargetPosition()+5))
//                claw.setPosition(0);
//            if(timer.seconds()>1.2) {
//                slideMotor.setTargetPosition(-800);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(-sp);
//            }
//            if(timer.seconds()>1.2) {
//                angleMotor.setTargetPosition(-1831-50);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(-1);
//
//            }
//            return !(timer.seconds()>1.21);
//        }
//    }
//
//    public static class pick1 implements Action{
//        ElapsedTime timer;
//        public static int anglet=725-2
////                ,anglet2=-2817
//                ,slidet=-1518+1;
//        //                ,slidet2=-428;
//        double sp=-1,ap=1;
//
//        public pick1() {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//
//
//        }
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(timer==  null){timer = new ElapsedTime();}
//
//
//
//            if (timer.seconds()<1.2) {
//                angleMotor.setTargetPosition(anglet);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(ap);
//                slideMotor.setTargetPosition(slidet);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(sp);
//            }
//            if (timer.seconds()>=1)
//                claw.setPosition(0);
//            if(timer.seconds()>1.4) {
//                slideMotor.setTargetPosition(-800);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(-sp);
//            }
//            if(timer.seconds()>1.4) {
//                angleMotor.setTargetPosition(-1831-50);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(-1);
//
//            }
//            return !(timer.seconds()>1.45);
//        }
//    }
//
//
//    @Config
//    public static class deliver implements Action{
//        ElapsedTime timer;
//        public static int angle1=-1750
//                ,slide1=-1540,slide2=-900,slidedrop=-1007;
//        double sp=-1,ap=0.7;
//
//        public deliver() {
//
//        }
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(timer==  null){timer = new ElapsedTime();}
//            if (timer.seconds()<1){
//                slideMotor.setTargetPosition(slide1);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(sp);
////                angleMotor.setPower(ap);
//            }
//
//            if (timer.seconds() >= 1) {
//                angleMotor.setTargetPosition(angle1);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(1);
//            }
//            if (timer.seconds() >= 1.2) {
//                slideMotor.setTargetPosition(slide2);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(1);
//
//            }
//            if(slideMotor.getCurrentPosition()>-1020&&timer.seconds()>=1.1) {
//                claw.setPosition(1);
//
//            }
//
//            return !(claw.getPosition()==1);
//        }
//    }
//    public class pushy implements Action {
//        Servo pushy;
//        ElapsedTime timer;
//
//        public pushy(Servo push) {
//            this.pushy = push;
//
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(timer==  null){timer = new ElapsedTime();}
//            pushy.setPosition(0.07);
//            return !(timer.seconds()>0.1);
//        }
//    }
//    public class pushyup implements Action {
//        Servo pushyup;
//
//        public pushyup(Servo push) {
//            this.pushyup = push;
//
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            pushyup.setPosition(1);
//            return false;
//        }
//    }
//    //claw gappy
//    public class clawy implements Action {
//        Servo clawy;
//        ElapsedTime timer;
//
//        public clawy (Servo claw) {
//            this.clawy = claw;
//
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(timer==  null){timer = new ElapsedTime();}
//            clawy.setPosition(0.6);
//            return !(timer.seconds() >= 0.1);
//        }
//    }
//    public class clawose implements Action {
//        Servo clawose;
//
//        public clawose(Servo claw) {
//            this.clawose = claw;
//
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            clawose.setPosition(0);
//            return false;
//        }
//    }
//
//    //    @Config
//    public class preloaddhruv implements Action{
//        ElapsedTime timer;
//        int anglet=82
//                ,slidet=-1786;
//        double sp=1,ap=0.7;
//
//        public preloaddhruv() {
//
//        }
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(timer==  null){timer = new ElapsedTime();}
//            if (timer.seconds()<1.3){
//                angleMotor.setTargetPosition(anglet);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                angleMotor.setPower(1);
//                slideMotor.setTargetPosition(slidet);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(sp);
////                angleMotor.setPower(ap);
//            }
//
//            if (timer.seconds() >= 1.3) {
//                angleMotor.setTargetPosition(-20);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(-1);
//                slideMotor.setTargetPosition(0);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(-1);
//                if(slideMotor.getCurrentPosition()>-1400) {
//                    claw.setPosition(0.6);
//                }
//            }
//
//            return !(claw.getPosition()==0.6);
//        }
//    }
//}