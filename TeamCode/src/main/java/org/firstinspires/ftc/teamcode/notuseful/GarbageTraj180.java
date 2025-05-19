//package org.firstinspires.ftc.teamcode;
//
//import android.annotation.SuppressLint;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.MinMax;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.RamseteController;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.MotorPidCoefficientsParameters;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//
//import java.util.List;
//
//@Autonomous(name = "inAnayWeTrust180°", group = "Final")
//@Config
//public class GarbageTraj180 extends LinearOpMode {
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
//    private Servo L_Hanger;
//    private Servo R_Hanger;
//    public static double p = 0.1, i = 0, d = 0.01, f = 0.5;
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
//    Action test, test2, test3;
//    Action test4;
//    Action scoreee1;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        drive = new MecanumDrive(hardwareMap, new Pose2d(-70.35, 0, 0)); //Initialize in init()
//
//        Pose2d pick = new Pose2d(13.20 - 70.35, -7.3, -122);
//        Pose2d deliver = new Pose2d(14.68 - 70.350, -1.4 + 1.5 + 2, Math.PI);
//        Pose2d deliver1 = new Pose2d(14.68 - 70.350, -1.4 + 2.5 + 3 + 2, Math.PI);
//        Pose2d deliver2 = new Pose2d(14.68 - 70.350, -1.4 + 2.5 + 6 + 2, Math.PI);
//        Pose2d deliver3 = new Pose2d(14.68 - 70.350, -1.4 + 2.5 + 6 + 2 + 3, Math.PI);
//
//        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
//            if (robotPose.position.x.value() < 28 + 4 + 2 + 1 + 4 + 2 - 70.35) {
//                return 100.0;
//            } else {
//                return 75.0;
//            }
//        };
//
//        angleMotor = hardwareMap.get(DcMotorEx.class, "AM");
//        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        backRightMotor = hardwareMap.dcMotor.get("rightBack");
//        slideMotor = hardwareMap.get(DcMotorEx.class, "SM");
////        boreThroughEnc = hardwareMap.get(DcMotorEx.class, "Back_Wind");
//        push = hardwareMap.get(Servo.class, "push");
//        claw = hardwareMap.get(Servo.class, "Claw");
//        L_Hanger = hardwareMap.get(Servo.class, "L_Hanger");
//        R_Hanger = hardwareMap.get(Servo.class, "R_Hanger");
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
//        angleMotor.setTargetPositionTolerance(1);
//        claw.setPosition(-0);
//        slideMotor.setTargetPositionTolerance(0);
////        angleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,f));
////        angleMotor.setTargetPosition(-1190);
////        push.setPosition(1);
////        L_Hanger.setPosition(0.97);
////        R_Hanger.setPosition(0.17);
//
//        scoreprelload = drive.actionBuilder(drive.pose)
//                .setTangent(0)
//                .strafeToConstantHeading(new Vector2d(12 - 70.35, 0))
//                .build();
//
//        strafespike = drive.actionBuilder(new Pose2d(12 - 70.35, 0, 0))
//                .setTangent(0)
//                //.stopandAdd(new pushy(push))
//                .strafeTo(new Vector2d(28 + 4 + 2 + 1 + 2 - 70.35, -30.1))
//                .build();
//
//        obzone1 = drive.actionBuilder(new Pose2d(28 + 4 + 2 + 1 + 2 - 70.35, -30.1, 0))
//                .setTangent(0)
////                .waitSeconds(0.54444444476476764767)
//                .strafeTo(new Vector2d(3 - 70.35, -34.25))
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
//                .strafeTo(new Vector2d(30 + 6 + 2 + 1 + 3 + 2 + 2 - 70.35, -40 - 9.5 + 2.5))
////                .waitSeconds(0.1)
//                .build();
//        obzone3 = drive.actionBuilder(new Pose2d(30 + 6 + 2 + 2 + 1 + 3 + 2 - 70.35, -40 - 9.5 + 2.5, 0))
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
////                //.stopandAdd(new clawy(claw))
////                .strafeToLinearHeading(deliver180.component1(), Math.toRadians(0))
////                .build();
//        test = drive.actionBuilder(new Pose2d(new Vector2d(3 - 70.35, -50.5), 0))
//                .setTangent(0)
//                .strafeToLinearHeading(pick.component1(), -Math.toRadians(123))
////                .waitSeconds(0.5)
//                //.stopandAdd(new pick ())
//
//                .stopAndAdd(new blabla())
//                .stopAndAdd(new pick())
//
////                .waitSeconds(0.2)
////                //.stopandAdd(new clawose(claw))
////                .waitSeconds(1)
//                .setTangent(0)
//                .strafeToLinearHeading(deliver.component1(), Math.PI)
//                .stopAndAdd(new blabla())
////                .waitSeconds(1)
//                //.stopandAdd(new deliver())
////                .waitSeconds(2)
//
//
//                .build();
//        test2 = drive.actionBuilder(deliver)
//                .setTangent(0)
//                .strafeToLinearHeading(pick.component1(), -Math.toRadians(123))
////                .strafeToLinearHeading(pick.component1(), -Math.toRadians(123))
////                .waitSeconds(0.1)
//                //.stopandAdd(new pick1 ())
//
////                .waitSeconds(0.1)
////                //.stopandAdd(new clawose(claw))
////                .waitSeconds(1)
//                .setTangent(0)
//                .strafeToLinearHeading(deliver1.component1(), Math.PI)
////                .waitSeconds(1)
//                //.stopandAdd(new deliver())
////                .waitSeconds(2)
//
//
//                .build();
//        test3 = drive.actionBuilder(deliver1)
//                .setTangent(0)
//                .strafeToLinearHeading(pick.component1(), -Math.toRadians(123))
////                .waitSeconds(0.1)
//                //.stopandAdd(new pick1 ())
////                .waitSeconds(0.1)
////                //.stopandAdd(new clawose(claw))
////                .waitSeconds(1)
//                .setTangent(0)
//                .strafeToLinearHeading(deliver2.component1(), Math.PI)
////                .waitSeconds(1)
//                //.stopandAdd(new deliver())
//                .build();
////                .waitSeconds(2)
//        test4 = drive.actionBuilder(deliver2)
//                .setTangent(0)
//                .strafeToLinearHeading(pick.component1(), -Math.toRadians(123))
////                .waitSeconds(0.1)
//                //.stopandAdd(new pick1 ())
////                .waitSeconds(0.1)
////                //.stopandAdd(new clawose(claw))
////                .waitSeconds(1)
//                .setTangent(0)
//                .strafeToLinearHeading(deliver3.component1(), Math.PI)
////                .waitSeconds(1)
//                //.stopandAdd(new deliver())
////                .waitSeconds(2)
//                .build();
//
//
//        waitForStart();
//        double ap1 = 1;
//        Actions.runBlocking(
//                new ParallelAction(new SequentialAction(
//                        new ParallelAction(scoreprelload, new preloaddhruv()),
//                        strafespike,
//                        obzone1,
//                        obzone1back, new pushy(push),
//                        obzone2,
//                        obzone2back,
//                        obzone3,
////                        pickup1
//                        test,
//                        test2,
//                        test3,
//                        test4
//                ), new bla())
//        );
//    }
//
//
//    public class preloaddhruv implements Action {
//        ElapsedTime timer;
//        int anglet = 49, slidet = -1900;
//        double sp = 1, ap = 0.7;
//
//        public preloaddhruv() {
//
//        }
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (timer == null) {
//                timer = new ElapsedTime();
//            }
//            if (timer.seconds() < 1.2) {
//                angleMotor.setTargetPosition(-90);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(-1);
//                slideMotor.setTargetPosition(slidet);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(sp);
////                angleMotor.setPower(ap);
//            }
//
//            if (timer.seconds() >= 1.2) {
//                angleMotor.setTargetPosition(anglet);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(1);
//                slideMotor.setTargetPosition(-1000);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(-1);
//                if (slideMotor.getCurrentPosition() > -1570) {
//                    claw.setPosition(0.6955);
//                }
//            }
//
//            return !(claw.getPosition() == 0.6955);
//        }
//    }
//
//@Config
//    public static class pick implements Action{
//        public static int angle1 = 725
//                , slide1=-1856,angle2=-1750,slidedel=-1700;
//        int MODELA = 1;
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (MODELA==1) {
//                angleMotor.setTargetPosition(angle1);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(0.7);
//                slideMotor.setTargetPosition(slide1);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(-1);
//                if(slideMotor.getCurrentPosition()==slide1-1||slideMotor.getCurrentPosition()==slide1||slideMotor.getCurrentPosition()==slide1+1)
//                    MODELA=2;
//            }
//            if (MODELA==2){
//                angleMotor.setTargetPosition(angle2);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(-1);
//                slideMotor.setTargetPosition(-1000);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(1);
//                if(slideMotor.getCurrentPosition()==-1000)
//                    MODELA=3;
//            }
////            if(MODELA==3){
////                slideMotor.setTargetPosition(slidedel);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slideMotor.setPower(-1);
////                try {
////                    Thread.sleep(100);
////                } catch (InterruptedException e) {
////                    throw new RuntimeException(e);
////                }
////                slideMotor.setTargetPosition(-1000);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slideMotor.setPower(1);
////                if (slideMotor.getCurrentPosition()==-1000)
////                    MODELA=4;
////            }
//
//
//
//
//            return !(MODELA==3);
//        }
//    }
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    public class bla implements Action {
//        public bla() {
//            double ap1 = 1;
//
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
////            if (angleMotor.getCurrentPosition() > -500)
////                angleBrakePower = -(0.00005 * slideMotor.getCurrentPosition());
////            else if (angleMotor.getCurrentPosition() < -1500)
////                angleBrakePower = 0.00005 * slideMotor.getCurrentPosition();
////            else
////                angleBrakePower = 0;
//            telemetry.addData("angle", angleMotor.getCurrentPosition());
//            telemetry.addData("slide", slideMotor.getCurrentPosition());
////            telemetry.addData("bore",boreThroughEnc.getCurrentPosition());
////        telemetry.addData("slide",slideMotor.getCurrentPosition());
//            telemetry.update();
//            return true;
//        }
//    }
//
//    public class blabla implements Action {
//        public blabla() {
//            double ap1 = 1;
//
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            if (gamepad2.a || gamepad1.a) {
//                claw.setPosition(0);
//            }
//            if (gamepad2.b || gamepad1.b) {
//                claw.setPosition(0.6955);
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
//            if (gamepad2.right_trigger > 0.05) {
//                angleMotor.setPower(-gamepad2.right_trigger / 1.5);
//            } else if (gamepad2.left_trigger > 0.05) {
//                angleMotor.setPower(gamepad2.left_trigger / 1.5);
//            } else {
//                angleMotor.setPower(-0);
//            }
//
//            //Slide Motor
////            if (slideMotor.getCurrentPosition() < -3100)
//            {
//                if (gamepad2.left_stick_y > 0.1) {
//                    slideMotor.setPower(gamepad2.left_stick_y);
//                } else {
//                    slideMotor.setPower(0);
//                }
//            }
////            else if (slideMotor.getCurrentPosition() > -10)
//            {
//                if (gamepad2.left_stick_y < -0.1) {
//                    slideMotor.setPower(gamepad2.left_stick_y);
//                } else {
//                    slideMotor.setPower(0);
//                }
//            }
////           a else
//            {
//                if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
//                    slideMotor.setPower(gamepad2.left_stick_y);
//                } else {
//                    slideMotor.setPower(0);
//                }
//            }
//            return !(gamepad2.start);
//        }
//    }
//
//    //    @Config
////
////    public static class pick implements Action{
////        ElapsedTime timer;
////        public static int anglet=725+10
//////                ,anglet2=-2817
////                ,slidet=-1700-14-3;
////        //                ,slidet2=-428;
////        double sp=-1,ap=1;
////
////        public pick() {
//////            this.ap =ap1;
//////            this.SM =slideMotor;
////
////
////        }
////        @SuppressLint("SuspiciousIndentation")
////        @Override
////        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
////            if(timer==  null){timer = new ElapsedTime();}
////
//////            ap = (angleMotor.getCurrentPosition()<-1460) ? 1:0.3;
//////            telemetryPacket.put("ap",ap);
//////            telemetryPacket.addLine(String.valueOf(ap));
////
////            if (timer.seconds()<1) {
////                angleMotor.setTargetPosition(anglet);
//////                //angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                angleMotor.setPower(ap);
////                slideMotor.setTargetPosition(slidet);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slideMotor.setPower(sp);
////            }
////            if (slideMotor.getCurrentPosition()>(slideMotor.getTargetPosition()-5)&&slideMotor.getCurrentPosition()<(slideMotor.getTargetPosition()+5))
////                claw.setPosition(0.12);
////            if(timer.seconds()>1.2) {
////                slideMotor.setTargetPosition(-900);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slideMotor.setPower(-sp);
////            }
////            if(timer.seconds()>1.2) {
////                angleMotor.setTargetPosition(-1831-50);
//////                angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                angleMotor.setPower(-0.8);
////
////            }
////            return !(timer.seconds()>1.21);
////        }
////    }
////@Config
////    public static class pick1 implements Action{
////        ElapsedTime timer;
////        public static int anglet=725+10+20
//////                ,anglet2=-2817
////                ,slidet=-1518-10-5-5;
////        //                ,slidet2=-428;
////        double sp=-1,ap=1;
////
////        public pick1() {
//////            this.AM =angleMotor;
//////            this.SM =slideMotor;
////
////
////        }
////        @SuppressLint("SuspiciousIndentation")
////        @Override
////        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
////            if(timer==  null){timer = new ElapsedTime();}
////
////
////            telemetryPacket.put("time",timer);
////            if (timer.seconds()<1.2) {
////                angleMotor.setTargetPosition(anglet);
////                //angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////                angleMotor.setPower(ap);
////                slideMotor.setTargetPosition(slidet);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slideMotor.setPower(sp);
////            }
////            if (timer.seconds()>=1)
////                claw.setPosition(0.12);
////            if(timer.seconds()>1.5) {
////                slideMotor.setTargetPosition(-600);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slideMotor.setPower(-sp);
////            }
////            if(timer.seconds()>1.5) {
////                angleMotor.setTargetPosition(-1831-50);
//////                angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                angleMotor.setPower(-1);
////
////            }
////            return !(timer.seconds()>1.41);
////        }
////    }
////
////
////    @Config
////    public static class deliver implements Action{
////        ElapsedTime timer;
////        public static int angle1=-1750
////                ,slide1=-1540-20,slide2=-900,slidedrop=-1007;
////        double sp=-0.775,ap=0.7;
////
////        public deliver() {
////
////        }
////        @SuppressLint("SuspiciousIndentation")
////        @Override
////        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
////            if(timer==  null){timer = new ElapsedTime();}
////            if (timer.seconds()<0.8){
////                slideMotor.setTargetPosition(slide1);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slideMotor.setPower(sp+0.2);
//////                angleMotor.setPower(ap);
////            }
////
////            if (timer.seconds() >= 0.8) {
////                angleMotor.setTargetPosition(angle1);
////                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                angleMotor.setPower(1);
////            }
////            if (timer.seconds() >= 1) {
////                slideMotor.setTargetPosition(slide2);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                slideMotor.setPower(1);
////
////            }
////            if(slideMotor.getCurrentPosition()>-1280&&timer.seconds()>=0.9) {
////                claw.setPosition(0.6955);
////
////            }
////
////            return !(claw.getPosition()==0.6955);
////        }
////    }
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
//            if (timer == null) {
//                timer = new ElapsedTime();
//            }
//            pushy.setPosition(0.07);
//            return !(timer.seconds() > 0.1);
//        }
//    }
//
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
//
////
//
//    //-1190
//    //    @Config
//
//
//}