//package org.firstinspires.ftc.teamcode.notuseful;
//
//import android.annotation.SuppressLint;
//import androidx.annotation.NonNull;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
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
//@Autonomous(name = "inAnayWeTrust180°", group = "Final")
//@Config
//public class newanay extends LinearOpMode {
//
//
//
////    Pose2d pick = new Pose2d(14.5 - 70.35, -18.83, -122);
////    Pose2d deliver = new Pose2d(19 - 70.350, 0.5, Math.PI);
////    Pose2d deliver1 = new Pose2d(19 - 70.350, 0.5+4, Math.PI);
////    Pose2d deliver2 = new Pose2d(19 - 70.350, 0.5+8, Math.PI);
////    Pose2d deliver3 = new Pose2d(19 - 70.350, 0.5+12, Math.PI);
//    // Hardware components
//    MecanumDrive drive;
//    Servo push;
//    static Servo claw;
//    static DcMotorEx angleMotor;
//    static DcMotorEx slideMotor;
//    DcMotor backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor;
//    Servo L_Hanger, R_Hanger;
//
//    // PID coefficients (configurable via FTC Dashboard)
////    public static double p = 0.1, i = 0, d = 0.01, f = 0.5;
//
//    // Actions
//    Action scorePreload, strafeSpike, obZone1, obZone1Back, obZone2, obZone2Back, obZone3, pickup1;
//    Action test, test2, test3, test4;
//
//
//
//    private void executeAutonomous() {
//        Actions.runBlocking(
//                new ParallelAction(
//                        new SequentialAction(
//                                new ParallelAction(scorePreload, new preloaddhruv()),
//                                new ParallelAction(strafeSpike, new Action() {
//                                    @Override
//                                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                                        push.setPosition(0.07);
//                                        return false;
//                                    }
//                                }),
//                                obZone1,
//                                obZone1Back,
//                                obZone2,
//                                obZone2Back,
//                                obZone3,
//                                test
////                                test2,
////                                test3,
////                                test4
//                        ),
//                        new Bla()
//                )
//        );
//    }
//
//    // Action classes
//    public class preloaddhruv implements Action {
//                ElapsedTime timer;
//        int anglet = 140, slidet = -1750;
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
//@Config
//    public static class Pick implements Action {
//        int a=0;
//        private ElapsedTime timer;
//        public static int angle1 = 840, slide1 = -1700, angle2 = -1750;
//        private int mode = 1;
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            if(timer==null)
////                timer = new ElapsedTime();
//            if (timer.seconds() < 1.2) {
//                angleMotor.setTargetPosition(angle1);
//
//                angleMotor.setPower(0.4);
//                slideMotor.setTargetPosition(slide1);
//
//                slideMotor.setPower(-1);
////                angleMotor.setPower(ap);
//            }
//            if (timer.seconds() > 1.2&&timer.seconds()<1.5) {
//                claw.setPosition(0);
//            }
//
//            if (timer.seconds() >= 1.4&&timer.seconds() <= 2) {
//                angleMotor.setTargetPosition(-1874);
////                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(1);
//                slideMotor.setTargetPosition(-800);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(-1);
//            }
//            if (timer.seconds() >= 2) {
//                slideMotor.setTargetPosition(-1500);
////                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(1);
//                claw.setPosition(0);
////                a =1;
//            }
//            return !(timer.seconds() >= 2.5);
//
//        }
//
//    }
//
//    public class Bla implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//
//
//            telemetry.addData("angle", angleMotor.getCurrentPosition());
//            telemetry.addData("slide", slideMotor.getCurrentPosition());
//            telemetry.update();
//            return true;
//        }
//    }
//
//    public class Blabla implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//            if (gamepad2.a || gamepad1.a) claw.setPosition(0);
//            if (gamepad2.b || gamepad1.b) claw.setPosition(0.6955);
//
//            // Manual control for angle and slide motors
//            angleMotor.setPower(-gamepad2.right_trigger / 1.5 + gamepad2.left_trigger / 1.5);
//            slideMotor.setPower(gamepad2.left_stick_y);
//
//            return !gamepad2.start;
//        }
//    }
//    @Override
//    public void runOpMode() {
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
////        L_Hanger = hardwareMap.get(Servo.class, "L_Hanger");
////        R_Hanger = hardwareMap.get(Servo.class, "R_Hanger");
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
//        push.setPosition(0.5);
//        slideMotor.setTargetPositionTolerance(0);
//        scorePreload = drive.actionBuilder(drive.pose)
//                .setTangent(0)
//                .strafeToConstantHeading(new Vector2d(12 - 70.35, 0))
//                .build();
//
//        strafeSpike = drive.actionBuilder(new Pose2d(12 - 70.35, 0, 0))
//                .setTangent(0)
//                .strafeTo(new Vector2d(38- 70.35, -31))
//                .build();
//
//        obZone1 = drive.actionBuilder(new Pose2d(38 - 70.35, -31, 0))
//                .setTangent(0)
//                .strafeTo(new Vector2d(3 - 70.35, -34.25))
//                .build();
//
//        obZone1Back = drive.actionBuilder(new Pose2d(3 - 70.35, -34, 0))
//                .setTangent(0)
//                .strafeTo(new Vector2d(40- 70.35, -39.25))
//                .build();
//
//        obZone2 = drive.actionBuilder(new Pose2d(40 - 70.35, -40, 0))
//                .setTangent(0)
//                .strafeTo(new Vector2d(3- 70.35, -46))
//                .build();
//
//        obZone2Back = drive.actionBuilder(new Pose2d(3 - 70.35, -46, 0))
//                .setTangent(0)
//                .strafeTo(new Vector2d(30 + 6 + 2 + 1 + 3 + 2 + 2 - 70.35, -40 - 9.5 +0.5))
//                .build();
//
//        obZone3 = drive.actionBuilder(new Pose2d(30 + 6 + 2 + 2 + 1 + 3 + 2 - 70.35, -40 - 9.5 + 0.5, 0))
//                .setTangent(0)
//                .lineToX(3 - 70.35)
//                .build();
//
//        pickup1 = drive.actionBuilder(new Pose2d(3 - 70.35, -50.5, 0))
//                .setTangent(0)
//                .strafeToLinearHeading(new Vector2d(-62.55, -36.5), Math.PI)
//                .build();
//
//        waitForStart();
//        executeAutonomous();
//    }
//}