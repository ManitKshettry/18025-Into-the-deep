//package org.firstinspires.ftc.teamcode;
//import java.util.Arrays;
//
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//
//
//import com.acmerobotics.roadrunner.SequentialAction;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//
//
//@Config
//@Autonomous(name = "INCERCARE 1+4 BARA", group = "Autonomous")
//public class AutoBara extends LinearOpMode {
//    private DcMotor glism1 = null;
//    private DcMotor brat = null;
//    private DcMotor b_extindere = null;
//    private Servo gheara = null;
//    private DcMotor IntakePasiv = null;
//    private Servo RotireIntake = null;
//    private CRServo IntakeTurn = null;
//    private ColorSensor colorSensor;
//    private static final double MAX_VEL = 60; // Max velocity in inches per second
//    private static final double MAX_ANG_VEL = Math.toRadians(200); // Max angular velocity in radians per second
//
//    private final MinVelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
//            new AngularVelConstraint(MAX_ANG_VEL),
//            new TranslationalVelConstraint(MAX_VEL)
//    ));
//    private final ProfileAccelConstraint accelConstraint = new ProfileAccelConstraint(-80,80);
//
//    enum AutoState {
//        INIT, TRAJ1, TRAJ2, TRAJ3, TRAJ4, TRAJ5, TRAJ6, TRAJ7, TRAJ8, TRAJ9,COMPLETE
//    }
//    AutoState currentState = AutoState.INIT;
//
//    private void initializeMotors() {
//        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        glism1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        glism1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        b_extindere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        b_extindere.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        brat.setDirection(DcMotorSimple.Direction.REVERSE);
//        glism1.setDirection(DcMotorSimple.Direction.REVERSE);
//        glism1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        IntakePasiv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        IntakePasiv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        IntakePasiv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        IntakePasiv.setDirection(DcMotorSimple.Direction.REVERSE);
//        IntakeTurn.setPower(0);
//        gheara.setPosition(1);
//    }
//
//    private Action moveBratToPosition(int position) {
//        return new Action() {
//            @Override
//            public boolean run(TelemetryPacket p) {
//                brat.setTargetPosition(position);
//                brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                brat.setPower(1);
//                return !brat.isBusy();
//            }
//        };
//    }
//
//    private Action moveExtindereToPosition(int position) {
//        return new Action() {
//            @Override
//            public boolean run(TelemetryPacket p) {
//                b_extindere.setTargetPosition(position);
//                b_extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                b_extindere.setPower(1);
//                return !b_extindere.isBusy();
//            }
//        };
//    }
//
//    private Action moveIntakeToPosition(int encoderCounts) {
//        return new Action() {
//            @Override
//            public boolean run(TelemetryPacket p) {
//                IntakePasiv.setTargetPosition(encoderCounts);
//                IntakePasiv.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                IntakePasiv.setPower(1);
//                return !IntakePasiv.isBusy();
//            }
//        };
//    };
//
//
//    private Action spinServoAction(Servo servo, double targetPosition) {
//        return packet -> {
//            servo.setPosition(targetPosition);
//
//            // Add telemetry
//            packet.put("Servo Name", servo.getDeviceName());
//            packet.put("Servo Target Position", targetPosition);
//            packet.put("Servo Current Position", servo.getPosition());
//
//            return false; // Indicates that the action is complete
//        };
//    }
//
//
//    @Override
//    public void runOpMode() {
//        Pose2d startPose = new Pose2d(1, -61, Math.toRadians(0));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
//        glism1 = hardwareMap.get(DcMotor.class, "glism");
//        brat = hardwareMap.get(DcMotor.class, "brat");
//        b_extindere = hardwareMap.get(DcMotor.class, "b_extindere");
//        gheara = hardwareMap.get(Servo.class, "gheara");
//        RotireIntake = hardwareMap.get(Servo.class, "IntakeSpin");
//        glism1 = hardwareMap.get(DcMotor.class, "glism");
//        IntakeTurn = hardwareMap.get(CRServo.class, "IntakeTurn");
//        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
//        IntakePasiv = hardwareMap.get(DcMotor.class, "IntakePasiv");
//
//
//        initializeMotors();
//
//        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);
//        Action traj0 = builder
//                .strafeToLinearHeading(new Vector2d(3,-28.5), Math.toRadians(0), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj0Pose = new Pose2d(3,-29,Math.toRadians(0));
//
//        Action traj1 = drive.actionBuilder(traj0Pose)
//                .strafeToLinearHeading(new Vector2d(3,-35), Math.toRadians(0), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj1Pose = new Pose2d(3,-35,Math.toRadians(0));
//
//        Action traj2 = drive.actionBuilder(traj1Pose)
//                .strafeToLinearHeading(new Vector2d(35,-50), Math.toRadians(90), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj2Pose = new Pose2d(35,-50,Math.toRadians(90));
//
//        Action traj3 = drive.actionBuilder(traj2Pose)
//                .strafeToLinearHeading(new Vector2d(35,-14), Math.toRadians(90), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj3Pose = new Pose2d(35,-14,Math.toRadians(90));
//
//        Action traj4 = drive.actionBuilder(traj3Pose)
//                .strafeToLinearHeading(new Vector2d(45,-14), Math.toRadians(90), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj4Pose = new Pose2d(45,-14,Math.toRadians(90));
//
//        Action traj5 = drive.actionBuilder(traj4Pose)
//                .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(90), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj5Pose = new Pose2d(45,-50,Math.toRadians(90));
//
//        Action traj6 = drive.actionBuilder(traj5Pose)
//                .strafeToLinearHeading(new Vector2d(45,-14), Math.toRadians(90), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj6Pose = new Pose2d(45,-14,Math.toRadians(90));
//
//        Action traj7 = drive.actionBuilder(traj6Pose)
//                .strafeToLinearHeading(new Vector2d(53,-14), Math.toRadians(90), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj7Pose = new Pose2d(53,-14,Math.toRadians(90));
//
//        Action traj8 = drive.actionBuilder((traj7Pose))
//                .strafeToLinearHeading(new Vector2d(55,-50), Math.toRadians(90), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj8Pose = new Pose2d(55,-50,Math.toRadians(90));
//
//        Action traj9 = drive.actionBuilder(traj8Pose)
//                .strafeToLinearHeading(new Vector2d(55,-40), Math.toRadians(180), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj9Pose = new Pose2d(55,-40,Math.toRadians(180));
//
//        Action traj10 = drive.actionBuilder(traj9Pose)
//                .strafeToLinearHeading(new Vector2d(55,-65), Math.toRadians(180), velConstraint,accelConstraint)
//                .build();
//        Pose2d traj10Pose = new Pose2d(55,-65,Math.toRadians(180));
//
//        Action traj11 = drive.actionBuilder(traj10Pose)
//                .strafeToLinearHeading(new Vector2d(-9,-29),Math.toRadians(0),velConstraint)
//                .build();
//        Pose2d traj11Pose = new Pose2d(-9,-29,Math.toRadians(0));
//
//        Action traj12 = drive.actionBuilder(traj11Pose)
//                .strafeToLinearHeading(new Vector2d(55,-40),Math.toRadians(180),velConstraint)
//                .build();
//        Pose2d traj12Pose = new Pose2d(55,-40,Math.toRadians(180));
//
//        Action traj13 = drive.actionBuilder((traj12Pose))
//                .strafeToLinearHeading(new Vector2d(55,-65), Math.toRadians(180), velConstraint)
//                .build();
//        Pose2d traj13Pose = new Pose2d(55,-65,Math.toRadians(180));
//
//        Action traj14 = drive.actionBuilder(traj13Pose)
//                .strafeToLinearHeading(new Vector2d(0,-29),Math.toRadians(0),velConstraint)
//                .build();
//        Pose2d traj14Pose = new Pose2d(0,-29,Math.toRadians(0));
//
//        Action traj15 = drive.actionBuilder(traj14Pose)
//                .strafeToLinearHeading(new Vector2d(-9,-29),Math.toRadians(0),velConstraint)
//                .build();
//        Pose2d traj15Pose = new Pose2d(-9,-29,Math.toRadians(0));
//
//        Action traj16 = drive.actionBuilder(traj15Pose)
//                .strafeToLinearHeading(new Vector2d(55,-40),Math.toRadians(180),velConstraint)
//                .build();
//        Pose2d traj16Pose = new Pose2d(55,-40,Math.toRadians(180));
//
//        Action traj17 = drive.actionBuilder((traj16Pose))
//                .strafeToLinearHeading(new Vector2d(55,-65), Math.toRadians(180), velConstraint)
//                .build();
//        Pose2d traj17Pose = new Pose2d(55,-65,Math.toRadians(180));
//
//        Action traj18 = drive.actionBuilder(traj17Pose)
//                .strafeToLinearHeading(new Vector2d(-3,-29),Math.toRadians(0),velConstraint)
//                .build();
//        Pose2d traj18Pose = new Pose2d(-3,-29,Math.toRadians(0));
//        waitForStart();
//        if (opModeIsActive()) {
//            while (opModeIsActive() && !isStopRequested()) {
//                switch (currentState) {
//                    case INIT:
//                        currentState = AutoState.TRAJ1;
//                        break;
//                    case TRAJ1:
//                        Actions.runBlocking(new ParallelAction(
//                                traj0,
//                                moveIntakeToPosition(1370)
//                        ));
//                        sleep(50);
//                        Actions.runBlocking(moveIntakeToPosition(0));
//                        Actions.runBlocking(traj1);
//                        currentState = AutoState.TRAJ2;
//                        break;
//                    case TRAJ2:
//                        Actions.runBlocking(traj2);
//                        Actions.runBlocking(traj3);
//                        Actions.runBlocking(traj4);
//                        Actions.runBlocking(traj5);
//                        Actions.runBlocking(traj6);
//                        Actions.runBlocking(traj7);
//                        Actions.runBlocking(traj8);
//                        Actions.runBlocking(traj9);
//                        Actions.runBlocking(traj10);
//                        Actions.runBlocking(new ParallelAction(
//                                traj11,
//                                moveIntakeToPosition(1370)
//                        ));
//                        Actions.runBlocking(moveIntakeToPosition(0));
//                        Actions.runBlocking(traj11);
//                        Actions.runBlocking(traj12);
//                        Actions.runBlocking(traj13);
//                        Actions.runBlocking(new ParallelAction(
//                                traj14,
//                                moveIntakeToPosition(1370)
//                        ));
//                        Actions.runBlocking(moveIntakeToPosition(0));
//                        Actions.runBlocking(traj15);
//                        Actions.runBlocking(traj16);
//                        Actions.runBlocking(traj17);
//                        Actions.runBlocking(new ParallelAction(
//                                traj18,
//                                moveIntakeToPosition(1370)
//                        ));
//                        Actions.runBlocking(moveIntakeToPosition(0));
//                        sleep(600);
//                        /*
//                        sleep(200);
//                        Actions.runBlocking(traj6);
//                        sleep(200);
//                        Actions.runBlocking(traj7);
//                        sleep(100);
//                        Actions.runBlocking(new ParallelAction(
//                                moveIntakeToPosition(1400),
//                                traj8
//                        ));
//                        sleep(200);
//                        Actions.runBlocking(moveIntakeToPosition(0));
//                        sleep(600);
//                        Actions.runBlocking(traj9);
//                        sleep(100);
//                        Actions.runBlocking(traj10);
//                        Actions.runBlocking(new ParallelAction(
//                                moveIntakeToPosition(1350),
//                                traj11
//                        ));
//                        sleep(300);
//                        Actions.runBlocking(moveIntakeToPosition(0));
//                        sleep(600);*/
//                        currentState = AutoState.COMPLETE;
//                    case COMPLETE:
//                        telemetry.addData("State", "COMPLETE");
//                        requestOpModeStop();
//                        break;
//                }}}}}