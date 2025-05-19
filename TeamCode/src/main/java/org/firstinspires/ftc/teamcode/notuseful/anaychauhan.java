////package org.firstinspires.ftc.teamcode;
////
////import com.acmerobotics.roadrunner.Pose2d;
////import com.acmerobotics.roadrunner.Vector2d;
////import com.acmerobotics.roadrunner.ftc.Actions;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.eventloop.opmode.OpMode;
////
////@TeleOp(name = "Bakwaasbutside", group = "newnewnew")
////public class anaychauhan extends LinearOpMode {
////    MecanumDrive drive;
////
////
////    @Override
////    public void runOpMode() throws InterruptedException {
////        waitForStart();
////        drive = new MecanumDrive(hardwareMap, new Pose2d(-70.35, 0, 0)); // Initialize in init()
////        Actions.runBlocking(
////                drive.actionBuilder(new Pose2d(-70.35, 0, 0))
////                        //Step: 1
////                        .splineToConstantHeading(new Vector2d(-39.75, -27.5), 0)
////                        .waitSeconds(1.5)
////
////                        //Step: 2
////                        .lineToXConstantHeading(-45)
////                        .waitSeconds(0.5)
////
////                        //Step: 3
////                        .strafeToLinearHeading(new Vector2d(-45,5.5),Math.toRadians(90))
////                        .waitSeconds(0.5)
////
////                        //Step: 4
////                        .strafeToConstantHeading(new Vector2d(-28.75,5.5))
////                        .waitSeconds(1)
////
////                        //Step: 5
////                        .strafeToLinearHeading(new Vector2d(-63.75,23.5),Math.toRadians(135))
////                        .waitSeconds(1)
////
////                        //Step: 6
////                        .strafeToLinearHeading(new Vector2d(-28.75,12),Math.toRadians(90))
////                        .strafeToConstantHeading(new Vector2d(-28.75,14.575))
////                        .waitSeconds(1)
////
////                        //Step: 7
////                        .strafeToLinearHeading(new Vector2d(-63.75,23.5),Math.toRadians(135))
////                        .waitSeconds(1)
////
////                        //Step: 8
////                        .strafeToLinearHeading(new Vector2d(-28.75,26.5-1),Math.toRadians(90))
////                        .waitSeconds(1)
////
////                        //Step: 9
////                        .strafeToLinearHeading(new Vector2d(-63.75,23.5),Math.toRadians(135))
////                        .waitSeconds(1)
////
////                        //Step: ah yes odometry
////                        .setTangent(2*Math.PI)
////                        .splineToLinearHeading(new Pose2d(-16.75,-5.5,Math.toRadians(90)), Math.toRadians(270))
////                        .build());
////        //Ah yes Odometry
////    }
////}
////
//package  org.firstinspires.ftc.teamcode;
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
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Config
//@Autonomous(name = "Template Autoop", group = "16481-Example")
//public class anaychauhan extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
//
//        DcMotor motor1 = hardwareMap.get(DcMotor.class,  "motor");
//
//        // Delcare Trajectory as such
//        Action TrajectoryAction1 = drive.actionBuilder(drive.pose)
//                .lineToX(10)
//                .build();
//
////        Action TrajectoryAction2 = drive.actionBuilder(new Pose2d(15,20,0))
////                .splineTo(new Vector2d(5,5), Math.toRadians(90))
////                .build();
//
//
//        while(!isStopRequested() && !opModeIsActive()) {
//
//        }
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        TrajectoryAction1, // Example of a drive action
//
//                        // This action and the following action do the same thing
//                        new Action() {
//                            @Override
//                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                                telemetry.addLine("Action!");
//                                telemetry.update();
//                                return false;
//                            }
//                        },
//                        // Only that this action uses a Lambda expression to reduce complexity
//                        (telemetryPacket) -> {
//                            telemetry.addLine("Action!");
//                            telemetry.update();
//                            return false; // Returning true causes the action to run again, returning false causes it to cease
//                        },
//                        new ParallelAction( // several actions being run in parallel
////                                TrajectoryAction2, // Run second trajectory
//                                (telemetryPacket) -> { // Run some action
//                                    motor1.setPower(1);
//                                    return false;
//                                }
//                        ),
//                        drive.actionBuilder(new Pose2d(15,10,Math.toRadians(125))) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
//                                .splineTo(new Vector2d(25, 15), 0)
//                                .build()
//
//                )
//        );
//
//
//    }
//
//}