package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "pleasehelpmeanay", group = "Final")
@Config
public class SpecimenAuto  extends LinearOpMode {

    public static double kG = 0.25;

    // Hardware components
    MecanumDrive drive;
    //    Servo push;
    static Servo claw,adj,push;
    static DcMotorEx angleMotor;
    static DcMotorEx slideMotor;
    DcMotor backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor;


    public static int target = 0;


    // Actions
    Action scorePreload,p2top,d1top,p1top,sleeep, backy,asd, obZone1,dtop, obZone1Back, strafeSpike1,woahbackup,obZone2,obZone2Back,obZone3,slep,ptod,strafeSpike2;

//public void initm() {

//}

    Action slideTo(int position) {
        return new Action() {
            private ElapsedTime timer;
            @Override
            public boolean run(TelemetryPacket p) {
                if(timer==null)
                    timer = new ElapsedTime();
                if(timer.seconds()<1.2) {
                    slideMotor.setTargetPosition(position);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotor.setPower(1);
                }
                return (slideMotor.isBusy());
            }
        };
    }
    Action slideTo1(int position) {
        return new Action() {
            private ElapsedTime timer;
            @Override
            public boolean run(TelemetryPacket p) {
                if(timer==null)
                    timer = new ElapsedTime();
                if(timer.seconds()<1.2) {
                    slideMotor.setTargetPosition(position);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotor.setPower(1);
                }
                return !(slideMotor.isBusy());
            }
        };
    }

    Action angleTo(int position) {
        return new Action() {
            private ElapsedTime timer;
            //            boolean arm = true;
            @Override
            public boolean run(TelemetryPacket p) {
                if(timer==null)
                    timer = new ElapsedTime();
                if(timer.seconds()<1.2) {
                    angleMotor.setTargetPosition(position);
                    angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    angleMotor.setPower(1);
                }
                return (angleMotor.isBusy());
            }

        };
    }
    Action angleTo1(int position) {
        return new Action() {
            private ElapsedTime timer;
            //            boolean arm = true;
            @Override
            public boolean run(TelemetryPacket p) {
                if(timer==null)
                    timer = new ElapsedTime();
                if(timer.seconds()<1.2) {
                    angleMotor.setTargetPosition(position);
                    angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    angleMotor.setPower(1);
                }
                return !(angleMotor.isBusy());
            }

        };
    }
    Action Adjto(double position) {
        return new Action() {
            //            private ElapsedTime timer;
            //            boolean arm = true;
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                adj.setPosition(position);
                return false;
            }

        };
    }
    Action claw1() {
        return new Action() {
            //            private ElapsedTime timer;
            //            boolean arm = true;
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
//                if(slideMotor.getCurrentPosition()<-600) {
                claw.setPosition(1);
//                }
                return false;
            }

        };
    }
    Action claw2() {
        return new Action() {
            //            private ElapsedTime timer;
            //            boolean arm = true;
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
//                if(slideMotor.getCurrentPosition()<-600) {

                claw.setPosition(0);
                claw.setPosition(0);
//                }
                return false;
            }

        };
    }
    Action claw3() {
        return new Action() {
            private ElapsedTime timer;
            //            boolean arm = true;
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
//                if(slideMotor.getCurrentPosition()<-600) {
                if(timer==null)
                    timer = new ElapsedTime();
                if(angleMotor.getCurrentPosition()<-175)
                    claw.setPosition(1);
//                }
                return !(claw.getPosition()==1);
            }

        };
    }
    Action claw4() {
        return new Action() {
            private ElapsedTime timer;
            //            boolean arm = true;
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
//                if(slideMotor.getCurrentPosition()<-600) {
                if(timer==null)
                    timer = new ElapsedTime();
                if(timer.seconds()>0.8)
                    claw.setPosition(0);
//                }
                return !(timer.seconds()>1.47 );
            }

        };
    }
    Action push(double position) {
        return new Action() {
            //            private ElapsedTime timer;
            //            boolean arm = true;
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
//                if(slideMotor.getCurrentPosition()<-600) {
                push.setPosition(position);
//                }
                return false;
            }

        };
    }


//    private void executeAutonomous() {
//
//    }




    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); //Initialize in init()
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        slideMotor = hardwareMap.get(DcMotorEx.class, "SM");
        claw = hardwareMap.get(Servo.class, "Claw");
        push = hardwareMap.get(Servo.class, "push");
        adj = hardwareMap.get(Servo.class, "Adj");
        angleMotor = hardwareMap.get(DcMotorEx.class, "AM");
        angleMotor.setTargetPositionTolerance(2);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        angleMotor.setTargetPosition(-1392);
//        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        angleMotor.setPower(-1);
//        sleep(4000);




        claw.setPosition(0);
        adj.setPosition(0.05);
        push.setPosition(1);
        slideMotor.setTargetPositionTolerance(1);

        AccelConstraint a = new ProfileAccelConstraint(-90,90);
        AccelConstraint ab = new ProfileAccelConstraint(-25,25);
        VelConstraint v = (pose2dDual, posePath, v1) -> 75;
        VelConstraint vb = (pose2dDual, posePath, v1) -> 50;

        scorePreload = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(27+0.5+1,6.7 ),v)//grab0, adj 0.35 ang 224 slide -747
                .build();

        strafeSpike1 = drive.actionBuilder(new Pose2d(27 , 6.7, 0))
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(37, -29.9),-Math.PI
                        ,v)
//            .waitSeconds(1.6)
                .build();

        obZone1 = drive.actionBuilder(new Pose2d(39.8, -35.1, 0))
                .setTangent(0)
                .strafeTo(new Vector2d(3 , -37),v,a)
                .build();

        obZone1Back = drive.actionBuilder(new Pose2d(4.5, -37, 0))
                .setTangent(0)
                .strafeTo(new Vector2d(40, -42),v,a)
                .build();

        obZone2 = drive.actionBuilder(new Pose2d(40 , -45.6, 0))
                .setTangent(0)
                .strafeTo(new Vector2d(3, -47),v,a)
                .build();
//x=37.03
        //y=-29.9
        //x2=36.5
        //y2-35.1
        //x3=36.7
        //y3=-45.2
        obZone2Back = drive.actionBuilder(new Pose2d(5 , -47, 0))
                .setTangent(0)
                .strafeTo(new Vector2d(38.3, -47),v,a)
                .build();

        obZone3 = drive.actionBuilder(new Pose2d(38.3, -51.5, 0))
                .setTangent(0)
                .lineToX(2  )
                .build();

        woahbackup = drive.actionBuilder(new Pose2d(5, -51.5, 0))
                .setTangent(0)
                .lineToX(13.20205107+1.5+1+1,vb)
                .build();
        backy = drive.actionBuilder(new Pose2d(13.20205107+1.5+1+1, -51.5, 0))
                .setTangent(0)
                .lineToX(6.944+0.8 ,vb,ab)
                .build();
        slep = drive.actionBuilder(new Pose2d(5, -51.5, 0))
                .setTangent(0)
                .waitSeconds(0.5)
                .build();
        sleeep = drive.actionBuilder(new Pose2d(5, -51.5, 0))
                .setTangent(0)
                .waitSeconds(1)
                .build();
















        asd = drive.actionBuilder(new Pose2d(6.944+0.8 , -51.5, 0))
                .setTangent(0)
                .strafeToLinearHeading(new Vector2d(27,6.7 ),Math.PI)
                .build();

        ptod = drive.actionBuilder(new Pose2d(6.944+0.8, -51.5, 0))
                .setTangent(0)
//                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(18,3.48))
                .strafeToConstantHeading(new Vector2d(23.3-0.5,3.48))
                .build();
        dtop = drive.actionBuilder(new Pose2d(23.3, 3.48, 0))
                .setTangent(0)
//                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(18-1.35-3.5-1.35-0.25-0.35,-36))
                .build();
        d1top = drive.actionBuilder(new Pose2d(23, 3.48+3, 0))
                .setTangent(0)
//                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(18-1.35-3.5-1.35-0.25-0.35,-36))
                .build();
        p1top = drive.actionBuilder(new Pose2d(18-1.35-3.5-1.35-0.25-0.35, -36, 0))
                .setTangent(0)
//                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(17,3.48+3))
                .strafeToConstantHeading(new Vector2d(23-0.5,3.48+3))
                .build();
        p2top = drive.actionBuilder(new Pose2d(18-1.35-3.5-1.35-0.25-0.35, -36, 0))
                .setTangent(0)
//                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(18,3.48))
                .strafeToConstantHeading(new Vector2d(23,1))
                .build();
//                .build();
        sleep(1000);
        angleMotor.setPower(0);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





//        del1 = drive.actionBuilder(new Pose2d(20.3, -51.12, -Math.toRadians(52.127)))
//
//                .strafeToLinearHeading(new Vector2d(5.6, -25.234),0,v)
//                .waitSeconds(1.6)
//                .build();
//
//        pick1 = drive.actionBuilder(new Pose2d(5.6, -25.234, 0))
//
//                .strafeTo(new Vector2d(29.14, 5.04),v,a)
//                .build();
//
//        del2 = drive.actionBuilder(new Pose2d(29.14, 5.04, 0))
//
//                .strafeTo(new Vector2d(5.6, -25.234),v)
//                .build();

        telemetry.addData("init","complete");
        telemetry.update();
        waitForStart();// -2039 //-252
        Actions.runBlocking(
                new SequentialAction(
//                                angleTo(-1000),slideTo(-500)
                        new ParallelAction(scorePreload,slideTo1(-792),angleTo(182),Adjto(0.39)),push(0),new ParallelAction(slideTo1(1),claw1(), strafeSpike1),obZone1,new ParallelAction(obZone1Back,push(0.5)),push(0),obZone2,obZone2Back,
                        new ParallelAction(obZone3,Adjto(0.55),slideTo1(-272),angleTo(-930)),woahbackup,
                        new ParallelAction(angleTo1(-2040), slideTo1(-200),backy)
                        , claw2(),slep,new ParallelAction(angleTo(75),slideTo(-630),ptod,Adjto(0.545)),new ParallelAction(claw3(),angleTo1(-1500)),new ParallelAction(dtop,slideTo1(-265)),angleTo1(-2027),claw4(),slep,
                        new ParallelAction(angleTo(75),slideTo(-640),p1top,Adjto(0.545)),new ParallelAction(claw3(),angleTo1(-1500)),new ParallelAction(d1top,slideTo1(-265)),angleTo1(-2027),claw4(),slep,
                        new ParallelAction(angleTo(75),slideTo(-640),p2top,Adjto(0.545)),new ParallelAction(claw3(),angleTo1(-1500)),ptod
//                        new ParallelAction(dtop,slideTo1(-265)),angleTo(-2027),claw4(),
//                        slep,new ParallelAction(angleTo(75),slideTo(-640),p1top,Adjto(0.545)),new ParallelAction(claw3(),angleTo(-1500)),ptod
//                        new ParallelAction(dtop,slideTo1(-265)),angleTo(-2027),claw4(),slep,new ParallelAction(angleTo(75),slideTo(-640),p1top,Adjto(0.545)),new ParallelAction(claw3(),angleTo(-1500)),ptod

                )

        );
    }

}