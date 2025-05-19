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
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous(name = "inDhruvGuptaWeTrust", group = "Final")
//public class dhruvtraj extends OpMode
//{
//    double x,y;
//    MecanumDrive drive;
//    Servo push;
//    Servo claw;
//
//    //Intake Motors
//    private DcMotorEx angleMotor;
//    private DcMotorEx slideMotor;
//
//    //Wheelbase Motors
//    private DcMotor backRightMotor;
//    private DcMotor backLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor frontLeftMotor;
//
//    //Actions
//    Action scorePreload;
//    Action strafeSpike;
//    Action obZone1;
//    Action obZone1Back;
//    Action obZone2;
//    Action obZone2Back;
//    Action obZone3;
//    Action pickup1;
//    Action test;
//
//    Action deliver;
//    Action deliver2;
//    Action score1;
//    Action park;
//
//    //Linear Braking
//    double angleBrakePower = 0;
//
//    @Override
//    public void init()
//    {
//        drive = new MecanumDrive(hardwareMap, new Pose2d(-70.35, 0, 0)); //Initialize in init()
//
//        Pose2d pick = new Pose2d(13.20-70.35, -7.3,-129.3);
////        Pose2d deliver = new Pose2d(14.68-70.350, -1.4, Math.PI);
//
//        //Hardware Mapping
//        angleMotor = hardwareMap.get(DcMotorEx.class, "AM");
//        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        backRightMotor = hardwareMap.dcMotor.get("rightBack");
//        slideMotor = hardwareMap.get(DcMotorEx.class, "SM");
//        push = hardwareMap.get(Servo.class, "push");
//        claw = hardwareMap.get(Servo.class, "Claw");
//
//        //Braking
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        //Reversing 2 Wheelbase Motors
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //Encoders
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        //Setting Tolerance
//        angleMotor.setTargetPositionTolerance(10);
//        slideMotor.setTargetPositionTolerance(5);
//
//        //Initializing Servo
//        claw.setPosition(-0.2);//-0.2
//
//        scorePreload = drive.actionBuilder(drive.pose)
//                .setTangent(0)
//                .strafeToConstantHeading(new Vector2d(12 - 70.35, 0))
//                .build();
//
//        test = drive.actionBuilder(new Pose2d(new Vector2d(12 - 70.35, 0), 0))
//                .setTangent(0)
//                .strafeToLinearHeading(new Vector2d(18.6-70.35,39.4),0.02)
//                .stopAndAdd(new pick())
//                .waitSeconds(1)
//                .stopAndAdd(new clawose(claw))
//                .waitSeconds(0.5)
//                .stopAndAdd(new reset())
//                .build();
//        deliver =  drive.actionBuilder(new Pose2d(new Vector2d(18.6-70.35,39.4),0.02))
//                .setTangent(0)
//                .strafeToLinearHeading(new Vector2d(3.3444-70.35,50.1624),Math.toRadians(130.45))
//                .waitSeconds(1)
//////                .stopAndAdd(new nocollide())
////                .waitSeconds(0.2)]
//                //-2273-586
//                .stopAndAdd(new yellow2angreset())
//                .waitSeconds(0.5)
//                .stopAndAdd(new delvslide())
//                .stopAndAdd(new delv())
//                .waitSeconds(1)
//                .stopAndAdd(new delvfin())
//                .waitSeconds(2)
//                .stopAndAdd(new clawy(claw))
//                .waitSeconds(0.8)
//                .stopAndAdd(new yellow2angreset())
//                .stopAndAdd(new resetsl())
//                .waitSeconds(0.4)
//                //pick2
//                .strafeToLinearHeading(new Vector2d(21.45-70.35,48.7),Math.toRadians(0))
//                .stopAndAdd(new pick2())
//                .waitSeconds(2)
//                .stopAndAdd(new clawose(claw))
//                .waitSeconds(1)
//                .stopAndAdd(new reset())
//                .waitSeconds(0.3)
//                //deliver2
//                .strafeToLinearHeading(new Vector2d(3.3444-70.35,50.1624),Math.toRadians(130.45))
//                .waitSeconds(1)
//////                .stopAndAdd(new nocollide())
////                .waitSeconds(0.2)]
//                //-2273-586
//                //start ang- -1358
//                //x=21.45, y=48.7 slide= -770, angle=-403
//                //x-20.8, y=44.2, heading=35.9, slide=-1307, angle=-480
//                .stopAndAdd(new yellow2angreset())
//                .waitSeconds(0.5)
//                .stopAndAdd(new delvslide())
//                .stopAndAdd(new delv())
//                .waitSeconds(1)
//                .stopAndAdd(new delvfin())
//                .waitSeconds(1)
//                .stopAndAdd(new clawy(claw))
//                .waitSeconds(0.8)
//                .stopAndAdd(new yellow2angreset())
//                .stopAndAdd(new resetsl())
//                //pick3
//                .strafeToLinearHeading(new Vector2d(20.8-70.35,44.2),Math.toRadians(35.9))
//                .waitSeconds(1.1)
//                .stopAndAdd(new pick3())
//                .waitSeconds(2)
//                .stopAndAdd(new clawose(claw))
//                .waitSeconds(1.4)
//                .stopAndAdd(new reset())
////                .waitSeconds(0.3)
//                //deliver3
//                .strafeToLinearHeading(new Vector2d(3.3444-70.35,50.1624),Math.toRadians(130.45))
////                .waitSeconds(1)
//////                .stopAndAdd(new nocollide())
////                .waitSeconds(0.2)]
//                //-2273-586
//                //start ang- -1358
//                //x=21.45, y=48.7 slide= -770, angle=-403
//                //x-20.8, y=44.2, heading=35.9, slide=-1307, angle=-480
//                .stopAndAdd(new yellow2angreset())
//                .waitSeconds(0.5)
//                .stopAndAdd(new delvslide())
//                .stopAndAdd(new delv())
//                .waitSeconds(1)
//                .stopAndAdd(new delvfin())
//                .waitSeconds(2)
//                .stopAndAdd(new clawy(claw))
//                .waitSeconds(0.8)
//                .stopAndAdd(new yellow2angreset())
//                .stopAndAdd(new resetsl())
//                .build();
//
////-974+112 ang
//
//        park = drive.actionBuilder(new Pose2d(new Vector2d(3.3444-70.35,50.1624),Math.toRadians(130.45)))
//                .strafeToLinearHeading(new Vector2d(0,20),Math.PI/2)
//                .build();
//    }
//
//    @Override
//    public void start()
//    {
//        Actions.runBlocking
//                (
//                        new SequentialAction
//                                (
//                                        new ParallelAction(scorePreload, new preloadDhruv()),
//                                        test,
//                                        deliver,
//                                        park
//
//
//                                )
//                );
//    }//-117.8
//
//    @Override
//    public void loop()
//    {
//        if (angleMotor.getCurrentPosition() > -500)
//            angleBrakePower = -(0.00005 * slideMotor.getCurrentPosition());
//        else if (angleMotor.getCurrentPosition() < -1500)
//            angleBrakePower = 0.00005 * slideMotor.getCurrentPosition();
//        else
//            angleBrakePower = 0;
//    }
//
//    @Config
//    public class pick implements Action
//    {
//        ElapsedTime timer;
//        int anglet = 870;
//        int slidet = -1092;
//        double slidePower = 0.6;
//        double anglePower = 0.8;
//
//        public pick()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class reset implements Action
//    {
//        ElapsedTime timer;
//        int anglet = 0;
//        int slidet = 0;
//        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public reset()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class resetsl implements Action
//    {
//        ElapsedTime timer;
//        //        int anglet = 0;
//        int slidet = 0;
//        double slidePower = 0.6;
////-160        double anglePower = 0.7;
//
//        public resetsl()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
////            angleMotor.setTargetPosition(anglet);
////            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class parksl implements Action
//    {
//        ElapsedTime timer;
//        //        int anglet = 0;
//        int slidet = -160;
//        double slidePower = 0.6;
////-160        double anglePower = 0.7;
//
//        public parksl()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
////            angleMotor.setTargetPosition(anglet);
////            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class delv implements Action
//    {
//        ElapsedTime timer;
//        int anglet = -1190;
//        //        int slidet = -2301;
//        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public delv()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
////            slideMotor.setTargetPosition(slidet);
////            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class delvslide implements Action
//    {
//        ElapsedTime timer;
//        //        int anglet = -1190;
//        int slidet = -2301;
//        double slidePower = 0.6;
////        double anglePower = 0.7;
//
//        public delvslide()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
////            angleMotor.setTargetPosition(anglet);
////            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class delvfin implements Action
//    {
//        ElapsedTime timer;
//        int anglet = -737+100;
//        int slidet = -2301;
//        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public delvfin()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    //qwertyuiopasdfghjklzxcvbnm 1234567890 !@#$%^&*()
//    public class pick2 implements Action
//    {
//        ElapsedTime timer;
//        int anglet = 1358-403;
//        int slidet = -710;
//        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public pick2()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class pick3pos1 implements Action
//    {
//        ElapsedTime timer;
//        int anglet = 1358-570;
//        int slidet =-1090;
//        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public pick3pos1()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class pick3 implements Action
//    {
//        ElapsedTime timer;
//        int anglet = 1358-480;
//        int slidet =-1090;
//        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public pick3()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
//            slideMotor.setTargetPosition(slidet);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//
//    public class resetang implements Action
//    {
//        ElapsedTime timer;
//        int anglet = -974+112;
//        //        int slidet = -2301;
////        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public resetang()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
////            slideMotor.setTargetPosition(slidet);
////            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class yellow2angreset implements Action
//    {
//        ElapsedTime timer;
//        int anglet = -974+112-500-50;
//        //        int slidet = -2301;
////        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public yellow2angreset()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
////            slideMotor.setTargetPosition(slidet);
////            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class nocollide implements Action
//    {
//        ElapsedTime timer;
//        int anglet = 0;
//        //        int slidet = -2301;
////        double slidePower = 0.6;
//        double anglePower = 0.7;
//
//        public nocollide()
//        {
////            this.AM =angleMotor;
////            this.SM =slideMotor;
//        }
//        //gappy2
//
//
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer == null)
//            {
//                timer = new ElapsedTime();
//            }
//
//            angleMotor.setTargetPosition(anglet);
//            angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            angleMotor.setPower(anglePower);
////            slideMotor.setTargetPosition(slidet);
////            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            slideMotor.setPower(slidePower);
//
//            return false;
//        }
//    }
//    public class pushy implements Action
//    {
//        Servo pushy;
//        ElapsedTime timer;
//
//        public pushy(Servo push)
//        {
//            this.pushy = push;
//        }
//
//        @Override
//        public boolean run (@NonNull TelemetryPacket telemetryPacket)
//        {
//            if (timer==  null)
//            {
//                timer = new ElapsedTime();
//            }
//            pushy.setPosition(0.07);
//            return !(timer.seconds() >= 0.1);
//        }
//    }
//    public class pushyUp implements Action
//    {
//        Servo pushyUp;
//
//        public pushyUp (Servo push)
//        {
//            this.pushyUp = push;
//        }
//
//        @Override
//        public boolean run (@NonNull TelemetryPacket telemetryPacket)
//        {
//            pushyUp.setPosition(1);
//            return false;
//        }
//    }
//
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
//            clawose.setPosition(-0.2);
//            return false;
//        }
//    }
//
//    //    @Config
//    public class preloadDhruv implements Action{
//        ElapsedTime timer;
//        int anglet=44
//                ,slidet=-1827;
//        double slidePower=1,anglePower=0.7;
//
//        public preloadDhruv() {
//
//        }
//        @SuppressLint("SuspiciousIndentation")
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if(timer==  null){timer = new ElapsedTime();}
//            if (timer.seconds()<1.7){
//                angleMotor.setTargetPosition(-130);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(-1);
//                slideMotor.setTargetPosition(slidet);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(slidePower);
////                angleMotor.setPower(anglePower);
//            }
//
//            if (timer.seconds() >= 1.7) {
//                angleMotor.setTargetPosition(anglet);
//                angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                angleMotor.setPower(1);
//                slideMotor.setTargetPosition(0);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(-0.8);
//                if(slideMotor.getCurrentPosition()>-1590) {
//                    claw.setPosition(0.6);
//                }
//            }
//
//            return !(claw.getPosition()==0.6);
//        }
//    }
//}
