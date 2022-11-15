package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class AutonAsync extends OpMode
{

//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//
//        aprilTagsInit init = new aprilTagsInit(hardwareMap, telemetry);
//        init.initialize();
//
//        while (!isStarted() && !isStopRequested())
//        {
//            init.search();
//            sleep(20);
//        }
//        waitForStart();
//        if (isStopRequested()) return;
//        int finalID = init.stopAndSave();
//        telemetry.addLine(Integer.toString(finalID));
//        telemetry.update();
//        Auton auton = new Auton(false, finalID);
//
//        waitForStart();
////        while (opModeIsActive()) {
//        auton.runAutonAsync(new Robot(hardwareMap), hardwareMap);
////        }
//    }

    Robot drive = new Robot(hardwareMap);
    int finalID = -1;
    Auton auton = new Auton(false, finalID);

    @Override
    public void init() {
        Trajectory trajSeq1 = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
        drive.followTrajectoryAsync(trajSeq1);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
    }

    private static void liftToPosition(DcMotorEx left, DcMotorEx right,int pos_left, int pos_right)
    {
        double startTime = System.currentTimeMillis();
        while (((right.getCurrentPosition() < pos_right-10 || right.getCurrentPosition() >pos_right+10)&&(left.getCurrentPosition() < pos_left-10 || left.getCurrentPosition() >pos_left+10))){
            right.setTargetPosition(pos_right);
            left.setTargetPosition(pos_left);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setPower(1);
            left.setPower(1);
            if(System.currentTimeMillis()-startTime>3000){
                double startTime2 = System.currentTimeMillis();
                while(true){
                    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    right.setPower(1);
                    left.setPower(1);

                    if(System.currentTimeMillis()-startTime2>2000) {
                        break;
                    }

                }
                break;
            }
        }
    }

    public void goToMediumGoal(DcMotorEx left, DcMotorEx right) {
        liftToPosition(left,right, 620, 620);
    }

    @Override
    public void loop() {
        drive.update();
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        goToMediumGoal(liftLeft, liftRight);
        telemetry.addData("STARTED", "STARTED");
    }

}