package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp
public class AutonAsync extends OpMode
{



    Robot drive;
    int finalID = -1;
    Auton auton = new Auton(false, finalID);
    DcMotorEx liftLeft;
    DcMotorEx liftRight;
    TrajectorySequence trajSeq2;
    TrajectorySequence trajSeq3;
    Intake intake;
    int sequenceON = 0;
    @Override
    public void init() {
        drive = new Robot(hardwareMap);
        liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = new Intake(hardwareMap);
        intake.close();
        sequenceON++;
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(49)

                .addDisplacementMarker(() ->{intake.open();})
                .build();






        drive.followTrajectorySequenceAsync(trajSeq1);

    }
    public void lift_thingies()
    {
        if(sequenceON == 1)
        {
            goToMediumGoal(liftLeft, liftRight);
        }
        else
        {
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }
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

        if(((right.getCurrentPosition() < pos_right-10 || right.getCurrentPosition() >pos_right+10)&&(left.getCurrentPosition() < pos_left-10 || left.getCurrentPosition() >pos_left+10))){
            right.setTargetPosition(pos_right);
            left.setTargetPosition(pos_left);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setPower(1);
            left.setPower(1);

        }
        else
        {
            right.setTargetPosition(right.getCurrentPosition());
            left.setTargetPosition(left.getCurrentPosition());
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setPower(1);
            left.setPower(1);
        }
    }



    public void goToMediumGoal(DcMotorEx left, DcMotorEx right) {
        liftToPosition(left,right, 400, 400);
    }

    @Override
    public void loop() {

        drive.update();

        lift_thingies();
        telemetry.addData("STARTED", sequenceON);
        telemetry.update();

    }

}