package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auton.Auton;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp
public class AutonAsync extends OpMode
{
    Robot robot;
    int finalID = -1;
    DcMotorEx liftLeft;
    DcMotorEx liftRight;
    TrajectorySequence trajSeq2;
    TrajectorySequence trajSeq3;
    Intake intake;
    int sequenceON = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        liftLeft = robot.lift.leftLift;
        liftRight = robot.lift.rightLift;
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = new Intake(hardwareMap);
        intake.close();
        sequenceON++;

        TrajectorySequence trajSeq1 = robot.drive.trajectorySequenceBuilder(new Pose2d())
            .strafeLeft(49)

            .addDisplacementMarker(() -> {intake.open();})
            .build();

        robot.drive.followTrajectorySequenceAsync(trajSeq1);

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
        robot.drive.update();

        lift_thingies();
        telemetry.addData("STARTED", sequenceON);
        telemetry.update();
    }
}