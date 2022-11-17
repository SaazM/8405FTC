package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    Trajectory trajSeq2;
    Trajectory trajSeq3;
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

        Trajectory trajSeq1 = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(0.0, 49))
                .addDisplacementMarker(() -> {robot.drive.followTrajectoryAsync(trajSeq2);})
                .build();
        trajSeq2 = robot.drive.trajectoryBuilder(trajSeq1.end())
                .forward(2.489)
                .addDisplacementMarker(() -> {while((robot.lift.rightLift.getCurrentPosition() < 610 || robot.lift.rightLift.getCurrentPosition() > 630)
                    &&robot.lift.leftLift.getCurrentPosition() < 610 || robot.lift.leftLift.getCurrentPosition() > 630){
                    telemetry.addData("waiting", robot.lift.rightLift.getCurrentPosition());
                    telemetry.update();
                    }
                    robot.intake.open();})
                .build();

        robot.drive.followTrajectoryAsync(trajSeq1);


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
            right.setPower(0.5);
            left.setPower(0.5);
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
        liftToPosition(left,right, 620, 620);
    }

    @Override
    public void loop() {
        robot.drive.update();

        lift_thingies();
        telemetry.addData("STARTED", sequenceON);
        telemetry.update();
    }
}