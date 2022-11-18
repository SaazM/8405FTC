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
    TrajectorySequence trajSeq3;
    Intake intake;
    double startTime;
    int sequenceON = 0;


    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        robot = new Robot(hardwareMap);
        liftLeft = robot.lift.leftLift;
        liftRight = robot.lift.rightLift;
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = new Intake(hardwareMap);
        intake.close();
        sequenceON++;

        Trajectory trajSeq1 = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(49)
                .addDisplacementMarker(() -> {robot.drive.followTrajectoryAsync(trajSeq2);})
                .build();
        trajSeq2 = robot.drive.trajectoryBuilder(trajSeq1.end())
                .forward(2.5)
                .addDisplacementMarker(() -> {
                    startTime = System.currentTimeMillis();
                    while(!robot.lift.liftReached){
                    telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                    telemetry.addData("liftReached ", robot.lift.liftReached);
                    telemetry.update();
                    }
                    robot.intake.open();
                    sequenceON++;
                    robot.drive.followTrajectorySequenceAsync(trajSeq3);})
                .build();

        trajSeq3 = robot.drive.trajectorySequenceBuilder(trajSeq2.end())
                .waitSeconds(5)
                .back(2.5)
                .strafeLeft(16)
                .turn(Math.toRadians(-150))
                .build();

        robot.drive.followTrajectoryAsync(trajSeq1);


    }
    public void lift_thingies()
    {
        if(sequenceON == 1) {
            if (System.currentTimeMillis() - startTime <= 10000)
                robot.lift.goToMediumGoal();
            else
            {
                robot.lift.liftToPosition(0, 0);
                robot.lift.rightLift.setPower(0);
                robot.lift.leftLift.setPower(0);
            }
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

    @Override
    public void loop() {
        robot.drive.update();

        lift_thingies();
        telemetry.addData("STARTED", sequenceON);
        telemetry.update();
    }
}