package org.firstinspires.ftc.teamcode;
import java.lang.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Auton {
    private boolean direction;
    private int parkingZone;
    public Auton(boolean left, int tag_id) {
        this.direction = left;
        tag_id++;
        if (tag_id == 2) {
            parkingZone = 1;
        } else if (tag_id == 1) {
            parkingZone = 2;
        } else if (tag_id == 3) {
            parkingZone = 3;
        }
    }

    public void runAutonParkOnly(Robot drive, HardwareMap hardwareMap) {
        Servo intake = hardwareMap.get(Servo.class, "intake");

        drive.setMotorPowers(0.1,0.1,0.1,0.1);
        intake.setPosition(0.4);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(25)
                .build();
        drive.followTrajectorySequence(trajSeq);



        if (parkingZone == 1) { // red
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(36)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(36)
                    .build();
            drive.followTrajectory(park);
        }

    }

    public void runAutonLeft(Robot drive, HardwareMap hardwareMap) throws InterruptedException {
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        drive.setMotorPowers(0, 0, 0, 0);
        drive.claw(false);
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(6)
                .forward(6)
                .turn(Math.toRadians(-94))
                .strafeLeft(44)
                .build();
        drive.followTrajectorySequence(trajSeq1);
        goToMediumGoal(liftLeft, liftRight);
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(3)
                .build();
        drive.followTrajectorySequence(trajSeq2);
        drive.claw(true);
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(3)
                .strafeLeft(18)
                .turn(Math.toRadians(-183))
                .build();
        drive.followTrajectorySequence(trajSeq3);
        goToTopOfStack(liftLeft, liftRight);
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(24)
                .build();
        drive.followTrajectorySequence(trajSeq4);
        drive.claw(false);
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(new Pose2d())
                        .waitSeconds(2)
                        .build();
        drive.followTrajectorySequence(trajSeq5);
        goToLowGoal(liftLeft, liftRight);
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(20)
                .turn(Math.toRadians(65))
                .forward(7)
                .build();
        drive.followTrajectorySequence(trajSeq6);
        drive.claw(true);
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(7)
                .turn(Math.toRadians(-65))
                .build();
        drive.followTrajectorySequence(trajSeq7);

        if (parkingZone == 1) { // red
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .forward(20)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .back(20)
                    .build();
            drive.followTrajectory(park);
        } else {
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .back(3)
                    .build();
            drive.followTrajectory(park);
        }
    }

    public void runAutonRight(Robot drive, HardwareMap hardwareMap) throws InterruptedException {
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        drive.setMotorPowers(0, 0, 0, 0);
        drive.claw(false);
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(6)
                .forward(6)
                .turn(Math.toRadians(94))
                .strafeRight(44)
                .build();
        drive.followTrajectorySequence(trajSeq1);
        goToMediumGoal(liftLeft, liftRight);
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(3)
                .build();
        drive.followTrajectorySequence(trajSeq2);
        drive.claw(true);
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(3)
                .strafeRight(18)
                .turn(Math.toRadians(-183))
                .build();
        drive.followTrajectorySequence(trajSeq3);
        goToTopOfStack(liftLeft, liftRight);
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(24)
                .build();
        drive.followTrajectorySequence(trajSeq4);
        drive.claw(false);
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(2)
                .build();
        drive.followTrajectorySequence(trajSeq5);
        goToLowGoal(liftLeft, liftRight);
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(20)
                .turn(Math.toRadians(-65))
                .forward(7)
                .build();
        drive.followTrajectorySequence(trajSeq6);
        drive.claw(true);
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(7)
                .turn(Math.toRadians(65))
                .build();
        drive.followTrajectorySequence(trajSeq7);

        if (parkingZone == 1) { // red
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .back(20)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .forward(20)
                    .build();
            drive.followTrajectory(park);
        } else {
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .forward(3)
                    .build();
            drive.followTrajectory(park);
        }
    }

    public void goToMediumGoal(DcMotorEx left, DcMotorEx right) {
        liftToPosition(left,right, 625, 625);

    }

    public void goToLowGoal(DcMotorEx left, DcMotorEx right) {
        liftToPosition(left,right, 400, 400);

    }

    public void goToTopOfStack(DcMotorEx left, DcMotorEx right) {
        liftToPosition(left,right, 120, 120);

    }
    private static void liftToPosition(DcMotorEx left, DcMotorEx right,int pos_left, int pos_right)
    {
        while ((right.getCurrentPosition() < pos_right-5 || right.getCurrentPosition() >pos_right+5)&&(left.getCurrentPosition() < pos_left-5 || left.getCurrentPosition() >pos_left+5)) {
            right.setTargetPosition(pos_right);
            left.setTargetPosition(pos_left);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setVelocity(600);
            left.setPower(600);
        }
    }
}
