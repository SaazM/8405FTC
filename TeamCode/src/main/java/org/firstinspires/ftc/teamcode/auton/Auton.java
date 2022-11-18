package org.firstinspires.ftc.teamcode.auton;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import java.lang.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Auton {
    private boolean direction;
    private int parkingZone;
    private Robot robot;
    public Auton(boolean left, int tag_id, Robot rob) {
        this.direction = left;
        tag_id++;
        if (tag_id == 2) {
            parkingZone = 1;
        } else if (tag_id == 1) {
            parkingZone = 2;
        } else if (tag_id == 3) {
            parkingZone = 3;
        }

        robot = rob;
    }

    public void runAutonParkOnly() {

        robot.intake.close();


        if (parkingZone == 1) { // red
            Trajectory park = robot.drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(36)
                    .build();
            robot.drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = robot.drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(36)
                    .build();
            robot.drive.followTrajectory(park);
        }

    }

    public void runAutonLeft() throws InterruptedException {
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.intake.close();
        //medium goal
        TrajectorySequence trajSeq1 = robot.drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(49)
                .build();
        robot.drive.followTrajectorySequence(trajSeq1);
        goToMediumGoal();
        TrajectorySequence trajSeq2 = robot.drive.trajectorySequenceBuilder(trajSeq1.end())
                .forward(2.5)
                .build();
        robot.drive.followTrajectorySequence(trajSeq2);
        robot.intake.open();
        //get cone from stack
        TrajectorySequence trajSeq3 = robot.drive.trajectorySequenceBuilder(trajSeq2.end())
                .back(2.5)
                .strafeLeft(16)
                .turn(Math.toRadians(-169))
                .build();
        robot.drive.followTrajectorySequence(trajSeq3);
        goToTopOfStack();
        Trajectory trajSeq4 = robot.drive.trajectoryBuilder(trajSeq3.end(), 80.4828434*0.95*0.6, 73.17330064499293*0.6)
                .forward(26.5)
                .build();
        robot.drive.followTrajectory(trajSeq4);
        robot.intake.close();
        TrajectorySequence trajSeq5 = robot.drive.trajectorySequenceBuilder(trajSeq4.end())
                .waitSeconds(1.5)
                .back(1)
                .build();
        robot.drive.followTrajectorySequence(trajSeq5);
        //low goal
        goToLowGoal();
        TrajectorySequence trajSeq6 = robot.drive.trajectorySequenceBuilder(trajSeq5.end())
                .back(20)
                .turn(Math.toRadians(52))
                .forward(7)
                .build();
        robot.drive.followTrajectorySequence(trajSeq6);
        robot.intake.open();
        TrajectorySequence trajSeq7 = robot.drive.trajectorySequenceBuilder(trajSeq6.end())
                .waitSeconds(1)
                .build();
        robot.drive.followTrajectorySequence(trajSeq7);
        TrajectorySequence trajSeq8 = robot.drive.trajectorySequenceBuilder(trajSeq7.end())
                .back(6)
                .turn(Math.toRadians(-55))
                .build();
        robot.drive.followTrajectorySequence(trajSeq8);

        if (parkingZone == 1) { // red
            Trajectory park = robot.drive.trajectoryBuilder(trajSeq8.end())
                    .forward(20)
                    .build();
            robot.drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = robot.drive.trajectoryBuilder(trajSeq8.end())
                    .back(26)
                    .build();
            robot.drive.followTrajectory(park);
        } else {
            Trajectory park = robot.drive.trajectoryBuilder(trajSeq8.end())
                    .back(3)
                    .build();
            robot.drive.followTrajectory(park);
        }
    }

    public void runAutonRight() throws InterruptedException {
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.intake.close();
        TrajectorySequence trajSeq1 = robot.drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(48)
                .build();
        robot.drive.followTrajectorySequence(trajSeq1);
        goToMediumGoal();
        TrajectorySequence trajSeq2 = robot.drive.trajectorySequenceBuilder(trajSeq1.end())
                .forward(2)
                .build();
        robot.drive.followTrajectorySequence(trajSeq2);
        robot.intake.open();
        TrajectorySequence trajSeq3 = robot.drive.trajectorySequenceBuilder(trajSeq2.end())
                .back(2)
                .strafeRight(16)
                .turn(Math.toRadians(184))
                .build();
        robot.drive.followTrajectorySequence(trajSeq3);
        goToTopOfStack();
        Trajectory trajSeq4 = robot.drive.trajectoryBuilder(trajSeq3.end(), 80.4828434*0.95*0.6, 73.17330064499293*0.6)
                .forward(27)
                .build();
        robot.drive.followTrajectory(trajSeq4);
        robot.intake.close();
        TrajectorySequence trajSeq5 = robot.drive.trajectorySequenceBuilder(trajSeq4.end())
                .waitSeconds(1.5)
                .back(1)
                .build();
        robot.drive.followTrajectorySequence(trajSeq5);
        goToLowGoal();
        TrajectorySequence trajSeq6 = robot.drive.trajectorySequenceBuilder(trajSeq5.end())
                .back(20)
                .turn(Math.toRadians(-51.5))
                .forward(4.5)
                .build();
        robot.drive.followTrajectorySequence(trajSeq6);
        robot.intake.open();
        TrajectorySequence trajSeq7 = robot.drive.trajectorySequenceBuilder(trajSeq6.end())
                .waitSeconds(1)
                .build();
        robot.drive.followTrajectorySequence(trajSeq7);
        TrajectorySequence trajSeq8 = robot.drive.trajectorySequenceBuilder(trajSeq7.end())
                .back(6.5)
                .turn(Math.toRadians(48))
                .build();
        robot.drive.followTrajectorySequence(trajSeq8);

        if (parkingZone == 1) { // red
            Trajectory park = robot.drive.trajectoryBuilder(trajSeq8.end())
                    .back(24)
                    .build();
            robot.drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = robot.drive.trajectoryBuilder(trajSeq8.end())
                    .forward(20)
                    .build();
            robot.drive.followTrajectory(park);
        }
    }

    public void goToMediumGoal() {
        robot.lift.liftToPosition(620,620);

    }

    public void goToLowGoal() {
        robot.lift.liftToPosition(440,440);

    }

    public void goToTopOfStack() {
        robot.lift.liftToPosition(150,150);

    }
}