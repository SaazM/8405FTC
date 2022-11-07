package org.firstinspires.ftc.teamcode;

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

    public void runAutonLeft(Robot drive, HardwareMap hardwareMap) {
        Servo intake = hardwareMap.get(Servo.class, "intake");
        DcMotor liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotor liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");

        drive.setMotorPowers(0, 0, 0, 0);
        intake.setPosition(0.6);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                // go to medium goal
                .strafeLeft(50)
                // raise lift
                .addDisplacementMarker(() -> {
                    goToMediumGoal(liftLeft, liftRight);
                })
                .forward(5)
                // close claw
                .addDisplacementMarker(() -> {
                    intake.setPosition(0);
                })
                .back(5)
                // go to cone stack
                .strafeLeft(15)
                .turn(Math.toRadians(-180))
                .addDisplacementMarker(() -> {
                    goToTopOfStack(liftLeft, liftRight);
                })
                .forward(24)
                //grab cone from stack
                .addDisplacementMarker(() -> {
                    intake.setPosition(0.6);
                    goToLowGoal(liftLeft, liftRight);
                })
                //go to and turn to low goal
                .back(20)
                .turn(Math.toRadians(60))
                .forward(8)
                .addDisplacementMarker(() -> {
                    intake.setPosition(0);
                })
                .back(8)
                .turn(Math.toRadians(-60))
                .build();
        drive.followTrajectorySequence(trajectorySequence);

        if (parkingZone == 1) { // red
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .forward(20)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .back(26)
                    .build();
            drive.followTrajectory(park);
        }
    }

    public void runAutonRight(Robot drive, HardwareMap hardwareMap) {
        Servo intake = hardwareMap.get(Servo.class, "intake");
        DcMotor liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotor liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");

        drive.setMotorPowers(0, 0, 0, 0);
        intake.setPosition(0.6);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                // go to medium goal
                .strafeRight(50)
                // raise lift
                .addDisplacementMarker(() -> {
                    goToMediumGoal(liftLeft, liftRight);
                })
                .forward(5)
                // close claw
                .addDisplacementMarker(() -> {
                    intake.setPosition(0);
                })
                .back(5)
                // go to cone stack
                .strafeRight(15)
                .turn(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    goToTopOfStack(liftLeft, liftRight);
                })
                .forward(24)
                //grab cone from stack
                .addDisplacementMarker(() -> {
                    intake.setPosition(0.6);
                    goToLowGoal(liftLeft, liftRight);
                })
                //go to and turn to low goal
                .back(20)
                .turn(Math.toRadians(-60))
                .forward(8)
                .addDisplacementMarker(() -> {
                    intake.setPosition(0);
                })
                .back(8)
                .turn(Math.toRadians(60))
                .build();
        drive.followTrajectorySequence(trajectorySequence);

        if (parkingZone == 1) { // red
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .back(20)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .forward(26)
                    .build();
            drive.followTrajectory(park);
        }
    }

    public void goToMediumGoal(DcMotor left, DcMotor right) {
        left.setTargetPosition(600);
        right.setTargetPosition(600);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void goToLowGoal(DcMotor left, DcMotor right) {
        left.setTargetPosition(380);
        right.setTargetPosition(380);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void goToTopOfStack(DcMotor left, DcMotor right) {
        left.setTargetPosition(100);
        right.setTargetPosition(100);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
