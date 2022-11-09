package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");

//        drive.setMotorPowers(0, 0, 0, 0);
        //intake.setPosition(0.6);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                // LIFT: Medium Goal

                .strafeLeft(33)

                .build();
        drive.followTrajectorySequence(trajectorySequence);
        goToMediumGoal(liftLeft, liftRight);
        drive.motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.motorLiftLeft.setPower(0);
        drive.motorLiftRight.setPower(0);
        TrajectorySequence trajSeq1  = drive.trajectorySequenceBuilder(trajectorySequence.end())
                .strafeLeft(28)
                .strafeRight(10)
                .build();
        drive.followTrajectorySequence(trajSeq1);
        drive.turn(Math.toRadians(175));
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(trajSeq1.end().plus(new Pose2d(0, 0, Math.toRadians(175))))
                .forward(15)
                .build();
        drive.followTrajectorySequence(traj2);



//                .forward(5)
//                .waitSeconds(1)
//                // Close Claw
//                .addDisplacementMarker(() -> {
//                    intake.setPosition(0);
//                })
//                .waitSeconds(1)
//                .back(5)
//                .waitSeconds(1)
//                // GOTO: Cone Stack
//                .strafeLeft(15)
//                .waitSeconds(1)
//                .turn(Math.toRadians(-160))
//                .waitSeconds(1)
//                .forward(24)
//                .waitSeconds(1)
//
//                // Grab cone from Stack
//                .addDisplacementMarker(() -> {
//                    goToTopOfStack(liftLeft, liftRight);
//                    intake.setPosition(0.6);
//                    goToLowGoal(liftLeft,liftRight);
//                })
//
//                // GOTO: Low goal
//                .back(20)
//                .turn(Math.toRadians(60))
//                .forward(8)
//                .addDisplacementMarker(() -> {
//                    intake.setPosition(0);
//                })
//                .back(8)
//                .turn(Math.toRadians(-60))


        /**
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
         **/
    }

    public void runAutonRight(Robot drive, HardwareMap hardwareMap) {
        Servo intake = hardwareMap.get(Servo.class, "intake");
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");

        drive.setMotorPowers(0, 0, 0, 0);
        intake.setPosition(0.6);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                // go to medium goal
                .addDisplacementMarker(() -> {
                    goToMediumGoal(liftLeft, liftRight);
                })
                .strafeRight(55)
                // raise lift

                .forward(5)
                // close claw
                .addDisplacementMarker(() -> {
                    intake.setPosition(0);
                    goToTopOfStack(liftLeft, liftRight);
                })
                .back(5)
                // go to cone stack
                .strafeRight(15)
                .turn(Math.toRadians(180))
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

    public void goToMediumGoal(DcMotorEx left, DcMotorEx right) {
        liftToPosition(left,right, 400, 400);

    }

    public void goToLowGoal(DcMotorEx left, DcMotorEx right) {
        liftToPosition(left,right, 380, 380);

    }

    public void goToTopOfStack(DcMotorEx left, DcMotorEx right) {
        liftToPosition(left,right, 100, 100);

    }
    private static void liftToPosition(DcMotorEx left, DcMotorEx right,int pos_left, int pos_right)
    {
        while (right.getCurrentPosition() < pos_right) {
            right.setTargetPosition(pos_right);
            left.setTargetPosition(pos_left);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setVelocity(600);
            left.setPower(600);
        }
    }
}
