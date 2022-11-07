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
        if(tag_id == 2){
            parkingZone = 1;
        }
        if(tag_id == 1)
        {
            parkingZone = 2;
        }
        if(tag_id == 3)
        {
            parkingZone = 3;
        }
        ;//0-->1, 1-->2, 2-// ->3

    }

    public void runAutonParkOnly(Robot drive, HardwareMap hardwareMap)
    {
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
//        DcMotor motorLiftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
//        DcMotor motorLiftRight = hardwareMap.get(DcMotorEx.class, "rightLift");

        drive.setMotorPowers(0, 0, 0, 0);
        //intake.setPosition(0.6);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                //go to medium goal
                .strafeLeft(50)
                .forward(5)
                .addDisplacementMarker(() -> {
                    // lift up to medium goal
                    // claw open
                    //intake.setPosition(0);
                })
                .back(5)
                //go to cone stack
                .strafeLeft(15)
                .turn(Math.toRadians(-180))
                .addDisplacementMarker(() -> {
                    // lift go down to stack
                })
                .forward(24)
                //grab cone from stack
                .addDisplacementMarker(() -> {
                    // claw grab cone
                    //intake.setPosition(0.6);
                    //lift goes up to low goal
                })
                //go to and turn to low goal
                .back(20)
                .turn(Math.toRadians(60))
                .forward(8)
                .addDisplacementMarker(() -> {
                    // claw open
                    //intake.setPosition(0);
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
//        DcMotor motorLiftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
//        DcMotor motorLiftRight = hardwareMap.get(DcMotorEx.class, "rightLift");

        drive.setMotorPowers(0, 0, 0, 0);
        //intake.setPosition(0.6);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                //go to medium goal
                .strafeRight(50)
                .forward(5)
                .addDisplacementMarker(() -> {
                    // lift up to medium goal
                    // claw open
                    //intake.setPosition(0);
                })
                .back(5)
                //go to cone stack
                .strafeRight(15)
                .turn(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    // lift go down to stack
                })
                .forward(24)
                //grab cone from stack
                .addDisplacementMarker(() -> {
                    // claw grab cone
                    //intake.setPosition(0.6);
                    //lift goes up to low goal
                })
                //go to and turn to low goal
                .back(20)
                .turn(Math.toRadians(-60))
                .forward(8)
                .addDisplacementMarker(() -> {
                    // claw open
                    //intake.setPosition(0);
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

    public void linSlidesLift() {
        int n = 4;
        double a = (200 / Math.pow(n, 2));
        int m = 30;

        double x = 0;
        double y = 0;
        double startPosition = 0;//replace with slides.getPosition();
    }

    public void runAutonWithConeNew(Robot drive, HardwareMap hardwareMap)
    {
        Servo intake = hardwareMap.get(Servo.class, "intake");
//        DcMotor motorLiftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
//        DcMotor motorLiftRight = hardwareMap.get(DcMotorEx.class, "rightLift");

        int multiplier = 1;
        if(direction) { multiplier = -1; }

        drive.setMotorPowers(0,0,0,0);
        intake.setPosition(0.6);
        // claw close
        // lift go up

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(27)
                .turn(Math.toRadians(60 * multiplier))
                .forward(10)
                .addDisplacementMarker(() -> {
                    // claw open
                    intake.setPosition(0);
                    // lift go down to height of top cone on stack
                })
                .back(5)
                .turn(Math.toRadians(-60 * multiplier))
                .forward(24)
                .turn(Math.toRadians(-110 * multiplier))
                .forward(27)
                .addDisplacementMarker(() -> {
                    // claw close
                    intake.setPosition(0.6);
                    // lift go up
                })
                .back(5)
                .turn(Math.toRadians(multiplier * -130))
                .forward(8.5)
                .addDisplacementMarker(() -> {
                    // claw open
                    intake.setPosition(0);
                    // lift go down
                })
                .back(8.5)
                .turn(Math.toRadians(multiplier * 160))
                .build();
        drive.followTrajectorySequence(trajectorySequence);

        if (parkingZone == 2) { // red
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .back(24)
                    .build();
            drive.followTrajectory(park);
        } else if (parkingZone == 3) { // blue
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .back(48)
                    .build();
            drive.followTrajectory(park);
        }
    }
}
