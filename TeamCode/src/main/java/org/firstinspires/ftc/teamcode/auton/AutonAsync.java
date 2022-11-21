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
    Intake intake;
    double startTime;

    int sequenceON = 0;
    // sequenceON key:
    // 1 = medium
    // 2 = low
    // 3 = high
    // 4 = liftToTopStack
    // 0 = reset

    Trajectory trajSeq1;
    Trajectory trajSeq2;
    TrajectorySequence trajSeq3;
    TrajectorySequence waitTrajSeq;


    public void runAutonAsyncTesting() throws InterruptedException {
        Trajectory trajSeq1 = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(49)
                .addDisplacementMarker(() -> {robot.drive.followTrajectoryAsync(trajSeq2);})
                .build();
        Trajectory trajSeq2 = robot.drive.trajectoryBuilder(trajSeq1.end())
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
                    robot.drive.followTrajectorySequenceAsync(waitTrajSeq);
                })
                .build();

        waitTrajSeq  = robot.drive.trajectorySequenceBuilder(trajSeq2.end())
                .waitSeconds(5)
                .back(2.5)
                .strafeLeft(16)
                .turn(Math.toRadians(-150))
                .build();

        robot.drive.followTrajectoryAsync(trajSeq1);
    }

    public void runAutonLeftHigh() {

        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.intake.close();
        //strafe to high
        trajSeq1 = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(63)
                .addDisplacementMarker(() -> {
                    robot.drive.followTrajectoryAsync(trajSeq2);
                })
                .build();
        Trajectory trajSeq2 = robot.drive.trajectoryBuilder(trajSeq1.end())
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
                    robot.drive.followTrajectorySequenceAsync(trajSeq3);
                })
                .build();
        TrajectorySequence trajSeq3 = robot.drive.trajectorySequenceBuilder(trajSeq2.end())
                .back(2.5)
                .strafeRight(16)
                .turn(Math.toRadians(150))
                .build();

        robot.drive.followTrajectoryAsync(trajSeq1);
    }

    // --- START OF OFFENSIVE AUTON AGAINST GOOD TEAMS --- //
    TrajectorySequence ts1;
    TrajectorySequence ts2;
    TrajectorySequence ts3;
    TrajectorySequence ts4;
    TrajectorySequence ts5;
    TrajectorySequence ts6;
    TrajectorySequence ts7;
    TrajectorySequence ts8;

    public void runAutonOffensive() {
        // RESET
        robot.drive.setMotorPowers(0, 0, 0, 0);
        robot.intake.close();

        // TESTING HOW DISPLACEMENT MARKERS INTERACT WITH TRAJSEQS

        // first low goal
        TrajectorySequence ts1 = robot.drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(23)
                .addDisplacementMarker(() -> {
                    startTime = System.currentTimeMillis();
                    while(!robot.lift.liftReached) {
                        telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                        telemetry.addData("liftReached ", robot.lift.liftReached);
                        telemetry.update();
                    }
                    sequenceON = 2;
                    robot.intake.open();
                    robot.drive.followTrajectorySequenceAsync(ts2);
                    sequenceON = 0;
                })
                .build();


        // get cone from cone stack
        TrajectorySequence ts2 = robot.drive.trajectorySequenceBuilder(ts1.end())
                .waitSeconds(5)
                .strafeLeft(40)
                .turn(Math.toRadians((-160)))
                .forward(40)
                .addDisplacementMarker(() -> {
                    startTime = System.currentTimeMillis();
                    while(!robot.lift.liftReached) {
                        telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                        telemetry.addData("liftReached ", robot.lift.liftReached);
                        telemetry.update();
                    }
                    sequenceON = 4;
                    robot.intake.close();
                    robot.drive.followTrajectorySequenceAsync(ts3);
                })
                .build();

        // put cone on other low goal
        TrajectorySequence ts3 = robot.drive.trajectorySequenceBuilder(ts2.end())
                .waitSeconds(3)
                .back(24)
                .turn(Math.toRadians(-90))
                .forward(5)
                .addDisplacementMarker(() -> {
                    startTime = System.currentTimeMillis();
                    while(!robot.lift.liftReached) {
                        telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                        telemetry.addData("liftReached ", robot.lift.liftReached);
                        telemetry.update();
                    }
                    sequenceON = 2;
                    robot.intake.open();
                    sequenceON = 0;
                    robot.drive.followTrajectorySequenceAsync(ts4);
                })
                .build();

        // go back to cone stack
       TrajectorySequence ts4 = robot.drive.trajectorySequenceBuilder(ts3.end())
               .waitSeconds(3)
               .back(10)
               .turn(Math.toRadians(90))
               .forward(24)
               .addDisplacementMarker(() -> {
                   startTime = System.currentTimeMillis();
                   while(!robot.lift.liftReached) {
                       telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                       telemetry.addData("liftReached ", robot.lift.liftReached);
                       telemetry.update();
                   }
                   sequenceON = 4;
                   robot.intake.close();
                   robot.drive.followTrajectorySequenceAsync(ts5);
               })

               .build();

        // score on medium goal
        TrajectorySequence ts5 = robot.drive.trajectorySequenceBuilder(ts4.end())
                .waitSeconds(3)
                .back(50)
                .turn(Math.toRadians(-90))
                .forward(5)
                .addDisplacementMarker(() -> {
                    startTime = System.currentTimeMillis();
                    while(!robot.lift.liftReached) {
                        telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                        telemetry.addData("liftReached ", robot.lift.liftReached);
                        telemetry.update();
                    }
                    sequenceON = 3;
                    robot.intake.open();
                    robot.drive.followTrajectorySequenceAsync(ts6);
                })
                .build();

        // go back to cone stack
        TrajectorySequence ts6 = robot.drive.trajectorySequenceBuilder(ts5.end())
                .waitSeconds(3)
                .back(10)
                .turn(Math.toRadians(90))
                .forward(50)
                .addDisplacementMarker(() -> {
                    startTime = System.currentTimeMillis();
                    while(!robot.lift.liftReached) {
                        telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                        telemetry.addData("liftReached ", robot.lift.liftReached);
                        telemetry.update();
                    }
                    sequenceON = 4;
                    robot.intake.close();
                    robot.drive.followTrajectorySequenceAsync(ts7);
                })
                .build();

        // score on high goal
        TrajectorySequence ts7 = robot.drive.trajectorySequenceBuilder(ts6.end())
                .waitSeconds(3)
                .back(10)
                .turn(Math.toRadians(180))
                .forward(65)
                .turn(Math.toRadians(-90))
                .forward(10)
                .addDisplacementMarker(() -> {
                    startTime = System.currentTimeMillis();
                    while(!robot.lift.liftReached) {
                        telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                        telemetry.addData("liftReached ", robot.lift.liftReached);
                        telemetry.update();
                    }
                    sequenceON = 3;
                    robot.intake.open();
                    sequenceON = 0;
                })

                .build();

        robot.drive.followTrajectorySequenceAsync(ts1);
    }
    // --- END OF OFFENSIVE AUTON AGAINST GOOD TEAMS --- //


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

        runAutonLeftHigh();
    }



    public void lift_thingies()
    {
        if(sequenceON == 1) {
            if (System.currentTimeMillis() - startTime <= 10000) {
                robot.lift.goToMediumGoal();
            }
            else
            {
                robot.lift.liftToPosition(0, 0);
                robot.lift.rightLift.setPower(0);
                robot.lift.leftLift.setPower(0);
            }
        } else if (sequenceON == 2) {
            if (System.currentTimeMillis() - startTime <= 10000) {
                robot.lift.goToLowGoal();
            }
            else {
                robot.lift.liftToPosition(0, 0);
                robot.lift.rightLift.setPower(0);
                robot.lift.leftLift.setPower(0);
            }
        } else if (sequenceON == 3) {
            if (System.currentTimeMillis() - startTime <= 10000) {
                robot.lift.goToHighGoal();
            }
            else {
                robot.lift.liftToPosition(0, 0);
                robot.lift.rightLift.setPower(0);
                robot.lift.leftLift.setPower(0);
            }
        } else if (sequenceON == 4) {
            if (System.currentTimeMillis() - startTime <= 10000) {
                robot.lift.liftToTopStack();
            }
            else {
                robot.lift.liftToPosition(0, 0);
                robot.lift.rightLift.setPower(0);
                robot.lift.leftLift.setPower(0);
            }
        }
        else
        {
            robot.lift.liftToPosition(0, 0);
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