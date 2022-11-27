package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.OldRobot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp
public class AutonAsync extends OpMode
{
    OldRobot robot;
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


    public void runAutonAsyncTesting() {
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

    public void runAutonHighSpam(){
        sequenceON = 2;
        Trajectory trajSeq1 = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(75)
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
                    sequenceON = 4;
                    robot.drive.followTrajectorySequenceAsync(waitTrajSeq);
                })
                .build();

        waitTrajSeq  = robot.drive.trajectorySequenceBuilder(trajSeq2.end())
                .waitSeconds(1)
                .strafeRight(13)
                .turn(Math.toRadians(-150))
                .build();

        robot.drive.followTrajectoryAsync(trajSeq1);
    }

    // --- START OF OFFENSIVE AUTON AGAINST GOOD TEAMS --- //
    Trajectory ts1;
    Trajectory ts2;
    Trajectory ts3;
    Trajectory ts4;
    Trajectory ts5;
    Trajectory ts6;
    Trajectory ts7;
    Trajectory ts8;
    public void runAutonTest()
    {
        trajSeq3 = robot.drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(23)
                .addDisplacementMarker(() -> robot.drive.followTrajectorySequence(waitTrajSeq))
                .build();
        waitTrajSeq = robot.drive.trajectorySequenceBuilder(trajSeq3.end())
                .forward(10)
                .build();
        robot.drive.followTrajectorySequenceAsync(trajSeq3);
    }
    public void runAutonOffensive() {
        // RESET



        // TESTING HOW DISPLACEMENT MARKERS INTERACT WITH TRAJSEQS

        // first low goal
        sequenceON = 2;
        Trajectory ts1 = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(23)
                .addDisplacementMarker(() -> {
                    startTime = System.currentTimeMillis();
                    while(!robot.lift.liftReached) {
                        telemetry.addData("waiting ", robot.lift.rightLift.getCurrentPosition());
                        telemetry.addData("liftReached ", robot.lift.liftReached);
                        telemetry.addData("sequence on", sequenceON);
                        telemetry.addData("math", (Math.abs(robot.lift.rightLift.getCurrentPosition() - 380) < 20) || (Math.abs(robot.lift.leftLift.getCurrentPosition() - 380) < 20));
                        telemetry.update();
                    }

                    robot.intake.open();
                    sequenceON = 0;
                    robot.drive.followTrajectoryAsync(ts2);
                })
                .build();


        // get cone from cone stack
        ts2 = robot.drive.trajectoryBuilder(ts1.end())
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
                })
                .build();



        robot.drive.followTrajectoryAsync(ts1);
    }
    // --- END OF OFFENSIVE AUTON AGAINST GOOD TEAMS --- //


    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        robot = new OldRobot(hardwareMap);
        liftLeft = robot.lift.leftLift;
        liftRight = robot.lift.rightLift;
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = new Intake(hardwareMap);
        intake.close();

        runAutonHighSpam();
    }



    public void lift_thingies()
    {
        if(sequenceON == 1) {
            if (System.currentTimeMillis() - startTime <= 10000) {
                robot.lift.liftToMedium();
            }
            else
            {
                robot.lift.liftToPosition(0, 0, 0.3);
                robot.lift.rightLift.setPower(0);
                robot.lift.leftLift.setPower(0);
            }
        } else if (sequenceON == 2) {
//            if (System.currentTimeMillis() - startTime <= 10000) {
            robot.lift.liftToLow();
//            }
//            else {
//                robot.lift.liftToPosition(0, 0);
//                robot.lift.rightLift.setPower(0);
//                robot.lift.leftLift.setPower(0);
//            }
        } else if (sequenceON == 3) {
            if (System.currentTimeMillis() - startTime <= 10000) {
                robot.lift.liftToHigh();
            }
            else {
                robot.lift.liftToPosition(0, 0, 0.3);
                robot.lift.rightLift.setPower(0);
                robot.lift.leftLift.setPower(0);
            }
        } else if (sequenceON == 4) {
            if (System.currentTimeMillis() - startTime <= 10000) {
                //robot.lift.liftToTopStack();
            }
            else {
                robot.lift.liftToPosition(0, 0, 0.3);
                robot.lift.rightLift.setPower(0);
                robot.lift.leftLift.setPower(0);
            }
        }
        else
        {
            robot.lift.liftToPosition(0, 0, 0.3);
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