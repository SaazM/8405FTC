package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class ParkAuton extends LinearOpMode
{

    @Override
    public void runOpMode()
    {

        aprilTagsInit init = new aprilTagsInit(hardwareMap, telemetry);
        init.initialize();

        while (!isStarted() && !isStopRequested())
        {
            init.search();
            sleep(20);
        }
        waitForStart();
        int finalID = init.stopAndSave();
        telemetry.addLine(Integer.toString(finalID));
        telemetry.update();

        Robot robot = new Robot(hardwareMap, gamepad1);
        robot.intake.intake();
        TrajectorySequence park;
        robot.drive.odomRetraction.setPosition(2);

        if (finalID == 1) {
            park = robot.drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeRight(30)
                    .forward(24)
                    .build();
        } else if (finalID == 2) {
            park = robot.drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeRight(30)
                    .build();
        } else {
            park = robot.drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeRight(30)
                    .back(24)
                    .build();
        }

        robot.drive.followTrajectorySequence(park);
    }

}