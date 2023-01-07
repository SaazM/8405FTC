
package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class RightAuton extends OpMode
{
    aprilTagsInit init;
    AutonAsync auton;
    boolean activated = false;
    Trajectory t1, t2, t3,t4,t5,t6,t7,t8,t9,t10,t11,t12, park;
    Gamepad gamepad1;

    @Override
    public void init() {
        //init = new aprilTagsInit(hardwareMap, telemetry);
        //init.initialize();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void init_loop()
    {
        //init.search();
        //int finalID = init.stopAndSave() + 1;

    }
    @Override
    public void start()
    {
        // int finalID = init.stopAndSave() + 1;
        // if(finalID == 1){finalID = 2;}
        // else if(finalID == 2){finalID = 1;}
        telemetry.addLine(Integer.toString(0));
        telemetry.update();
        auton = new AutonAsync(0, hardwareMap, telemetry, gamepad1);
//        auton.runAutonRight();
        //auton.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //.addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2))
        int parkingZone = 1;


        // change first movement bc tile omegalol

        t1 = auton.robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(-45)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2))
                .build();

        t2 = auton.robot.drive.trajectoryBuilder(t1.end())//1 cone
                .lineToLinearHeading(new Pose2d(12, -48, Math.toRadians(-90)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t3))
                .build();

        t3 = auton.robot.drive.trajectoryBuilder(t2.end())
                .lineToSplineHeading(new Pose2d(-22, -53, Math.toRadians(179.95)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t4))
                .build();

        t4 = auton.robot.drive.trajectoryBuilder(t3.end())//2 cone
                .lineToLinearHeading(new Pose2d(12, -50, Math.toRadians(-90)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t5))
                .build();

        t5 = auton.robot.drive.trajectoryBuilder(t4.end())
                .lineToSplineHeading(new Pose2d(-22, -53, Math.toRadians(179.95)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t6))
                .build();

        t6 = auton.robot.drive.trajectoryBuilder(t5.end())//2 cone
                .lineToLinearHeading(new Pose2d(12, -50, Math.toRadians(-90)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t7))
                .build();

        t7 = auton.robot.drive.trajectoryBuilder(t6.end())
                .lineToSplineHeading(new Pose2d(-22, -53, Math.toRadians(179.95)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t8))
                .build();

        t8 = auton.robot.drive.trajectoryBuilder(t7.end())//2 cone
                .lineToLinearHeading(new Pose2d(12, -50, Math.toRadians(-90)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t9))
                .build();

        t9 = auton.robot.drive.trajectoryBuilder(t8.end())
                .lineToSplineHeading(new Pose2d(-22, -53, Math.toRadians(179.95)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t10))
                .build();

        t10 = auton.robot.drive.trajectoryBuilder(t9.end())//2 cone
                .lineToLinearHeading(new Pose2d(12, -50, Math.toRadians(-90)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t11))
                .build();

        t11 = auton.robot.drive.trajectoryBuilder(t10.end())
                .lineToSplineHeading(new Pose2d(-22, -53, Math.toRadians(179.95)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t12))
                .build();

        t12 = auton.robot.drive.trajectoryBuilder(t11.end())//2 cone
                .lineToLinearHeading(new Pose2d(12, -50, Math.toRadians(-90)))
                .build();
//
//        if  (parkingZone == 1)  {
//            park = auton.robot.drive.trajectoryBuilder(t12.end())
//                    .lineToLinearHeading(new Pose2d(24, -53, Math.toRadians(-90)))
//                    .build();
//        } else if  (parkingZone == 2) {
//            park = auton.robot.drive.trajectoryBuilder(t12.end())
//                    .lineToLinearHeading(new Pose2d(0, -53, Math.toRadians(-90)))
//                    .build();
//        } else  {
//            park = auton.robot.drive.trajectoryBuilder(t12.end())
//                    .lineToLinearHeading(new Pose2d(-24, -53, Math.toRadians(-90)))
//                    .build();
//        }



        auton.robot.drive.followTrajectoryAsync(t1);
        activated = true;
        telemetry.addData("external heading velo: ", auton.robot.drive.getExternalHeadingVelocity());
        telemetry.addData("activated? ", activated);
        telemetry.update();
    }
    @Override
    public void loop() {
        if(activated)
        {
            telemetry.addData("X: ", auton.robot.drive.getPoseEstimate().getX());
            telemetry.addData("Y: ", auton.robot.drive.getPoseEstimate().getY());
            telemetry.addData("Heading: ", auton.robot.drive.getPoseEstimate().getHeading());
            auton.robot.drive.update();
            //auton.lift_thingies();
            //auton.intaking();

            telemetry.update();
        }
    }
}