
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
    Trajectory t1, t2, t3,t4, t5, t1_1, t1_2, t2_1, t2_2, t3_1, t3_2, t4_1, t4_2, t5_1, t5_2,park;
    Gamepad gamepad1;
    int currLift = 0;

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
    public void liftAsync()
    {
        switch(currLift)
        {
            case 0:
                break;
            case 1:
                telemetry.addLine("HIGH GOAL");
                auton.robot.lift.liftToHigh();
                break;
            case 2:
                telemetry.addLine("HIGH STACK");
                auton.robot.lift.liftToTopStack();
                break;
            case 3:
                telemetry.addLine("LOW STACK");
                auton.robot.lift.liftToMiddleOfStack();
                break;
        }
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
                .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(-2.5,-54, Math.toRadians(-45)))

                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t1_1))
                .build();

        t1_1 = auton.robot.drive.trajectoryBuilder(t1.end())
                .addDisplacementMarker(() -> currLift = 2)
                .lineToLinearHeading(new Pose2d(-12.5, -50.5, Math.toRadians(180)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t1_2))
                .build();
        t1_2 = auton.robot.drive.trajectoryBuilder(t1_1.end())
                .lineToLinearHeading(new Pose2d(-26, -50.5, Math.toRadians(175)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2))
                .build();


       t2 = auton.robot.drive.trajectoryBuilder(t1_2.end())//2nd cone
               .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(14, -54, Math.toRadians(-90)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2_1))
                .build();
        t2_1 = auton.robot.drive.trajectoryBuilder(t2.end())//2nd cone
                .addDisplacementMarker(() -> currLift = 2)
                .lineToLinearHeading(new Pose2d(-18.5, -50, Math.toRadians(-180)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2_2))
                .build();
        t2_2 = auton.robot.drive.trajectoryBuilder(t2_1.end())//2nd cone
                .lineToLinearHeading(new Pose2d(-26, -50.5, Math.toRadians(-180)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t3))
                .build();

        t3 = auton.robot.drive.trajectoryBuilder(t2_2.end())//2nd cone
                .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(14.5, -54.5, Math.toRadians(-90)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t3_1))
                .build();
        t3_1 = auton.robot.drive.trajectoryBuilder(t3.end())//2nd cone
                .addDisplacementMarker(() -> currLift = 3)
                .lineToLinearHeading(new Pose2d(-18.5, -51, Math.toRadians(-180)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t3_2))
                .build();
        t3_2 = auton.robot.drive.trajectoryBuilder(t3_1.end())//2nd cone
                .lineToLinearHeading(new Pose2d(-26, -51, Math.toRadians(-180)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t4))
                .build();

        t4 = auton.robot.drive.trajectoryBuilder(t3_2.end())//2nd cone
                .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(14.5, -54.5, Math.toRadians(-90)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t4_1))
                .build();
        t4_1 = auton.robot.drive.trajectoryBuilder(t4.end())//2nd cone
                .addDisplacementMarker(() -> currLift = 3)
                .lineToLinearHeading(new Pose2d(-18.5, -51.5, Math.toRadians(-180)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t4_2))
                .build();
        t4_2 = auton.robot.drive.trajectoryBuilder(t4_1.end())//2nd cone
                .lineToLinearHeading(new Pose2d(-26, -51.5, Math.toRadians(-180)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t5))
                .build();
        t5 = auton.robot.drive.trajectoryBuilder(t4_2.end())//2nd cone
                .lineToLinearHeading(new Pose2d(15, -54.5, Math.toRadians(-90)))

                .build();

                //.addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t4))




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
            liftAsync();
            auton.robot.lift.autonRequest();
            //auton.lift_thingies();
            //auton.intaking();

            telemetry.update();
        }
    }
}