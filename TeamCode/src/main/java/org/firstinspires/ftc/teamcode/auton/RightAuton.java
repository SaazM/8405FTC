package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class RightAuton extends OpMode
{
    aprilTagsInit init;
    AutonAsync auton;
    boolean activated = false;
    Trajectory t0, t1, t2, t3,t4, t5, t1_0,  t1_1, t1_2, t2_1, t2_2, t3_1, t3_2, t4_1, t4_2, t5_1, t5_2,park;
    Gamepad gamepad1;
    double parkingZone;
    int currLift = 0;
    boolean coneON = false;

    @Override
    public void init() {
        init = new aprilTagsInit(hardwareMap, telemetry);
        init.initialize();
//        auton.robot.drive.odomRetraction.setPosition(2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void init_loop()
    {
        init.search();
        parkingZone = init.stopAndSave();
        telemetry.addData("ZONE: ", parkingZone);
        telemetry.update();
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
            case 4:
                auton.robot.lift.liftToBottomOfStack();
                break;
            case 5:
                auton.robot.lift.currentMode = Lift.LIFT_MODE.RESET;
                break;
            case 6:
                auton.robot.lift.liftToHigh2();
                break;
        }
    }
    public void assertCone()
    {
        if(auton.robot.distanceSensor.getDistance(DistanceUnit.INCH) <= 3)
        {
            coneON = true;
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
        auton.robot.drive.auton();
        auton.robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // SAVED THREE CONE AUTON //
        // USE THIS AUTON PLEASE //
        auton.robot.intake.intake();

        t1 = auton.robot.drive.trajectoryBuilder(new Pose2d()) // SCORE autoloaded

                .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(1,-45, Math.toRadians(0)))

                .addTemporalMarker(3, () -> {
                    auton.robot.drive.followTrajectoryAsync(t1_0);
                })
                .build();
        t1_0 = auton.robot.drive.trajectoryBuilder(t1.end()) // SCORE autoloaded

                .addTemporalMarker(15, () -> auton.robot.drive.followTrajectoryAsync(park))
                .lineToLinearHeading(new Pose2d(9.5,-54.5, Math.toRadians(-54)))
                .addDisplacementMarker(() -> {
                    auton.robot.intake.outtake();
                })
                .addTemporalMarker(3, () -> {
                    currLift = 5;
                })
//                .addTemporalMarker(3, () -> {
//                    auton.robot.drive.followTrajectoryAsync(t1_1);
//                })

                .build();

        t1_1 = auton.robot.drive.trajectoryBuilder(t1.end()) // go back and turn
                .addTemporalMarker(0.5,() -> currLift = 2)
                .lineToLinearHeading(new Pose2d(0, -54, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    currLift=4;
                    auton.robot.intake.intake();
                    auton.robot.drive.followTrajectoryAsync(t1_2);
                })
                .build();

        t1_2 = auton.robot.drive.trajectoryBuilder(t1_1.end()) // go to the cone stack
                .lineToLinearHeading(new Pose2d(-23, -55.5, Math.toRadians(189)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2))
                .build();

        t2 = auton.robot.drive.trajectoryBuilder(t1_2.end()) // slide back and score SECOND
                .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(16, -52.5, Math.toRadians(-80)))
//                .addTemporalMarker(8, () -> {auton.robot.intake.outtake();})
                .addDisplacementMarker(() -> {
                    auton.robot.intake.outtake();
                })
//                .addTemporalMarker(3, () -> auton.robot.drive.followTrajectoryAsync(t2_1))
                .addTemporalMarker(4, () -> currLift=5)

                .build();

        t2_1 = auton.robot.drive.trajectoryBuilder(t2.end()) // go back and turn
                .addTemporalMarker(1, () -> currLift = 3)
                .addTemporalMarker(1, () -> auton.robot.intake.intake())
                .lineToLinearHeading(new Pose2d(-2, -54, Math.toRadians(-180)))
//                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2_2))
//                .addTemporalMarker(3, () -> currLift=5)
                .build();

        t2_2 = auton.robot.drive.trajectoryBuilder(t2_1.end()) // go to cone stack
                .lineToLinearHeading(new Pose2d(-23, -54, Math.toRadians(190)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t3))
                .build();

        t3 = auton.robot.drive.trajectoryBuilder(t2_2.end()) // slide back and score THIRD
                .addDisplacementMarker(() -> currLift = 1)

                .addTemporalMarker(2, () -> {auton.robot.intake.outtake();})
                .lineToLinearHeading(new Pose2d(16, -53, Math.toRadians(-80)))
                .addTemporalMarker(3,() -> auton.robot.drive.followTrajectoryAsync(t3_1))
                .build();
        t3_1 = auton.robot.drive.trajectoryBuilder(t3.end()) // go back and turn
                .addTemporalMarker(1, () -> currLift = 3)
                .addTemporalMarker(1, () -> auton.robot.intake.intake())
                .lineToLinearHeading(new Pose2d(-2, -54, Math.toRadians(-180)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t3_2))
                .build();

        t3_2 = auton.robot.drive.trajectoryBuilder(t3_1.end()) // go to cone stack
                .lineToLinearHeading(new Pose2d(-24.6, -54, Math.toRadians(215)))
                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t4))
                .build();
        t4 = auton.robot.drive.trajectoryBuilder(t3_2.end()) // slide back and score FOURTH
                .addDisplacementMarker(() -> currLift = 1)
                .addTemporalMarker(3, () -> {auton.robot.intake.outtake();})
                .lineToLinearHeading(new Pose2d(15.5, -55, Math.toRadians(-80)))
                .addTemporalMarker(4, () -> auton.robot.drive.followTrajectoryAsync(park))
                .build();

        currLift = 5;



//        auton.robot.intake.intake();
//        t0 = auton.robot.drive.trajectoryBuilder(new Pose2d()) // SCORE autoloaded
//                .addDisplacementMarker(() -> currLift = 1)
//                .lineToLinearHeading(new Pose2d(1,-50, Math.toRadians(0)))
//
//                .addDisplacementMarker( () -> {
//                    auton.robot.drive.followTrajectoryAsync(t1);
//                })
//                .build();
//
//        t1 = auton.robot.drive.trajectoryBuilder(t0.end()) // SCORE autoloaded
//                .addTemporalMarker(3, () -> {
//                    auton.robot.intake.outtake();
//                })
//                .addDisplacementMarker(() -> currLift = 1)
//                .lineToLinearHeading(new Pose2d(5.4,-50, Math.toRadians(-53)))
//
//                .addTemporalMarker(4, () -> {
//                    auton.robot.drive.followTrajectoryAsync(t1_1);
//                })
//                .build();
//
//        t1_1 = auton.robot.drive.trajectoryBuilder(t1.end()) // go back and turn
//                .addTemporalMarker(0.5,() -> currLift = 2)
//                .lineToLinearHeading(new Pose2d(0, -54, Math.toRadians(180)))
//                .addDisplacementMarker(() -> {
//                    auton.robot.intake.intake();
//                    auton.robot.drive.followTrajectoryAsync(t1_2);
//                })
//                .build();
//
//        t1_2 = auton.robot.drive.trajectoryBuilder(t1_1.end()) // go to the cone stack
//                .lineToLinearHeading(new Pose2d(-23, -53, Math.toRadians(180)))
//                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2))
//                .build();
//
//        t2 = auton.robot.drive.trajectoryBuilder(t1_2.end()) // slide back and score SECOND
//                .addDisplacementMarker(() -> currLift = 1)
//                .addTemporalMarker(2, () -> {auton.robot.intake.outtake();})
//                .lineToLinearHeading(new Pose2d(15.5, -52.73, Math.toRadians(-80)))
//                .addTemporalMarker(3, () -> auton.robot.drive.followTrajectoryAsync(t2_1))
//                .build();
//
//        t2_1 = auton.robot.drive.trajectoryBuilder(t2.end()) // go back and turn
//                .addTemporalMarker(1, () -> currLift = 3)
//                .addTemporalMarker(1, () -> auton.robot.intake.intake())
//                .lineToLinearHeading(new Pose2d(-2, -54, Math.toRadians(-180)))
//                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2_2))
//                .build();
//
//        t2_2 = auton.robot.drive.trajectoryBuilder(t2_1.end()) // go to cone stack
//                .lineToLinearHeading(new Pose2d(-23, -54, Math.toRadians(185)))
//                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t3))
//                .build();
//
//        t3 = auton.robot.drive.trajectoryBuilder(t2_2.end()) // slide back and score THIRD
//                .addDisplacementMarker(() -> currLift = 1)
//
//                .addTemporalMarker(2, () -> {auton.robot.intake.outtake();})
//                .lineToLinearHeading(new Pose2d(15.5, -52.5, Math.toRadians(-80)))
//                .addTemporalMarker(3,() -> auton.robot.drive.followTrajectoryAsync(t3_1))
//                .build();
//        t3_1 = auton.robot.drive.trajectoryBuilder(t3.end()) // go back and turn
//                .addTemporalMarker(1, () -> currLift = 3)
//                .addTemporalMarker(1, () -> auton.robot.intake.intake())
//                .lineToLinearHeading(new Pose2d(-2, -54, Math.toRadians(-180)))
//                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t3_2))
//                .build();
//
//        t3_2 = auton.robot.drive.trajectoryBuilder(t3_1.end()) // go to cone stack
//                .lineToLinearHeading(new Pose2d(-24.6, -54, Math.toRadians(185)))
//                .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t4))
//                .build();
//        t4 = auton.robot.drive.trajectoryBuilder(t3_2.end()) // slide back and score FOURTH
//                .addDisplacementMarker(() -> currLift = 1)
//                .addTemporalMarker(3, () -> {auton.robot.intake.outtake();})
//                .lineToLinearHeading(new Pose2d(15, -55, Math.toRadians(-80)))
//                .addTemporalMarker(4, () -> auton.robot.drive.followTrajectoryAsync(park))
//                .build();
//
//        currLift = 5;

       if (parkingZone == 1) {
           park = auton.robot.drive.trajectoryBuilder(t4.end())
                .addTemporalMarker(1, () -> currLift = 5)
                .lineToLinearHeading(new Pose2d(24, -50, Math.toRadians(-90)))
                .build();
       } else if (parkingZone == 2) {
            park = auton.robot.drive.trajectoryBuilder(t4.end())
                .addTemporalMarker(1, () -> currLift = 5)
                .lineToLinearHeading(new Pose2d(0, -50, Math.toRadians(-90)))
                .build();
       } else {
            park = auton.robot.drive.trajectoryBuilder(t4.end())
                .addTemporalMarker(1, () -> currLift = 5)
                .lineToLinearHeading(new Pose2d(-24, -50, Math.toRadians(-90)))
                .build();
       }


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
            telemetry.addData("DISTANCE: ", auton.robot.distanceSensor.getDistance(DistanceUnit.INCH));
            auton.robot.drive.update();
            auton.robot.drive.getLocalizer().update();
            liftAsync();
            auton.robot.lift.autonRequest();
            telemetry.update();
        }
    }
}