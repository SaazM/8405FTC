package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp
public class NewAuton extends OpMode
{
    AutonAsync auton;
    boolean activated = false;
    Trajectory t0, t1, t2, t3,t4, t5, t1_0,  t1_1, t1_2, t2_1, t2_2, t3_1, t3_2, t4_1, t4_2, t5_1, t5_2,park;
    Trajectory st0, st1, st2, st3, st4;
    Gamepad gamepad1;
    double parkingZone = 2.0;
    int currLift = 0;
    boolean intaking = true;

    @Override
    public void init() {
    }
    @Override
    public void init_loop()
    {
    }
    public void intakeAsync()
    {
        if(intaking)
        {
            auton.robot.intake.intake();
        }
        else
        {
            auton.robot.intake.outtake();
        }
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

    @Override
    public void start()
    {
        telemetry.update();
        auton = new AutonAsync(0, hardwareMap, telemetry, gamepad1);
        auton.robot.drive.auton();
        auton.robot.intake.intake();
        auton.robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intaking = true;

        st0 = auton.robot.drive.trajectoryBuilder(new Pose2d()) // move to pole
                .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(0,-57.5, Math.toRadians(0)))

//                .addTemporalMarker(4, () -> {
//                    intaking=false;
//                })
//                .addTemporalMarker(4, () -> {
//                    currLift = 5;
//                })

                .addTemporalMarker(5, () -> {
                    auton.robot.drive.followTrajectoryAsync(st1);
                })
                .build();


        st1 = auton.robot.drive.trajectoryBuilder(st0.end()) // SCORE autoloaded                .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(3.5, -57.51, Math.toRadians(-35)))
                .addDisplacementMarker(() -> {
                    intaking=false;
                })

                .addTemporalMarker(4, () -> auton.robot.drive.followTrajectoryAsync(st2))
                .build();


        st2 = auton.robot.drive.trajectoryBuilder(st0.end()) // go back and turn
                .addTemporalMarker(0.5,() -> currLift = 2)
                .lineToLinearHeading(new Pose2d(0, -53, Math.toRadians(185)))
                .addTemporalMarker(3, () -> {
                    currLift=2;
                    intaking=true;
                    auton.robot.drive.followTrajectoryAsync(st3);
                })
                .build();


        st3 = auton.robot.drive.trajectoryBuilder(st2.end()) // go to the cone stack
                .forward(25)
                .addTemporalMarker(3,() -> {auton.robot.drive.followTrajectoryAsync(t1); currLift = 1;})
                .build();

        t1 = auton.robot.drive.trajectoryBuilder(st3.end()) // start from stack and go to pole


                //.lineToLinearHeading(new Pose2d(13.5,-54, Math.toRadians(-85)))
                .lineToLinearHeading(new Pose2d(13.5, -54, Math.toRadians(185)))
//                .addTemporalMarker(3, () -> {
//                    intaking = false;
//                })
                .addTemporalMarker(6, () -> {
                    auton.robot.drive.followTrajectoryAsync(t1_0);
                    //intaking = true;
                })
                .build();

        t1_0 = auton.robot.drive.trajectoryBuilder(t1.end()) // score +1

                .lineToLinearHeading(new Pose2d(13.5,-54.01, Math.toRadians(-90)))
                .addTemporalMarker(2, () -> {
                    intaking = false;
                })
                .addTemporalMarker(2.5, () -> {
                    auton.robot.aligner.outAligner();
                })
                .addTemporalMarker(3.5, () -> auton.robot.aligner.retractAligner())
                .addTemporalMarker(4, () -> {intaking = true; auton.robot.drive.followTrajectoryAsync(t1_1);})

                .build();

        t1_1 = auton.robot.drive.trajectoryBuilder(t1_0.end()) // go to the cone stack
                .addTemporalMarker(0.5, () -> currLift = 2)
                .lineToLinearHeading(new Pose2d(13.5, -54, Math.toRadians(-180)))
                .addTemporalMarker(3, () -> auton.robot.drive.followTrajectoryAsync(t2))
                .build();

        t2 = auton.robot.drive.trajectoryBuilder(t1_1.end()) // come back for cone
                .forward(37)

                .addTemporalMarker(4, () -> {
                    currLift =1;
                    auton.robot.drive.followTrajectoryAsync(t2_1);})
                .build();

        t2_1 = auton.robot.drive.trajectoryBuilder(t2.end()) // go to the cone stack
                .lineToLinearHeading(new Pose2d(13.5, -52, Math.toRadians(185)))

                .addTemporalMarker(4, () -> {
                    auton.robot.drive.followTrajectoryAsync(t3);
                })
                .build();

        t3 = auton.robot.drive.trajectoryBuilder(t2_1.end()) // slide back and score SECOND
                //.addTemporalMarker(15, () -> auton.robot.drive.followTrajectoryAsync(park))
                .lineToLinearHeading(new Pose2d(13.5,-52.01, Math.toRadians(-90)))
                .addTemporalMarker(2, () -> {
                    intaking = false;
                })
                .addTemporalMarker(2.5, () -> {
                    auton.robot.aligner.outAligner();
                })
                .addTemporalMarker(3.5, () -> auton.robot.aligner.retractAligner())
                .addTemporalMarker(4, () -> {intaking = true;}) //auton.robot.drive.followTrajectoryAsync(t3_1);})

                .build();

        t3_1 = auton.robot.drive.trajectoryBuilder(t3.end()) // go to cone stack
                .addTemporalMarker(0.5, () -> currLift = 4)
                .lineToLinearHeading(new Pose2d(-35, 1, Math.toRadians(0)))
                .addTemporalMarker(3, () -> auton.robot.drive.followTrajectoryAsync(t3_2))
                .build();

        t3_2 = auton.robot.drive.trajectoryBuilder(t3_1.end()) // slide back and score THIRD
                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))

                .addTemporalMarker(4, () -> {currLift =1;})
                .build();

        t4 = auton.robot.drive.trajectoryBuilder(t3_2.end()) // park
                .addDisplacementMarker(() -> currLift = 1)
                .lineToLinearHeading(new Pose2d(-37.5, -10, Math.toRadians(-90)))
                .addTemporalMarker(4, () -> auton.robot.drive.followTrajectoryAsync(park))
                .build();

        currLift = 1;

       if (parkingZone == 1) {
           park = auton.robot.drive.trajectoryBuilder(t4.end())
                .addTemporalMarker(1, () -> currLift = 5)
                .lineToLinearHeading(new Pose2d(-13.5, -15, Math.toRadians(-90)))
                .build();
       } else if (parkingZone == 2) {
            park = auton.robot.drive.trajectoryBuilder(t4.end())
                .addTemporalMarker(1, () -> currLift = 5)
                .lineToLinearHeading(new Pose2d(-37.5, -15, Math.toRadians(-90)))
                .build();
       } else {
            park = auton.robot.drive.trajectoryBuilder(t4.end())
                .addTemporalMarker(1, () -> currLift = 5)
                .lineToLinearHeading(new Pose2d(-61.5, -15, Math.toRadians(-90)))
                .build();
       }


        auton.robot.drive.followTrajectoryAsync(st0);
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


            auton.robot.drive.getLocalizer().update();
            auton.robot.drive.update();

            intakeAsync();
            liftAsync();
            auton.robot.lift.autonRequest();
            telemetry.update();
        }
    }
}