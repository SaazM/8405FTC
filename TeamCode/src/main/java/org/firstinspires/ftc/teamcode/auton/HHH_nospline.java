package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


@TeleOp
public class HHH_nospline extends OpMode
{
    AutonAsync auton;
    aprilTagsInit init;
    boolean activated = false;
    Trajectory t0, t1, t2, t3,t4, t5, t1_0,  t1_1, t1_2, t2_1, t2_2, t3_1, t3_2, t4_1, t4_2, t5_1, t5_2,park;
    Trajectory st0,st0_0, st1,st1_1, st1_2, st2, st2_1, st3, st4, st4_4, st5_0, st5, st6, st7, st8, st9, st10, st11;
    Gamepad gamepad1;
    double parkingZone = 2.0;
    int currLift = 0;
    boolean intaking = true;

    @Override
    public void init() {
        init = new aprilTagsInit(hardwareMap, telemetry);
        init.initialize();
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
                auton.robot.lift.liftToMedium();
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

        st0_0 = auton.robot.drive.trajectoryBuilder(new Pose2d(0,0, 0)) // move forward to align
                .addDisplacementMarker(() -> currLift = 1)

                .forward(2)
                .addTemporalMarker(2.5, () -> {
                    auton.robot.aligner.outAligner();
                })
                .addTemporalMarker(3.5, () -> auton.robot.aligner.retractAligner())
                .addTemporalMarker(6, () -> {
                    auton.robot.drive.followTrajectoryAsync(st1);
                })
                .build();
        st0 = auton.robot.drive.trajectoryBuilder(st0_0.end()) // move to M pole and drop preload
                .addDisplacementMarker(() -> currLift = 1)
                .addDisplacementMarker(()->auton.robot.aligner.alignAligner())
                .strafeRight(53.8)
                .addTemporalMarker(4, () -> {
//                    auton.robot.aligner.outAligner();
                    auton.robot.drive.followTrajectoryAsync(st0_0);
                })
                .build();
        st0_0 = auton.robot.drive.trajectoryBuilder(st0.end()) // move to M pole and drop preload
                .lineToLinearHeading(new Pose2d(2, -53.9, Math.toRadians(-24)))
                .addTemporalMarker(4, () -> {
//                    auton.robot.aligner.outAligner();
                    auton.robot.drive.followTrajectoryAsync(st1_1);
                })
                .build();

        st1_1 = auton.robot.drive.trajectoryBuilder(st0_0.end()) // align to drop
                .forward(8)
                .addTemporalMarker(2, () ->{
                    auton.robot.aligner.outAligner();
                    intaking = false;
                    auton.robot.drive.followTrajectoryAsync(st1_2);
                })
                .build();

        st1_2 = auton.robot.drive.trajectoryBuilder(st1_1.end()) // end of align to drop
                .addTemporalMarker(2, () ->{
                    auton.robot.aligner.retractAligner();
                })
                .back(14)
                .addTemporalMarker(5, () -> {
//                    auton.robot.aligner.retractAligner();
                    auton.robot.drive.followTrajectoryAsync(st2);
                })
                .build();
//
//
        st2 = auton.robot.drive.trajectoryBuilder(st1_2.end()) // move to cone stack
                .lineToLinearHeading(new Pose2d(st1_2.end().getX(), st1_2.end().getY()-0.01, Math.toRadians(180.01)))
                .addTemporalMarker(0.5,() -> currLift = 2)
                .addTemporalMarker(5, () -> {
                    currLift=2;
                    intaking=true;
                    auton.robot.drive.followTrajectoryAsync(st2_1);
                })
                .build();
//
        st2_1 = auton.robot.drive.trajectoryBuilder(st2.end()) // turn right to cone stack
                .forward(19)
                .addTemporalMarker(3, () -> {
                    auton.robot.drive.followTrajectoryAsync(st3);
                })
                .build();
        st3 = auton.robot.drive.trajectoryBuilder(st2_1.end()) // turn right to cone stack
                .back(19)
                .addTemporalMarker(3, () -> {
                    auton.robot.drive.followTrajectoryAsync(st4);
                })
                .build();

        st4 = auton.robot.drive.trajectoryBuilder(st3.end()) // move to cone stack
                .lineToLinearHeading(new Pose2d(st3.end().getX(), st3.end().getY()+0.01, Math.toRadians(-24)))
                .addTemporalMarker(0.5,() -> currLift = 1)
                .addTemporalMarker(5, () -> {
                    auton.robot.aligner.alignAligner();
                    auton.robot.drive.followTrajectoryAsync(st5);
                })
                .build();
        st5 = auton.robot.drive.trajectoryBuilder(st4.end()) // move to cone stack
                .forward(12)
                .addTemporalMarker(2, () ->{
                    auton.robot.aligner.outAligner();
                    intaking = false;
                    auton.robot.drive.followTrajectoryAsync(st6);
                })
                .build();
        st6 = auton.robot.drive.trajectoryBuilder(st5.end()) // end of align to drop
                .addTemporalMarker(2, () ->{
                    auton.robot.aligner.retractAligner();
                })
                .back(11)
                .addTemporalMarker(5, () -> {
//                    auton.robot.aligner.retractAligner();
                    auton.robot.drive.followTrajectoryAsync(st7);
                })
                .build();
//
//
        st7 = auton.robot.drive.trajectoryBuilder(st6.end()) // move to cone stack
                .lineToLinearHeading(new Pose2d(st6.end().getX(), st6.end().getY()-0.01, Math.toRadians(180.01)))
                .addTemporalMarker(0.5,() -> currLift = 2)
                .addTemporalMarker(5, () -> {
                    currLift=2;
                    intaking=true;
                    auton.robot.drive.followTrajectoryAsync(st8);
                })
                .build();
//
        st8 = auton.robot.drive.trajectoryBuilder(st7.end()) // turn right to cone stack
                .forward(19)
                .addTemporalMarker(3, () -> {
                    auton.robot.drive.followTrajectoryAsync(st9);
                })
                .build();
        st9 = auton.robot.drive.trajectoryBuilder(st8.end()) // turn right to cone stack
                .back(19)
                .addTemporalMarker(3, () -> {
                    auton.robot.drive.followTrajectoryAsync(st10);
                })
                .build();

        st10 = auton.robot.drive.trajectoryBuilder(st9.end()) // move to cone stack
                .lineToLinearHeading(new Pose2d(st9.end().getX(), st9.end().getY()+0.01, Math.toRadians(-24)))
                .addTemporalMarker(0.5,() -> currLift = 1)
                .addTemporalMarker(5, () -> {
                    auton.robot.aligner.alignAligner();
                    auton.robot.drive.followTrajectoryAsync(st11);
                })
                .build();
        st11 = auton.robot.drive.trajectoryBuilder(st10.end()) // move to cone stack
                .forward(15)
                .addTemporalMarker(2, () ->{
                    auton.robot.aligner.outAligner();
                    intaking = false;
                })
                .build();

//
//        st4 = auton.robot.drive.trajectoryBuilder(st2_1.end()) // go to the cone stack
//                .forward(22.5)
//                .addTemporalMarker(4,() -> {currLift=7; auton.robot.drive.followTrajectoryAsync(st4_4);})
//                .build();
//
//        st4_4 = auton.robot.drive.trajectoryBuilder(st4.end()) // go back from cone stack to L goal
//                .back(13)
//                .addTemporalMarker(4,() -> {auton.robot.drive.followTrajectoryAsync(st5_0);})
//                .build();
//        st5_0 = auton.robot.drive.trajectoryBuilder(st4_4.end()) // turn to low goal
//                .lineToLinearHeading(new Pose2d(-7.5, -52, Math.toRadians(90)))
//                .addTemporalMarker(4,() -> {
////                    auton.robot.aligner.outAligner();
//                    auton.robot.drive.followTrajectoryAsync(st5);
//                })
//                .build();
//        st5 = auton.robot.drive.trajectoryBuilder(st5_0.end()) // move forward to low goal and outtake
//                .forward(0.1)
//                .addTemporalMarker(4,() -> {intaking = false; auton.robot.drive.followTrajectoryAsync(st6);})
//                .build();
//        st6 = auton.robot.drive.trajectoryBuilder(st5.end()) // move back from low goal
//                .back(0.1)
//                .addTemporalMarker(4,() -> {
//                    currLift = 2;
////                    auton.robot.aligner.retractAligner();
//                    auton.robot.drive.followTrajectoryAsync(st7);
//                })
//                .build();
//        st7 = auton.robot.drive.trajectoryBuilder(st6.end()) // turn towards cone stack
//                .lineToLinearHeading(new Pose2d(-7.5, -52.01, Math.toRadians(180)))
//                .addTemporalMarker(3, () -> {
//                    intaking=true;
//                    auton.robot.drive.followTrajectoryAsync(st8);
//                })
//                .build();
//        st8 = auton.robot.drive.trajectoryBuilder(st7.end()) // move forward to cone stack
//                .forward(13)
//                .addTemporalMarker(3, () -> {
//                    auton.robot.drive.followTrajectoryAsync(st9);
//                })
//                .build();
//        st9 = auton.robot.drive.trajectoryBuilder(st8.end()) // go back from cone stack
//                .back(25.5)
//                .addTemporalMarker(5,() -> {currLift=6; auton.robot.aligner.alignAligner(); auton.robot.drive.followTrajectoryAsync(st10);})
//                .build();
//        st10 = auton.robot.drive.trajectoryBuilder(st9.end()) // turn to medium goal
//                .lineToLinearHeading(new Pose2d(6.5, -52, Math.toRadians(47)))
//                .addTemporalMarker(3,() -> {
////                    auton.robot.aligner.outAligner();
//                    auton.robot.drive.followTrajectoryAsync(st11);
//                })
//                .build();
//        st11 = auton.robot.drive.trajectoryBuilder(st10.end()) // move forward to medium goal and outtake
//                .forward(8)
//                .addTemporalMarker(3,() -> {auton.robot.aligner.outAligner(); intaking = false; auton.robot.drive.followTrajectoryAsync(st12);})
//                .build();
//        st12 = auton.robot.drive.trajectoryBuilder(st11.end()) // move back from medium goal
//                .addTemporalMarker(2, () ->{
//                    auton.robot.aligner.retractAligner();
//                })
//                .back(8)
//                //.addTemporalMarker(3,() -> {currLift = 2; auton.robot.drive.followTrajectoryAsync(st7);})
//                .addTemporalMarker(3,() -> {
//                    currLift = 2;
////                    auton.robot.aligner.retractAligner();
//                })
//                .build();

        currLift = 1;

//        t1 = auton.robot.drive.trajectoryBuilder(st3.end()) // start from stack and go back
//                .lineToLinearHeading(new Pose2d(0, -57.01, Math.toRadians(180)))
//                .addTemporalMarker(4, () -> {
//                    auton.robot.drive.followTrajectoryAsync(t1_0);
//                })
//                .build();
//
//        t1_0 = auton.robot.drive.trajectoryBuilder(t1.end()) // turn around to face the poles
//
//                .lineToLinearHeading(new Pose2d(0,-57.0, Math.toRadians(0)))
//                .addTemporalMarker(4, () -> {intaking = true; auton.robot.drive.followTrajectoryAsync(park);}) // TWO CONE HERE
//
//                .build();
//
//        t1_1 = auton.robot.drive.trajectoryBuilder(t1_0.end()) // go to the cone stack
//                .addTemporalMarker(0.5, () -> currLift = 6)
//                .lineToLinearHeading(new Pose2d(2, 48, Math.toRadians(0)))
//                .addTemporalMarker(3, () -> auton.robot.drive.followTrajectoryAsync(t2))
//                .build();
//
//        t2 = auton.robot.drive.trajectoryBuilder(t1_1.end()) // come back for cone
//                .forward(37)
//                .addTemporalMarker(4, () -> {
//                    currLift =1;
//                    auton.robot.drive.followTrajectoryAsync(t2_1);})
//                .build();
//
//        t2_1 = auton.robot.drive.trajectoryBuilder(t2.end()) // go to the cone stack
//                .lineToLinearHeading(new Pose2d(12.5, -54, Math.toRadians(185)))
//
//                .addTemporalMarker(4, () -> {
//                    auton.robot.drive.followTrajectoryAsync(t3);
//                })
//                .build();
//
//        t3 = auton.robot.drive.trajectoryBuilder(t2_1.end()) // slide back and score SECOND
//                .lineToLinearHeading(new Pose2d(15,-55.01, Math.toRadians(-90)))
//                .addTemporalMarker(2, () -> {
//                    intaking = false;
//                })
//                .addTemporalMarker(2.5, () -> {
//                    auton.robot.aligner.outAligner();
//                })
//                .addTemporalMarker(3.5, () -> auton.robot.aligner.retractAligner())
//                .addTemporalMarker(4, () -> {intaking = true; auton.robot.drive.followTrajectoryAsync(park);})
//
//                .build();

//        t3_1 = auton.robot.drive.trajectoryBuilder(t3.end()) // go to cone stack
//                .addTemporalMarker(0.5, () -> currLift = 4)
//                .lineToLinearHeading(new Pose2d(-35, 1, Math.toRadians(0)))
//                .addTemporalMarker(2, () -> auton.robot.drive.followTrajectoryAsync(t3_2))
//                .build();
//
//        t3_2 = auton.robot.drive.trajectoryBuilder(t3_1.end()) // slide back and score THIRD
//                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))
//
//                .addTemporalMarker(3, () -> {currLift =1;})
//                .build();
//
//        t4 = auton.robot.drive.trajectoryBuilder(t3_2.end()) // park
//                .addDisplacementMarker(() -> currLift = 1)
//                .lineToLinearHeading(new Pose2d(-37.5, -10, Math.toRadians(-90)))
//                .addTemporalMarker(2, () -> auton.robot.drive.followTrajectoryAsync(park))
//                .build();

        currLift = 1;

//       if (parkingZone == 1) {
//           park = auton.robot.drive.trajectoryBuilder(st4_4.end())
//                .addTemporalMarker(1, () -> currLift = 5)
//                .lineToLinearHeading(new Pose2d(22.5, -54.01, Math.toRadians(0)))
//                .build();
//       } else if (parkingZone == 2) {
//            park = auton.robot.drive.trajectoryBuilder(st4_4.end())
//                .addTemporalMarker(1, () -> currLift = 5)
//                .lineToLinearHeading(new Pose2d(0, -54.01, Math.toRadians(0)))
//                .build();
//       } else {
//            park = auton.robot.drive.trajectoryBuilder(st4_4.end())
//                .addTemporalMarker(1, () -> currLift = 5)
//                .lineToLinearHeading(new Pose2d(-22.5, -54.01, Math.toRadians(0)))
//                .build();
//       }

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