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
public class HHH_spline extends OpMode
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

                .addTemporalMarker(0.5, () -> {
                    auton.robot.drive.followTrajectoryAsync(st0);
                })
                .build();
        st0 = auton.robot.drive.trajectoryBuilder(new Pose2d(0,0, 0)) // move to H pole and drop preload
                .addDisplacementMarker(() -> currLift = 1)
                .addDisplacementMarker(() -> auton.robot.aligner.alignAligner())

                .lineToLinearHeading(new Pose2d(2,-51.5, Math.toRadians(-40)))

                .addTemporalMarker(5, () -> {
                    auton.robot.drive.followTrajectoryAsync(st1);
                })
                .build();
        st1 = auton.robot.drive.trajectoryBuilder(st0.end()) // move to H pole and drop preload
                .forward(12)
                .addTemporalMarker(1.25, () -> {intaking=false;auton.robot.aligner.outAligner();})
                .addTemporalMarker(2,() -> {


                    auton.robot.drive.followTrajectoryAsync(st2);
                })

//                .addTemporalMarker(3, () -> auton.robot.drive.followTrajectoryAsync(st2))
                .build();
        st2 = auton.robot.drive.trajectoryBuilder(st1.end()) // go back and turn
                .addTemporalMarker(0.5,() -> currLift = 2)
                .lineToLinearHeading(new Pose2d(0, -50.5, Math.toRadians(180)))
                .addTemporalMarker(3, () -> {
                    auton.robot.aligner.retractAligner();
                    currLift=2;
                    intaking=true;
                    auton.robot.drive.followTrajectoryAsync(st2_1);
                })
                .build();


        st2_1 = auton.robot.drive.trajectoryBuilder(st2.end()) // go to the cone stack
                .forward(20)
                .addTemporalMarker(1.5,() -> { currLift = 1; auton.robot.drive.followTrajectoryAsync(st3);})
                .build();


        st3 = auton.robot.drive.trajectoryBuilder(st2_1.end()) // turn right to cone stack
                .lineToLinearHeading(new Pose2d(4,-51.5, Math.toRadians(-35)))
                .addTemporalMarker(1, () -> auton.robot.aligner.alignAligner())
                .addTemporalMarker(3, () -> {

                    auton.robot.drive.followTrajectoryAsync(st4);
                })
                .build();

        st4 = auton.robot.drive.trajectoryBuilder(st3.end()) // move to cone stack
                .forward(10)
                .addTemporalMarker(1.25, () -> {intaking=false; auton.robot.aligner.outAligner();})
                .addTemporalMarker(1.75,() -> {


                    auton.robot.drive.followTrajectoryAsync(st5);
                })
                .build();
        st5 = auton.robot.drive.trajectoryBuilder(st4.end()) // move to cone stack
                .addTemporalMarker(0.5,() -> currLift = 3)
                .lineToLinearHeading(new Pose2d(0, -52.01, Math.toRadians(180)))
                .addTemporalMarker(3, () -> {
                    currLift=2;
                    auton.robot.aligner.retractAligner();
                    intaking=true;
                    auton.robot.drive.followTrajectoryAsync(st6);
                })

                .build();
        st6 = auton.robot.drive.trajectoryBuilder(st5.end()) // end of align to drop
                .forward(20)
                .addTemporalMarker(1.5,() -> { currLift = 1; auton.robot.drive.followTrajectoryAsync(st8);})
                .build();
//
//
        st8 = auton.robot.drive.trajectoryBuilder(st6.end()) // turn right to cone stack
                .lineToLinearHeading(new Pose2d(5,-51.5, Math.toRadians(-35)))
                .addTemporalMarker(1, () -> auton.robot.aligner.alignAligner())
                .addTemporalMarker(3, () -> {
                    auton.robot.drive.followTrajectoryAsync(st9);
                })
                .build();
        st9 = auton.robot.drive.trajectoryBuilder(st8.end()) // turn right to cone stack
                .forward(10)
                .addTemporalMarker(1, () -> {intaking=false; auton.robot.aligner.outAligner();})
                .addTemporalMarker(1.75,() -> {


                    auton.robot.drive.followTrajectoryAsync(park);
                })
                .build();


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