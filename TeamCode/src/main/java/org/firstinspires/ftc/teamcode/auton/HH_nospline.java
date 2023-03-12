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
public class HH_nospline extends OpMode
{
    AutonAsync auton;
    aprilTagsInit init;
    boolean activated = false;
    Trajectory t0, t1, t2, t3,t4, t5, t1_0,  t1_1, t1_2, t2_1, t2_2, t3_1, t3_2, t4_1, t4_2, t5_1, t5_2,park;
    Trajectory st00, st0,st0_0, st1,st1_1, st1_2, st2, st2_1, st3, st4, st4_4, st5_0, st5, st6, st7, st8, st9, st10, st11;
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

        st00 = auton.robot.drive.trajectoryBuilder(new Pose2d(0,0, 0)) // move to M pole and drop preload
                .forward(2)
                .addTemporalMarker(0.5, () -> {
                    auton.robot.drive.followTrajectoryAsync(st0);
                })
                .build();

        st0 = auton.robot.drive.trajectoryBuilder(st00.end()) // move to M pole and drop preload
                .addDisplacementMarker(() -> currLift = 1)
                .addDisplacementMarker(()->auton.robot.aligner.alignAligner())
                .strafeRight(53.8)
                .addTemporalMarker(3.5, () -> {
//                    auton.robot.aligner.outAligner();
                    auton.robot.drive.followTrajectoryAsync(st0_0);
                })
                .build();
        st0_0 = auton.robot.drive.trajectoryBuilder(st0.end()) // move to M pole and drop preload
                .lineToLinearHeading(new Pose2d(st0.end().getX(), st0.end().getY()-0.01, Math.toRadians(-24)))
                .addTemporalMarker(4, () -> {
                    auton.robot.drive.followTrajectoryAsync(st1_1);
                })
                .build();

        st1_1 = auton.robot.drive.trajectoryBuilder(st0_0.end()) // align to drop
                .forward(8)
                .addTemporalMarker(1.5, () ->{
                    auton.robot.aligner.outAligner();
                    intaking = false;
                    auton.robot.drive.followTrajectoryAsync(st1_2);
                })
                .build();

        st1_2 = auton.robot.drive.trajectoryBuilder(st1_1.end()) // end of align to drop
                .addTemporalMarker(1, () ->{
                    auton.robot.aligner.retractAligner();
                })
                .back(14)
                .addTemporalMarker(1.5, () -> {
//                    auton.robot.aligner.retractAligner();
                    auton.robot.drive.followTrajectoryAsync(st2);
                })
                .build();
//
//
        st2 = auton.robot.drive.trajectoryBuilder(st1_2.end()) // move to cone stack
                .lineToLinearHeading(new Pose2d(st1_2.end().getX(), st1_2.end().getY()-0.01, Math.toRadians(185.0)))
                .addTemporalMarker(0.5,() -> currLift = 2)
                .addTemporalMarker(3.5, () -> {
                    currLift=2;
                    intaking=true;
                    auton.robot.drive.followTrajectoryAsync(st2_1);
                })
                .build();
//
        st2_1 = auton.robot.drive.trajectoryBuilder(st2.end()) // turn right to cone stack
                .forward(15)
                .addTemporalMarker(2, () -> {
                    currLift = 1;
                    auton.robot.drive.followTrajectoryAsync(st3);
                })
                .build();
        st3 = auton.robot.drive.trajectoryBuilder(st2_1.end()) // turn right to cone stack
                .back(15)
                .addTemporalMarker(2, () -> {
                    auton.robot.drive.followTrajectoryAsync(st4);
                    auton.robot.aligner.alignAligner();
                })
                .build();

        st4 = auton.robot.drive.trajectoryBuilder(st3.end()) // move to cone stack
                .lineToLinearHeading(new Pose2d(st3.end().getX(), st3.end().getY()+0.01, Math.toRadians(-26)))
                .addTemporalMarker(4, () -> {
                    auton.robot.drive.followTrajectoryAsync(st5);
                })
                .build();
        st5 = auton.robot.drive.trajectoryBuilder(st4.end()) // move to cone stack
                .forward(17)
                .addTemporalMarker(1.5, () ->{
                    auton.robot.aligner.outAligner();
                    intaking = false;
                    auton.robot.drive.followTrajectoryAsync(st6);
                })
                .build();
        st6 = auton.robot.drive.trajectoryBuilder(st5.end()) // end of align to drop
                .addTemporalMarker(1, () ->{
                    auton.robot.aligner.retractAligner();
                })
                .back(11)
                .addTemporalMarker(2, () -> {
                    auton.robot.aligner.retractAligner();
                })
                .build();
//
//


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