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
public class MLM_AUTON_yesspline extends OpMode {
    AutonAsync auton;
    aprilTagsInit init;
    boolean activated = false;
    Trajectory st0,st0_0, st1,st1_1, st1_2, st1_3, st2, st2_1, st3, st3_1, st4, st4_4, st5_0, st5, st6, st7, st7_1, st8, st9, st10, st11, st12, st13;
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
            case 7:
                auton.robot.lift.liftToLow();
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
                .forward(2)
                .addTemporalMarker(4, () -> {
                    currLift = 6;
                    auton.robot.aligner.alignAligner();
                    auton.robot.drive.followTrajectoryAsync(st0);
                })
                .build();
        st0 = auton.robot.drive.trajectoryBuilder(st0_0.end()) // move to M pole and drop preload
                .strafeRight(41)
                .addTemporalMarker(4, () -> {
                    auton.robot.drive.followTrajectoryAsync(st1_1);
                })
                .build();

        st1_1 = auton.robot.drive.trajectoryBuilder(st0.end()) // align to drop
                .forward(6)
                .addTemporalMarker(1, () ->{
                    auton.robot.aligner.outAligner();
                    intaking = false;
                    auton.robot.drive.followTrajectoryAsync(st1_2);
                })
                .build();
        st1_2 = auton.robot.drive.trajectoryBuilder(st1_1.end()) // spline to code stack
                .addTemporalMarker(0.5, () ->{
                    auton.robot.aligner.retractAligner();
                    currLift = 2;
                    intaking = true;
                })
                .lineToLinearHeading(new Pose2d(-10, -55, Math.toRadians(181)))
                .addTemporalMarker(6, () -> {
                    auton.robot.drive.followTrajectoryAsync(st1_3);
                })
                .build();
        st1_3 = auton.robot.drive.trajectoryBuilder(st1_2.end()) // move forward to cone stack
                .forward(6)
                .addTemporalMarker(4, () -> {
                    currLift = 7;
                    auton.robot.drive.followTrajectoryAsync(st2);
                })
                .build();
        st2 = auton.robot.drive.trajectoryBuilder(st1_2.end()) // move to low pole
                .lineToLinearHeading(new Pose2d(-4, -51, Math.toRadians(90)))
                .addTemporalMarker(3, () -> {
                    intaking = false;
                })
                .addTemporalMarker(5, () -> {
                    currLift = 2;
                    auton.robot.drive.followTrajectoryAsync(st2_1);
                })
                .build();

        st2_1 = auton.robot.drive.trajectoryBuilder(st2.end()) // move back to cone stack
                .lineToLinearHeading(new Pose2d(-13, -52, Math.toRadians(185)))
                .addTemporalMarker(3, () -> {
                    intaking = true;
                    auton.robot.drive.followTrajectoryAsync(st3);
                })
                .build();

        st3 = auton.robot.drive.trajectoryBuilder(st2_1.end())
                .forward(2)
                .addTemporalMarker(3, () -> {
                    currLift = 6;
                    auton.robot.aligner.alignAligner();
                    auton.robot.drive.followTrajectoryAsync(st3_1);
                })
                .build();

        st3_1 = auton.robot.drive.trajectoryBuilder(st3.end())
                .back(10)
                .addTemporalMarker(3, () -> {
                    auton.robot.drive.followTrajectoryAsync(st4);
                })
                .build();

        st4 = auton.robot.drive.trajectoryBuilder(st3_1.end())
                .lineToLinearHeading(new Pose2d(12, -48, Math.toRadians(50)))
                .addTemporalMarker(4, () -> {
                    auton.robot.aligner.outAligner();
                    intaking = false;
                })
                .build();

        // 2, -41

        auton.robot.drive.followTrajectoryAsync(st0_0);

        activated = true;
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
