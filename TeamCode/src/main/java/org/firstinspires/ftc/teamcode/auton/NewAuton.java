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
        auton.robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intaking = true;

        t1 = auton.robot.drive.trajectoryBuilder(new Pose2d()) // start from stack and go to pole

                .addTemporalMarker(0.4, () -> {
                    auton.robot.aligner.alignAligner();
                })
                .lineToLinearHeading(new Pose2d(-30,0, Math.toRadians(-90)))
                .addTemporalMarker(3, () -> {
                    auton.robot.drive.followTrajectoryAsync(t1_0);
                })
                .build();

        t1_0 = auton.robot.drive.trajectoryBuilder(t1.end()) // score preloaded

                //.addTemporalMarker(15, () -> auton.robot.drive.followTrajectoryAsync(park))
                .lineToLinearHeading(new Pose2d(-37.5,0, Math.toRadians(-90)))
                .addTemporalMarker(2, () -> {
                    intaking = false;
                })
                .addTemporalMarker(2.5, () -> {
                    auton.robot.aligner.outAligner();
                })
                .addTemporalMarker(3.5, () -> auton.robot.aligner.retractAligner())
                .addTemporalMarker(4, () -> intaking = true)
                .build();

        t1_1 = auton.robot.drive.trajectoryBuilder(t1_0.end()) // go to the cone stack
                .addDisplacementMarker(() -> currLift = 2)
                .lineToLinearHeading(new Pose2d(-5, 0, Math.toRadians(0)))
                .build();

        t2 = auton.robot.drive.trajectoryBuilder(t1_1.end()) // slide back and score FIRST
                .lineToLinearHeading(new Pose2d(-37.5,0, Math.toRadians(-90)))
                .addTemporalMarker(2, () -> {
                    intaking = false;
                })
                .addTemporalMarker(2.5, () -> {
                    auton.robot.aligner.outAligner();
                })
                .addTemporalMarker(3.5, () -> auton.robot.aligner.retractAligner())
                .addTemporalMarker(4, () -> intaking = true)
                .build();

        t2_1 = auton.robot.drive.trajectoryBuilder(t2.end()) // go to the cone stack
                .addDisplacementMarker(() -> currLift = 3)
                .lineToLinearHeading(new Pose2d(-5, 0, Math.toRadians(0)))
                .build();

        t3 = auton.robot.drive.trajectoryBuilder(t2_1.end()) // slide back and score SECOND
                .lineToLinearHeading(new Pose2d(-37.5,0, Math.toRadians(-90)))
                .addTemporalMarker(2, () -> {
                    intaking = false;
                })
                .addTemporalMarker(2.5, () -> {
                    auton.robot.aligner.outAligner();
                })
                .addTemporalMarker(3.5, () -> auton.robot.aligner.retractAligner())
                .addTemporalMarker(4, () -> intaking = true)
                .build();

        t3_1 = auton.robot.drive.trajectoryBuilder(t3.end()) // go to cone stack
                .addDisplacementMarker(() -> currLift = 3)
                .lineToLinearHeading(new Pose2d(-5, 0, Math.toRadians(0)))
                .build();

        t3_2 = auton.robot.drive.trajectoryBuilder(t3_1.end()) // slide back and score THIRD
                .lineToLinearHeading(new Pose2d(-37.5,0, Math.toRadians(-90)))
                .addTemporalMarker(2, () -> {
                    intaking = false;
                })
                .addTemporalMarker(2.5, () -> {
                    auton.robot.aligner.outAligner();
                })
                .addTemporalMarker(3.5, () -> auton.robot.aligner.retractAligner())
                .addTemporalMarker(4, () -> intaking = true)
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
            auton.robot.drive.getLocalizer().update();
            intakeAsync();
            liftAsync();
            auton.robot.lift.autonRequest();
            telemetry.update();
        }
    }
}