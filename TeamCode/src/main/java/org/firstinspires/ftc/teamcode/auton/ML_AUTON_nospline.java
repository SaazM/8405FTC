package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class ML_AUTON_nospline extends OpMode {
    AutonAsync auton;
    aprilTagsInit init;
    boolean activated = false;
    Trajectory t0, t1, t2, t3,t4, t5, t1_0,  t1_1, t1_2, t2_1, t2_2, t3_1, t3_2, t4_1, t4_2, t5_1, t5_2,park;
    Trajectory st00, st0,st0_0, st1,st1_1, st1_0, st1_2, st2, st2_1, st2_2, st1_3, st3, st4, st4_4, st4_5, st5_0, st5, st6, st6_1, st7, st7_1, st8, st9, st10, st11, st12, st13;
    Gamepad gamepad1;
    double parkingZone = 2.0;
    int currLift = 0;
    boolean intaking = true;
    BNO055IMU imu;

    @Override
    public void init() {
        init = new aprilTagsInit(hardwareMap, telemetry);
        init.initialize();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

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
        st00 = auton.robot.drive.trajectoryBuilder(new Pose2d(0,0, 0)) // move to M pole and drop preload
                .forward(1)
                .addTemporalMarker(0.5, () -> {
                    auton.robot.drive.followTrajectoryAsync(st0);
                })
                .build();
        st0 = auton.robot.drive.trajectoryBuilder(st00.end()) // move to M pole and drop preload

                .addDisplacementMarker(() -> currLift = 6)
                .addDisplacementMarker(() -> auton.robot.aligner.alignAligner())
                .strafeRight(42)

                .addTemporalMarker(4, () -> {

                    auton.robot.drive.followTrajectoryAsync(st1_1);
                })
                .build();

        st1_1 = auton.robot.drive.trajectoryBuilder(st0.end()) // align to drop
                .forward(7)
                .addTemporalMarker(1, () -> intaking=false)
                .addTemporalMarker(1.3, () -> auton.robot.aligner.outAligner())
                .addTemporalMarker(1.75, () ->{
                    auton.robot.drive.followTrajectoryAsync(st1_2);
                })
                .build();

        st1_2 = auton.robot.drive.trajectoryBuilder(st1_1.end()) // end of align to drop
                .back(2)
                .addTemporalMarker(0.5, () -> {
                    auton.robot.drive.followTrajectoryAsync(st1_3);
                })
                .build();

        st1_3 = auton.robot.drive.trajectoryBuilder(st1_2.end()) // end of align to drop

                .back(4)
                .addTemporalMarker(1.5, () -> {
                    auton.robot.drive.followTrajectoryAsync(st2);
                })
                .build();

        st2 = auton.robot.drive.trajectoryBuilder(st0.end()) // move to cone stack
                .addTemporalMarker(0.5,() -> currLift = 2)
                .addTemporalMarker(0.5, () ->{
                    auton.robot.aligner.retractAligner();
                })
                .strafeRight(14)
                .addTemporalMarker(2.5, () -> {
                    currLift=2;
                    intaking=true;
                    auton.robot.drive.followTrajectoryAsync(st2_1);
                })
                .build();

        st2_1 = auton.robot.drive.trajectoryBuilder(st2.end()) // turn right to cone stack
                .lineToLinearHeading(new Pose2d(2, -53.01, Math.toRadians(180.1)))
                .addTemporalMarker(4,() -> {auton.robot.drive.followTrajectoryAsync(st4);})
                .build();

        st4 = auton.robot.drive.trajectoryBuilder(st2_1.end()) // go to the cone stack
                .forward(20)
                .addTemporalMarker(2.5,() -> {currLift=7; auton.robot.drive.followTrajectoryAsync(st4_4);})
                .build();


        st4_4 = auton.robot.drive.trajectoryBuilder(st4.end()) // go back from cone stack to L goal
                .back(1)
                .addTemporalMarker(0.5,() -> {currLift=7; auton.robot.drive.followTrajectoryAsync(st4_5);})
                .build();
        st4_5 = auton.robot.drive.trajectoryBuilder(st4_4.end()) // go back from cone stack to L goal
                .back(12.5)
                .addTemporalMarker(1.5,() -> {auton.robot.drive.followTrajectoryAsync(st5_0);})
                .build();

        st5_0 = auton.robot.drive.trajectoryBuilder(st4_5.end()) // turn to low goal
                .lineToLinearHeading(new Pose2d(st4_5.end().getX()-1, st4_5.end().getY()-0.1, Math.toRadians(93)))
                .addTemporalMarker(3,() -> {
                    auton.robot.drive.followTrajectoryAsync(st5);
                })
                .build();
        st5 = auton.robot.drive.trajectoryBuilder(st5_0.end()) // move forward to low goal and outtake
                .forward(2)
                .addTemporalMarker(0.5,() -> {intaking = false;})
                .addTemporalMarker(1.25,()-> auton.robot.drive.followTrajectoryAsync(st6))
                .build();
        st6 = auton.robot.drive.trajectoryBuilder(st5.end()) // move back from low goal
                .back(2)
                .addTemporalMarker(1,() -> {
                    auton.robot.drive.followTrajectoryAsync(st6_1);
                })
                .build();

        st6_1 = auton.robot.drive.trajectoryBuilder(st6.end()) // move back from low goal
                .back(1.5)
                .addTemporalMarker(1,() -> {
                    currLift = 2;
                    auton.robot.drive.followTrajectoryAsync(park);
                })
                .build();
        currLift = 1;

        if (parkingZone == 3) {
            park = auton.robot.drive.trajectoryBuilder(st6_1.end())
                    .addTemporalMarker(1, () -> currLift = 5)
                    .strafeLeft(17)
                    .build();
        } else if (parkingZone == 2) {
            park = auton.robot.drive.trajectoryBuilder(st6_1.end())
                    .addTemporalMarker(1, () -> currLift = 5)
                    .strafeRight(10)
                    .build();
        } else {
            park = auton.robot.drive.trajectoryBuilder(st6_1.end())
                    .addTemporalMarker(1, () -> currLift = 5)
                    .strafeRight(33)
                    .build();
        }



        auton.robot.drive.followTrajectoryAsync(st0);

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
            telemetry.addData("IMU: ", imu.getAngularOrientation().firstAngle);
            auton.robot.drive.update();

            intakeAsync();
            liftAsync();
            auton.robot.lift.autonRequest();
            telemetry.update();
        }
    }
}
