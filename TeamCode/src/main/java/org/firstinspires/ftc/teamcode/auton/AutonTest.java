package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp
public class AutonTest extends OpMode
{
    private enum ADJUSTMENT_LEVEL
    {
        FOLLOW_TRAJECTORY, FORWARD, STRAFE, STRAFE_ALPHA, ALPHA_DROP_RETURN, DROP_AND_RETURN, STOP
    }

    AutonAsync auton;
    Trajectory trajectory, t1, t1_0, t1_1, t1_2, t1_3, t1_4;
    Gamepad gamepad1;
    double parkingZone;
    int currLift = 0;
    boolean intaking = true;
    int checkNum = -1;
    int numCones = 0;
    int groundAlpha = 0;
    boolean backwardsFlag = false;
    aprilTagsInit init;

    boolean coneON = false;
    ADJUSTMENT_LEVEL toAdjust = ADJUSTMENT_LEVEL.FOLLOW_TRAJECTORY;
    ElapsedTime timer, coneTimer = null;

    public void assertCone()
    {
        if(auton.robot.distanceSensor.getDistance(DistanceUnit.INCH) <= 3)
        {
            coneON = true;
        }

    }
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        init = new aprilTagsInit(hardwareMap, telemetry);
        init.initialize();
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
    @Override
    public void start()
    {
        auton = new AutonAsync((int) parkingZone, hardwareMap, telemetry, gamepad1);
        auton.robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intaking = true;
        currLift = 1;
//        t1 = auton.robot.drive.trajectoryBuilder(new Pose2d()) // SCORE autoloaded
//
//                //.addDisplacementMarker(() -> currLift = 1)
//                .lineToLinearHeading(new Pose2d(0,30, Math.toRadians(0)))
//
//                .addDisplacementMarker(() -> {
//                    auton.robot.drive.followTrajectoryAsync(t1_1);
//                })
//                .build();
//        t1_1 = auton.robot.drive.trajectoryBuilder(t1.end())
//                .lineToLinearHeading(new Pose2d(19, 30, Math.toRadians(0)))
//                .addDisplacementMarker(() -> {
//                    currLift = 1;
//                    auton.robot.drive.followTrajectoryAsync(t1_2);
//                })
//                .build();
//        t1_2 = auton.robot.drive.trajectoryBuilder(t1_1.end())
//                .lineToLinearHeading(new Pose2d(19, 54.5, Math.toRadians(0)))
//                .addDisplacementMarker(() -> {
//                    auton.robot.drive.followTrajectoryAsync(t1_0);
//                })
//                .build();

        t1_0 = auton.robot.drive.trajectoryBuilder(new Pose2d()) // SCORE autoloaded


                .addTemporalMarker(1, () -> groundAlpha = 40)
                .lineToLinearHeading(new Pose2d(7,-56.5, Math.toRadians(-90)))
                .addTemporalMarker(4, () -> toAdjust = ADJUSTMENT_LEVEL.STRAFE_ALPHA)
                .build();

        auton.robot.drive.followTrajectoryAsync(t1_0);

    }
    public void intake()
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
    private void parseLift()
    {
        if(numCones >= 2)
        {
            currLift = 3;
        }
        else
        {
            currLift = 2;
        }
    }
    public void makeAdjustments()
    {

        switch(toAdjust)
        {
            case STRAFE_ALPHA:

                if(auton.robot.colorSensor.alpha() < groundAlpha+5)
                {
                    auton.robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    auton.robot.drive.moveTeleOp(0,-0.3,0, 0);

                }
                else
                {
                    if(timer == null)
                    {
                        timer = new ElapsedTime();
                        timer.reset();
                    }
                }
                int toAdd = 0;
                if(numCones>=1)
                {
                    toAdd = 200;
                }
                if(timer!=null && timer.milliseconds() >= toAdd){
                    auton.robot.drive.moveTeleOp(0,0,0, 0);
                    toAdjust = ADJUSTMENT_LEVEL.ALPHA_DROP_RETURN;
                    timer = null;
                }
                break;
            case STOP:
                auton.robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                auton.robot.drive.moveTeleOp(0,-0,0, 0);
                break;
            case DROP_AND_RETURN:
                double dropLength = 1;
                double dist = auton.robot.distanceSensor.getDistance(DistanceUnit.INCH);
                double thresh = 0.25;
                if(dist - dropLength > thresh)
                {
                    auton.robot.drive.moveTeleOp(0.25,0,0, 0);
                }
                else if(dist - dropLength < -thresh)
                {
                    auton.robot.drive.moveTeleOp(-0.25, 0, 0, 0);
                }
                else
                {
                    auton.robot.drive.moveTeleOp(0,-0,0, 0);
                    if (timer == null) {
                        timer = new ElapsedTime();
                        timer.reset();
                    }
                    if (timer.milliseconds() <= 1000) {
                        intaking = false;
                    }
                    else
                    {
                        toAdjust = ADJUSTMENT_LEVEL.FOLLOW_TRAJECTORY;
                        intaking = true;
                        auton.robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//                        trajectory = auton.robot.drive.trajectoryBuilder(auton.robot.drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-12,0, Math.toRadians(0)))
//                                .build();
//
//
//                        auton.robot.drive.followTrajectoryAsync(trajectory);
                        timer = null;

                    }
                }
                break;
            case ALPHA_DROP_RETURN:
                if(auton.robot.colorSensor.alpha() >= groundAlpha+4 && !backwardsFlag)
                {
                    auton.robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    auton.robot.drive.moveTeleOp(-0.3, 0 ,0 ,0);
                    if (timer != null && timer.milliseconds() <= 100) {
                        timer.reset();
                    }
                }
                else
                {
                    auton.robot.drive.moveTeleOp(-0, 0 ,0 ,0);
                    if (timer == null) {
                        timer = new ElapsedTime();
                        timer.reset();
                        numCones++;
                    }
                    if (timer.milliseconds() <= 1100 && timer.milliseconds() >= 100) {
                        intaking = false;
                        backwardsFlag = true;
                    }
                    else if(timer.milliseconds() >= 1100)
                    {

                        toAdjust = ADJUSTMENT_LEVEL.FOLLOW_TRAJECTORY;
                        intaking = true;
                        auton.robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        backwardsFlag = false;
                        //auton.robot.drive.followTrajectoryAsync(t1);
                        if(numCones == 3)
                        {
                            if(parkingZone == 1)
                            {
                                trajectory = auton.robot.drive.trajectoryBuilder(auton.robot.drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(30,-55,Math.toRadians(-90)))
                                        .build();
                            }
                            else if(parkingZone == 2)
                            {
                                trajectory = auton.robot.drive.trajectoryBuilder(auton.robot.drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(0,-55,Math.toRadians(-90)))
                                        .build();
                            }
                            else{

                                trajectory = auton.robot.drive.trajectoryBuilder(auton.robot.drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(-18,-55,Math.toRadians(-90)))
                                        .build();

                            }


                        }
                        else
                        {
                            double toPut = -55;
                            if(numCones == 2)
                            {
                                toPut = -56.5;
                            }
                            trajectory = auton.robot.drive.trajectoryBuilder(auton.robot.drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-14
                                            ,toPut,
                                            Math.toRadians(180)))
                                    .addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t1_0))
                                    .addTemporalMarker(0.5, this::parseLift)

                                    .build();

                            t1_0 = auton.robot.drive.trajectoryBuilder(trajectory.end())
                                    .lineToLinearHeading(new Pose2d(-20
                                            ,toPut,
                                            Math.toRadians(180)))
                                    .addTemporalMarker(1, () -> {
                                        currLift = 1;
                                        auton.robot.drive.followTrajectoryAsync(t1);})
                                    .build();
                            t1 = auton.robot.drive.trajectoryBuilder(t1_0.end())

                                    .lineToLinearHeading(new Pose2d(12
                                            ,toPut+1,
                                            Math.toRadians(-90)))
                                    .addDisplacementMarker(() -> toAdjust = ADJUSTMENT_LEVEL.STRAFE_ALPHA)
                                    //.addTemporalMarker(3, () -> auton.robot.drive.followTrajectoryAsync(t1_0))
                                    .build();
                        }



                        auton.robot.drive.followTrajectoryAsync(trajectory);
                        timer = null;

                    }
                }

                break;
            case STRAFE:
                if(!(auton.robot.distanceSensor.getDistance(DistanceUnit.INCH)< 3))
                {
                    auton.robot.drive.moveTeleOp(0,-0.3,0, 0);
                }
                else {
                    auton.robot.drive.moveTeleOp(0,0,0, 0);
                    toAdjust = ADJUSTMENT_LEVEL.DROP_AND_RETURN;
                }
                break;

            case FORWARD:
                auton.robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(!(auton.robot.distanceSensor.getDistance(DistanceUnit.INCH)< 3))
                {
                    auton.robot.drive.moveTeleOp(0.55,0,0, 0);
                }
                else {
                    auton.robot.drive.moveTeleOp(0,0,0, 0);
                }
                break;
        }

    }
    @Override
    public void loop() {

        telemetry.addData("X: ", auton.robot.drive.getPoseEstimate().getX());
        telemetry.addData("Y: ", auton.robot.drive.getPoseEstimate().getY());
        telemetry.addData("Heading: ", Math.toDegrees(auton.robot.drive.getPoseEstimate().getHeading()));
        telemetry.addData("CHECKNUM: ", checkNum);
        telemetry.addData("coneON? ", coneON);
        telemetry.addData("ALPHA ", auton.robot.colorSensor.alpha());
        telemetry.addData("GROUND ALPHA: ", groundAlpha);
        if(toAdjust == ADJUSTMENT_LEVEL.FOLLOW_TRAJECTORY)auton.robot.drive.update();
        else makeAdjustments();
        auton.robot.drive.getLocalizer().update();
        liftAsync();
        intake();

        auton.robot.lift.autonRequest();
        telemetry.update();

    }
}