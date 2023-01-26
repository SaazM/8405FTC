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
        FOLLOW_TRAJECTORY, FORWARD, STRAFE, DROP_AND_RETURN, STOP
    }

    AutonAsync auton;
    Trajectory trajectory;
    Gamepad gamepad1;
    double parkingZone;
    int currLift = 0;
    boolean intaking = true;
    int checkNum = -1;
    ADJUSTMENT_LEVEL toAdjust = ADJUSTMENT_LEVEL.FOLLOW_TRAJECTORY;
    ElapsedTime timer = null;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    @Override
    public void init_loop()
    {

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
        }
    }
    @Override
    public void start()
    {
        auton = new AutonAsync(0, hardwareMap, telemetry, gamepad1);
        auton.robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auton.robot.drive.auton();
        intaking = true;
        currLift = 1;
        trajectory = auton.robot.drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-25, 1.5, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    toAdjust = ADJUSTMENT_LEVEL.STRAFE;
                })
                .build();
        auton.robot.drive.followTrajectoryAsync(trajectory);

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
    public void makeAdjustments()
    {

        switch(toAdjust)
        {
            case STOP:
                auton.robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                        trajectory = auton.robot.drive.trajectoryBuilder(auton.robot.drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-12,0, Math.toRadians(0)))
                                .build();
                        auton.robot.drive.followTrajectoryAsync(trajectory);
                        timer = null;

                    }
                }
                break;
            case STRAFE:
                if(!(auton.robot.distanceSensor.getDistance(DistanceUnit.INCH)< 3))
                {
                    auton.robot.drive.moveTeleOp(0,-0.4,0, 0);
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
        if(toAdjust == ADJUSTMENT_LEVEL.FOLLOW_TRAJECTORY)auton.robot.drive.update();
        else makeAdjustments();
        auton.robot.drive.getLocalizer().update();
        liftAsync();
        intake();

        auton.robot.lift.autonRequest();
        telemetry.update();

    }
}