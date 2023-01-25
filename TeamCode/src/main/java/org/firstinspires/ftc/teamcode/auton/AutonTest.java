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
    AutonAsync auton;
    Trajectory t0, t1, t2, t3,t4, t5, t1_1, t1_2, t2_1, t2_2, t3_1, t3_2, t4_1, t4_2, t5_1, t5_2,park;
    Gamepad gamepad1;
    double parkingZone;
    int currLift = 0;
    boolean intaking = true;
    int checkNum = -1;
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
        checkNum = -1;
        // int finalID = init.stopAndSave() + 1;
        // if(finalID == 1){finalID = 2;}
        // else if(finalID == 2){finalID = 1;}
        telemetry.addLine(Integer.toString(0));
        telemetry.update();
        auton = new AutonAsync(0, hardwareMap, telemetry, gamepad1);
        auton.robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auton.robot.drive.auton();
        intaking = true;


        currLift = 1;
        t1 = auton.robot.drive.trajectoryBuilder(new Pose2d())

                .lineToLinearHeading(new Pose2d(-25, 1.5, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    checkNum = 1;
                })
                .build();







        auton.robot.drive.followTrajectoryAsync(t1);

        telemetry.addData("external heading velo: ", auton.robot.drive.getExternalHeadingVelocity());

        telemetry.update();
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
    public void fulfillChecks()
    {


        switch(checkNum)
        {
            case 1:

                if(!(auton.robot.distanceSensor.getDistance(DistanceUnit.INCH)< 3))
                {
                    auton.robot.drive.moveTeleOp(0,-0.4,0, 0);
                }
                else {
                    if (timer == null) {
                        timer = new ElapsedTime();
                        timer.reset();
                    }
                    auton.robot.drive.moveTeleOp(0,-0,0, 0);
                    if (timer.milliseconds() <= 1000) {
                        intaking = false;

                    } else
                    {

                        intaking = true;
                        checkNum = -1;
                        auton.robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        t2 = auton.robot.drive.trajectoryBuilder(auton.robot.drive.getPoseEstimate())
                                .addTemporalMarker(3, () -> {checkNum = 2;})
                                .lineToLinearHeading(new Pose2d(-25,0, Math.toRadians(0)))

                                .build();
                        auton.robot.drive.followTrajectoryAsync(t2);
                        timer = null;

                    }


                }
                break;

            case 0:
                auton.robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                auton.robot.drive.moveTeleOp(0,-0,0, 0);
            case 2:
                auton.robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(!(auton.robot.distanceSensor.getDistance(DistanceUnit.INCH)< 3))
                {
                    auton.robot.drive.moveTeleOp(0.55,0,0, 0);
                }
                else {
                    auton.robot.drive.moveTeleOp(0,0,0, 0);
                }
        }

    }
    @Override
    public void loop() {

        telemetry.addData("X: ", auton.robot.drive.getPoseEstimate().getX());
        telemetry.addData("Y: ", auton.robot.drive.getPoseEstimate().getY());
        telemetry.addData("Heading: ", Math.toDegrees(auton.robot.drive.getPoseEstimate().getHeading()));
        telemetry.addData("CHECKNUM: ", checkNum);
        if(checkNum == -1)auton.robot.drive.update();
        else fulfillChecks();
        //auton.robot.drive.getLocalizer().update();
        //liftAsync();
        intake();

        auton.robot.lift.autonRequest();
        telemetry.update();

    }
}