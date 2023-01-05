package org.firstinspires.ftc.teamcode.roadrunnerfiles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="LocTest",group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftLift"));

        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        long rightInitial = rightEncoder.getCurrentPosition();
        long leftInitial = leftEncoder.getCurrentPosition();
        long perpInitial = frontEncoder.getCurrentPosition();
        waitForStart();

        while (!isStopRequested()) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            robot.drive.update();

            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("right encoder: ", rightEncoder.getCurrentPosition() - rightInitial);
            telemetry.addData("left encoder: ", leftEncoder.getCurrentPosition() - leftInitial);
            telemetry.addData("perpendicular: ", frontEncoder.getCurrentPosition() - perpInitial);
            telemetry.addData("right encoder vel: ", rightEncoder.getCorrectedVelocity()/8192.0 * 63/32.0 * Math.PI);
            telemetry.addData("left encoder vel: ", leftEncoder.getCorrectedVelocity()/8192.0 * 63/32.0 * Math.PI);
            telemetry.addData("perpendicular vel: ", frontEncoder.getCorrectedVelocity()/8192.0 * 63/32.0 * Math.PI);
            telemetry.update();
        }
    }
}