package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name="rrTest")
public class rrTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        // --- RESET ALL MOTOR POWERS TO 0 --- //
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);

        waitForStart();

        if (isStopRequested()) return;

        //temporary variables
        int fieldCentricTrigger = 0;
        boolean liftToggled = true;
        robot.motorLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLiftRight.setDirection(DcMotor.Direction.REVERSE);
        robot.motorLiftLeft.setDirection(DcMotor.Direction.REVERSE);

        robot.setMotorPowers(0.1,0.1,0.1,0.1);
        TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(new Pose2d())
                .forward(20)
                .build();

        robot.followTrajectorySequence(trajSeq);

    }
}
