package org.firstinspires.ftc.teamcode.roadrunnerfiles.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.turn(Math.toRadians(ANGLE));
    }
}