package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;
import android.graphics.LinearGradient;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.dex.file.DebugInfoEncoder;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="TEST TELEOP")
public class TestTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);

        robot.lift.newBotStart();

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.square) {
                robot.aligner.retractAligner();
            }
            if(gamepad1.cross) {
                robot.aligner.alignAligner();
            }
            if(gamepad1.circle) {
                robot.aligner.outAligner();
            }

            //robot.aligner.aligner.setPosition(gamepad1.right_stick_y);

            telemetry.addData("Pos: ", robot.aligner.aligner.getPosition());

            telemetry.update();
        }
    }
}
