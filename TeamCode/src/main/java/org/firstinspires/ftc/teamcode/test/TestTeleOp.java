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
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="TEST TELEOP")
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DistanceSensor distanceSensor;
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");

        ColorSensor colorSensor;
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();
        while(opModeIsActive())
        {

            telemetry.addData("RED: ", colorSensor.red());
            telemetry.addData("GREEN: ", colorSensor.green());
            telemetry.addData("BLUE: ", colorSensor.blue());
            telemetry.addData("ALPHA: ", colorSensor.alpha());
            telemetry.addData("DIST: ", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
