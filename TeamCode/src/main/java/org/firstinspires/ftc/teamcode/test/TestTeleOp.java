package org.firstinspires.ftc.teamcode.test;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="TEST TELEOP")
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "rightLift");

        waitForStart();
        while(opModeIsActive())
        {
            lift.setPower(-0.8);
            telemetry.addData("CURRENT:",lift.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
