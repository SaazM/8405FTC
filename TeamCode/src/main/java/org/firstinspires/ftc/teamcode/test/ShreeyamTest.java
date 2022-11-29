package org.firstinspires.ftc.teamcode.test;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="ShreeyamTest")

public class ShreeyamTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        while(opModeIsActive()) {
            DcMotor motor = hardwareMap.get(DcMotor.class, "shreeyam");
            motor.setPower(-0.4);
        }
    }
}
