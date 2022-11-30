package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftTest extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        Lift lift = new Lift(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("Left lift encoder: ", lift.leftLift.getCurrentPosition());
            telemetry.addData("Right Lift Encoder: ", lift.rightLift.getCurrentPosition());
            telemetry.update();
        }
    }
}
