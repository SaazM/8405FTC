package org.firstinspires.ftc.teamcode.test;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.dex.file.DebugInfoEncoder;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="TEST TELEOP")
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "rightLift");
        DcMotor leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        DcMotor rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
        DcMotor perpEncoder = hardwareMap.get(DcMotor.class, "leftLift");

        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        Odometry odom = new Odometry(leftEncoder, rightEncoder, perpEncoder);
        waitForStart();
        int lStart = leftEncoder.getCurrentPosition();
        int rStart = rightEncoder.getCurrentPosition();
        int pStart = perpEncoder.getCurrentPosition();
        odom.reset();
        while(opModeIsActive())
        {
            odom.runOdom();
            //lift.setPower(-0.8);
            telemetry.addLine(odom.getX() + "");
            telemetry.addLine(odom.getY() + "");
            telemetry.addLine(odom.getHeading() + "");
            telemetry.addLine("" + leftEncoder.getCurrentPosition());
            telemetry.addLine("" + rightEncoder.getCurrentPosition());
            telemetry.addLine("" + perpEncoder.getCurrentPosition());
            telemetry.update();
        }

        /**
        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "leftLift");

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.left_trigger > 0.5)
            {
                lift.setPower(0.3);
            }
            else

            {
                lift.setPower(0);
            }
        }
         **/

    }
}
