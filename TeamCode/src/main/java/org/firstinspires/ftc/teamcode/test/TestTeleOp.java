package org.firstinspires.ftc.teamcode.test;

import android.graphics.LinearGradient;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        waitForStart();
        while(opModeIsActive())
        {

            //lift.setPower(-0.8);

            telemetry.addData("FIRST: ", imu.getAngularOrientation().firstAngle);
            telemetry.addData("SECOND: ", imu.getAngularOrientation().secondAngle);
            telemetry.addData("THIRD: ", imu.getAngularOrientation().thirdAngle);

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
