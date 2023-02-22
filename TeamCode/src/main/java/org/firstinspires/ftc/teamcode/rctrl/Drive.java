package org.firstinspires.ftc.teamcode.rctrl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.Arrays;
import java.util.List;

@Config
public class Drive {
    public BNO055IMU imu;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> motors;

    public Drive(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftFront.setDirection(DcMotor.Direction.REVERSE); // motor direction
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // We don't want to use PID for the motors using the encoders

        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    public void move(double power, double strafe, double turn) {
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        setDrivePowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    public void setDrivePowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}
