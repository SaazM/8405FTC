package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public CRServo roller1;
    public CRServo roller2;
    public DigitalChannel limitSwitch;

    public Servo claw;
    public int counter;
    public boolean open;

    public Intake(HardwareMap hardwareMap) {
        roller1 = hardwareMap.get(CRServo.class, "rollerLeft");
        roller1.resetDeviceConfigurationForOpMode();
        roller2 = hardwareMap.get(CRServo.class, "rollerRight");
        roller2.resetDeviceConfigurationForOpMode();
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public Intake(HardwareMap hardwareMap, int i) {
        claw = hardwareMap.get(Servo.class, "intake");
        claw.resetDeviceConfigurationForOpMode();
    }

    public void resetCounter() {
        counter = 0;
    }

    public void outtake() {
        roller1.setDirection(DcMotorSimple.Direction.FORWARD);
        roller2.setDirection(DcMotorSimple.Direction.REVERSE);
        roller1.setPower(1);
        roller2.setPower(1);
    }

    public void intake() {
        if (limitSwitch.getState()) {
            roller1.setDirection(DcMotorSimple.Direction.REVERSE);
            roller2.setDirection(DcMotorSimple.Direction.FORWARD);
            roller1.setPower(1);
            roller2.setPower(1);
        } else {
            roller1.setPower(0);
            roller2.setPower(0);
        }
    }
    public void intake(double i) { //for adjusting speed, we might not necessarily want to be going full speed with the gearbox
        if (limitSwitch.getState()) {
            roller1.setDirection(DcMotorSimple.Direction.REVERSE);
            roller2.setDirection(DcMotorSimple.Direction.FORWARD);
            roller1.setPower(i);
            roller2.setPower(i);
        } else {
            roller1.setPower(0);
            roller2.setPower(0);
        }
    }
}
