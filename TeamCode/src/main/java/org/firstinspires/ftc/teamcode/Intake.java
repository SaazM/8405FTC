package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class Intake {
    public Servo claw;

    public Intake(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "intake");
    }

    public void open() {
        claw.setPosition(0);
    }

    public void close() {
        claw.setPosition(0.4);
    }
}
