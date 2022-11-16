package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public Servo claw;
    public boolean open;
    public int counter;

    public Intake(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "intake");
        claw.resetDeviceConfigurationForOpMode();
        open = false;
        counter = 0;
    }

    public void open() {
        claw.setPosition(0.4);
    }

    public void close() {
        claw.setPosition(0.9);
    }

    public void moveClaw() {
        if (counter == 0) {
            if (open) {
                claw.setPosition(0.9);
            } else {
                claw.setPosition(0.4);
            }
            open = !open;
        }

        counter++;
    }

    public void resetCounter() {
        counter = 0;
    }
}