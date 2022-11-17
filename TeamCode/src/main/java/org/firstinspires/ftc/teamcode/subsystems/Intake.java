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
        claw.setPosition(0.3);
    }

    public void close() {
        claw.setPosition(1);
    }

    public void moveClaw() {
        if (counter == 0) {
            if (open) {
                open();
            } else {
                close();
            }
            open = !open;
        }

        counter++;
    }

    public void resetCounter() {
        counter = 0;
    }
}