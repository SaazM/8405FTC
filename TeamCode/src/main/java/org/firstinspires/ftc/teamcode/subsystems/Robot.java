package org.firstinspires.ftc.teamcode.subsystems;

// roadrunner imports

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Robot {
    public Drive drive;
    public Lift lift;
    public Intake intake;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public Robot(HardwareMap hardwareMap, int i) {
        drive = new Drive(hardwareMap);
        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap, i);
    }
}