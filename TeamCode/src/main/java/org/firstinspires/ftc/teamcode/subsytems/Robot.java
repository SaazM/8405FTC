package org.firstinspires.ftc.teamcode.subsytems;

// roadrunner imports

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Robot {
    public Intake intake;
    public Lift lift;
    public Drive drive;

    public Robot(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        drive = new Drive(hardwareMap);
    }
}