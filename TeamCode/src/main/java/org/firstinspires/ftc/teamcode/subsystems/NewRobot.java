package org.firstinspires.ftc.teamcode.subsystems;

// roadrunner imports

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class NewRobot {
    public Drive drive;
    public Lift lift;

    public NewRobot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        lift = new Lift(hardwareMap);
    }
}