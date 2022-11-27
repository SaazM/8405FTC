package org.firstinspires.ftc.teamcode.subsystems;

// roadrunner imports

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class OldRobot {
    public Intake intake;
    public Lift lift;
    public Drive drive;

    public OldRobot(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        drive = new Drive(hardwareMap);
    }
}