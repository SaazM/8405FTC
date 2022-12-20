package org.firstinspires.ftc.teamcode.subsystems;

// roadrunner imports

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Robot {
    public Drive drive;
    public Lift lift;
    public Intake intake;


    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new Drive(hardwareMap);
        lift = new Lift(hardwareMap, gamepad);
        intake = new Intake(hardwareMap);
    }
}