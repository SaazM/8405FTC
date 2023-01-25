package org.firstinspires.ftc.teamcode.subsystems;

// roadrunner imports

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Robot {
    public Drive drive;
    public Lift lift;
    public Intake intake;
    public DistanceSensor distanceSensor;


    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        drive = new Drive(hardwareMap);
        lift = new Lift(hardwareMap, gamepad);
        intake = new Intake(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color");
    }
}