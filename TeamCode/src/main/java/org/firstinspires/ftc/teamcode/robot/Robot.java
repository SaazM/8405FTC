package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequenceRunner;

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
        intake.claw.resetDeviceConfigurationForOpMode();
        lift = new Lift(hardwareMap);
        drive = new Drive(hardwareMap);
    }
}
