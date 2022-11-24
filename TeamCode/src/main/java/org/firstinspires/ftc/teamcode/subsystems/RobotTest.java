package org.firstinspires.ftc.teamcode.subsystems;

// roadrunner imports

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class RobotTest {
    public Drive drive;

    public RobotTest(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
    }
}