package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Aligner {
    public static final double retractval = 1;
    public static final double alignval = 0.11;
    public static final double outtakingval = 0;


    public Servo aligner;
    public Aligner(HardwareMap hardwareMap)
    {
        this.aligner = hardwareMap.get(Servo.class, "aligner");

        aligner.setPosition(retractval);
    }
    public void retractAligner()
    {
        aligner.setPosition(retractval); //0.2
    }
    public void alignAligner()
    {
        aligner.setPosition(alignval); //0.4
    }
    public void outAligner()
    {
        aligner.setPosition(outtakingval);
    }
}
