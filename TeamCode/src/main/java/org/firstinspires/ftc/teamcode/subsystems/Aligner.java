package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Aligner {
    public Servo aligner;
    public Aligner(HardwareMap hardwareMap)
    {
        this.aligner = hardwareMap.get(Servo.class, "aligner");
        aligner.setPosition(0);
    }
    public void retractAligner()
    {
        aligner.setPosition(0); //0.2
    }
    public void alignAligner()
    {
        aligner.setPosition(0.275); //0.4
    }
    public void outAligner()
    {
        aligner.setPosition(0.45);
    }
}
