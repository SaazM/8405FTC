package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class LeftAuton extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagsInit init = new aprilTagsInit(hardwareMap, telemetry);
        init.initialize();

        while (!isStarted() && !isStopRequested()) {
            init.search();
            sleep(20);
        }
        waitForStart();
        int finalID = init.stopAndSave();
        telemetry.addLine(Integer.toString(finalID));
        telemetry.update();
        AutonAsync auton = new AutonAsync();
        auton.runAutonOffensive();
    }
}