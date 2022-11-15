package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class AutonAsync extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        aprilTagsInit init = new aprilTagsInit(hardwareMap, telemetry);
        init.initialize();

        while (!isStarted() && !isStopRequested())
        {
            init.search();
            sleep(20);
        }
        waitForStart();
        if (isStopRequested()) return;
        int finalID = init.stopAndSave();
        telemetry.addLine(Integer.toString(finalID));
        telemetry.update();
        Auton auton = new Auton(false, finalID);

        waitForStart();
//        while (opModeIsActive()) {
        auton.loop(new Robot(hardwareMap), hardwareMap);
//        }
    }

}