
package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class LeftAuton extends OpMode
{
    aprilTagsInit init;
    AutonAsync auton;
    boolean activated = false;
    Gamepad gamepad1;


    @Override
    public void init() {
        init = new aprilTagsInit(hardwareMap, telemetry);
        init.initialize();
    }
    @Override
    public void init_loop()
    {
        init.search();
        int finalID = init.stopAndSave() + 1;

    }
    @Override
    public void start()
    {
        int finalID = init.stopAndSave() + 1;
        if(finalID == 1){finalID = 2;}
        else if(finalID == 2){finalID = 1;}
        telemetry.addLine(Integer.toString(finalID));
        telemetry.update();
        auton = new AutonAsync(finalID, hardwareMap, telemetry, gamepad1);
//        auton.runAutonThreeConeDefensiveLeft();
        auton.runAutonParkOnly();
        auton.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        activated = true;
    }
    @Override
    public void loop() {
        if(activated)
        {
            auton.robot.drive.update();
            auton.lift_thingies();
            auton.intaking();

            telemetry.update();
        }
    }
}