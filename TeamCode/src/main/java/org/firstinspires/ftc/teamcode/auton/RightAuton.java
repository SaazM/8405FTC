
package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class RightAuton extends OpMode
{
    aprilTagsInit init;
    AutonAsync auton;
    boolean activated = false;
    Trajectory t1, t2, t3;
    Gamepad gamepad1;

    @Override
    public void init() {
        //init = new aprilTagsInit(hardwareMap, telemetry);
        //init.initialize();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void init_loop()
    {
        //init.search();
        //int finalID = init.stopAndSave() + 1;

    }
    @Override
    public void start()
    {
        // int finalID = init.stopAndSave() + 1;
        // if(finalID == 1){finalID = 2;}
        // else if(finalID == 2){finalID = 1;}
        telemetry.addLine(Integer.toString(0));
        telemetry.update();
        auton = new AutonAsync(0, hardwareMap, telemetry, gamepad1);
//        auton.runAutonRight();
        //auton.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //.addDisplacementMarker(() -> auton.robot.drive.followTrajectoryAsync(t2))


        // change first movement bc tile omegalol

        t1 = auton.robot.drive.trajectoryBuilder(new Pose2d())
                .strafeRight(48)
                .build();

//        t2 = auton.robot.drive.trajectorySequenceBuilder(t1.end())
                //.lineToLinearHeading(new Pose2d(t1.end().getX(), t1.end().getY(), Math.toRadians(-45)))
//                .build();


        auton.robot.drive.followTrajectoryAsync(t1);
        activated = true;
        telemetry.addData("extermnal heading velo: ", auton.robot.drive.getExternalHeadingVelocity());
        telemetry.addData("activated? ", activated);
        telemetry.update();
    }
    @Override
    public void loop() {
        if(activated)
        {
            auton.robot.drive.update();
            //auton.lift_thingies();
            //auton.intaking();

            telemetry.update();
        }
    }
}