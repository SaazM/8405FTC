package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltags.aprilTagsInit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class AutonAsync extends OpMode{
    Robot robot;
    int finalID = -1;
    DcMotorEx liftLeft;
    DcMotorEx liftRight;
    Intake intake;
    double startTime;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    aprilTagsInit apriltags;

    double tagForward = 0.01;
    double tagBack = 0;

    boolean toIntake = true;
    int liftTo = 0;
    // liftTo key:
    // 1 = medium
    // 2 = low
    // 3 = high
    // 4 = liftToTopStack
    // 0 = reset

    ElapsedTime timer;
    Trajectory t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;

    public AutonAsync(int tag_id, HardwareMap hardwareMap, Telemetry t, Gamepad gamepad1) {
        startTime = System.currentTimeMillis();
        robot = new Robot(hardwareMap, gamepad1);

        timer = new ElapsedTime();

        if (tag_id == 3) { // parking zone 1
            tagForward = -27;
        } else if (tag_id == 1) { // parking zone 3
            tagForward = 23;
        }
        else{
            tagForward=-3;
        }
    }

    public boolean waitSeconds(double seconds) // MUST BE DOUBLE
    {
        return timer.seconds() <= seconds;
    }

    public void runAutonParkOnly() {
        TrajectorySequence park = robot.drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(30)
                .forward(tagForward)
                .build();
        robot.drive.followTrajectorySequence(park);
    }

    public void runAutonRight() {
       // change first movement bc tile omegalol
        t1 = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeRight(55)
                .build();

        t2 = robot.drive.trajectoryBuilder(t1.end())
                .lineToLinearHeading(new Pose2d(t1.end().getX(), t1.end().getY(), Math.toRadians(-45)))
                .build();


        robot.drive.followTrajectoryAsync(t1);
    }

    public void runAutonLeft() {

    }

    public void intaking()
    {
        if(toIntake) {
            intake.intake();
        } else {
            //intake.outtake();
        }
    }

    public void lift_thingies()
    {
//        if(liftTo == 1) {
//            robot.lift.liftToMedium();
//        } else if (liftTo == 2) {
////            if (System.currentTimeMillis() - startTime <= 10000) {
//            robot.lift.liftToLow();
//        } else if (liftTo == 3) {
//            robot.lift.liftToHigh();
//        } else if (liftTo == 4) {
//            robot.lift.liftToTopStack();
//        } else if(liftTo == 5){
//            robot.lift.liftToMiddleOfStack();
//        } else {
//            startTime = System.currentTimeMillis();
//            if (robot.lift.rightLift.getCurrentPosition() > 50) {
//                robot.lift.liftToPosition(0, 0.4);
//            }
//        }
//        if(liftTo>=1)robot.lift.autonRequest();
    }


    @Override
    public void init() {

    }
    @Override
    public void init_loop() {

    }
    @Override
    public void start() {

    }

    public void stop() {
    }
    @Override
    public void loop() {
        robot.drive.update();
        telemetry.addData("STARTED", liftTo);
        telemetry.addData("PARKING ID: ", finalID);
        telemetry.update();
    }
}