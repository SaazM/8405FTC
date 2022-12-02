package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp
public class AutonAsync extends OpMode
{
    Robot robot;
    int finalID = -1;
    DcMotorEx liftLeft;
    DcMotorEx liftRight;
    Intake intake;
    double startTime;

    boolean toIntake = true;
    int liftTo = 0;
    // liftTo key:
    // 1 = medium
    // 2 = low
    // 3 = high
    // 4 = liftToTopStack
    // 0 = reset

    Trajectory trajSeq1;
    ElapsedTime timer;

    Trajectory t1, t2, t3, t4, t5, t6, t7;

    public boolean waitSeconds(double seconds) // MUST BE DOUBLE
    {
        return timer.seconds() <= seconds;

    }




    public void runAutonHighSpamRight() {

         t1 = robot.drive.trajectoryBuilder(new Pose2d())
                 .addDisplacementMarker(() -> {liftTo = 3; toIntake = true;})
                .strafeRight(75)
                 .addDisplacementMarker(() -> {
                     timer.reset();

                     robot.drive.followTrajectoryAsync(t2);
                 })
                 .build();

         t2 = robot.drive.trajectoryBuilder(t1.end())
                .forward(3)
                .addDisplacementMarker(() -> {
                    timer.reset();

                    while(waitSeconds(1.0)){

                        robot.intake.outtake();
                        telemetry.addData("Intaking?: ", toIntake);
                        telemetry.update();
                    }
                    telemetry.addLine("Done Waiting");
                    telemetry.update();
                    toIntake = true;


                    robot.drive.followTrajectoryAsync(t4);
                })


                .build();

         t4 = robot.drive.trajectoryBuilder(t2.end())

                 .back(3)
                 .addDisplacementMarker(() ->{
                     liftTo = 4;
                     robot.drive.followTrajectoryAsync(t5);
                 })
                 .build();

         t5 = robot.drive.trajectoryBuilder(t4.end())
                 .lineToLinearHeading(new Pose2d(t4.end().getX(), t4.end().getY()+13, Math.toRadians(180)))
                 .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t6))


                 .build();

        t6 = robot.drive.trajectoryBuilder(t5.end())
                .forward(22)
                .addDisplacementMarker(() ->

                {
                    while (waitSeconds(1)) {
                    }
                    liftTo = 3;
                    while (waitSeconds(0.5)) {
                    }
                    robot.drive.followTrajectoryAsync(t7);
                })

                .build();

        t7 = robot.drive.trajectoryBuilder(t6.end())
                .back(23)
                .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t3))


                .build();

        t3 = robot.drive.trajectoryBuilder(t7.end())
                        .lineToLinearHeading(new Pose2d(t7.end().getX(), t7.end().getY()-13, 0))
                                .build();

        robot.drive.followTrajectoryAsync(t1);
    }

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        robot = new Robot(hardwareMap);
        timer = new ElapsedTime();
        liftLeft = robot.lift.leftLift;
        liftRight = robot.lift.rightLift;
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = new Intake(hardwareMap);

        runAutonHighSpamRight();
    }


    public void intaking()
    {
        if(toIntake)
        {
            intake.intake();
        }
        else
        {
            //intake.outtake();
        }
    }
    public void lift_thingies()
    {
        if(liftTo == 1) {

            robot.lift.liftToMedium();


        } else if (liftTo == 2) {
//            if (System.currentTimeMillis() - startTime <= 10000) {
            robot.lift.liftToLow();
//
        } else if (liftTo == 3) {
            robot.lift.liftToHigh();

        } else if (liftTo == 4) {

            robot.lift.liftToTopStack();


        }
        else
        {
            startTime = System.currentTimeMillis();
            if(robot.lift.rightLift.getCurrentPosition() > 50)
            {
                robot.lift.liftToPosition(0, 0, 0.4);
            }


        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
    }

    @Override
    public void loop() {
        robot.drive.update();
        lift_thingies();
        intaking();
        telemetry.addData("STARTED", liftTo);
        telemetry.update();
    }
}