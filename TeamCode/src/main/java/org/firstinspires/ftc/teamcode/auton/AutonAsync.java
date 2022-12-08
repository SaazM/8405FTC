package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    Trajectory trajSeq1;
    ElapsedTime timer;

    Trajectory t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17;

    public AutonAsync(int tag_id, HardwareMap hardwareMap, Telemetry t) {
        startTime = System.currentTimeMillis();
        robot = new Robot(hardwareMap);

        timer = new ElapsedTime();
        //liftLeft = robot.lift.leftLift;
        liftRight = robot.lift.rightLift;
        //liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = robot.intake;
        telemetry = t;



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

    public void runAutonThreeConeDefensiveLeft() {
        // strafe left to med goal
        // outtake
        // spline turn to stack
        // intake
        // go back
        // turn to high
        // outtake
        // park

        t1 = robot.drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {liftTo = 1; toIntake = true;})
                .strafeLeft(47)
                .addDisplacementMarker(() -> {
                    robot.drive.followTrajectoryAsync(t2);
                })
                .build();

        t2 = robot.drive.trajectoryBuilder(t1.end())
                .forward(3)
                .addDisplacementMarker(() -> {
                    timer.reset();
                    while(waitSeconds(1.0)) {
                        robot.intake.outtake();
                        telemetry.addData("Intaking?: ", toIntake);
                        telemetry.update();
                    }
                    telemetry.addLine("Done Waiting");
                    telemetry.update();
                    toIntake = true;
                    robot.drive.followTrajectoryAsync(t3);
                })
                .build();

        t3 = robot.drive.trajectoryBuilder(t2.end())
                .lineToLinearHeading(new Pose2d(t2.end().getX(), t2.end().getY()+13, Math.toRadians(185)))
                .addDisplacementMarker(() -> {
                    timer.reset();

                    telemetry.addLine("Done Waiting");
                    telemetry.update();
                    liftTo = 4;
                    robot.drive.followTrajectoryAsync(t4);
                })
                .build();

        t4 = robot.drive.trajectoryBuilder(t3.end())
                .forward(30)
                .addDisplacementMarker(() -> {
                    toIntake = true;
                    while (waitSeconds(1.0)) {
                        robot.intake.intake();
                        telemetry.addData("Intaking?: ", toIntake);
                        telemetry.update();
                    }
                    liftTo = 3;
                    robot.drive.followTrajectoryAsync(t5);
                })
                .build();



        t5 = robot.drive.trajectoryBuilder(t4.end())
                .back(60)
                .addDisplacementMarker(() -> {
                    robot.drive.followTrajectory(t6);
                })
                .build();


        t6 = robot.drive.trajectoryBuilder(t5.end())
                .lineToLinearHeading(new Pose2d(t5.end().getX()+0.01, t5.end().getY()-0.01, Math.toRadians(135)))
                .build();



        robot.drive.followTrajectoryAsync(t1);
    }



    public void runAutonHighSpamRight() {
         t1 = robot.drive.trajectoryBuilder(new Pose2d())
                 .addDisplacementMarker(() -> {liftTo = 3; toIntake = true;})
                 .lineToConstantHeading(new Vector2d(2.5, -74))
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


        //first cone
         t2 = robot.drive.trajectoryBuilder(t1.end())
                .forward(3.5)
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

         t4 = robot.drive.trajectoryBuilder(t1.end())
                 .back(3)
                 .addDisplacementMarker(() ->{
                     liftTo = 4;
                     robot.drive.followTrajectoryAsync(t5);
                 })
                 .build();

         t5 = robot.drive.trajectoryBuilder(t4.end())
                 .lineToLinearHeading(new Pose2d(t4.end().getX(), t4.end().getY()+13, Math.toRadians(185)))
                 .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t6))


                 .build();

        t6 = robot.drive.trajectoryBuilder(t5.end())
                .forward(24.5)
                .addDisplacementMarker(() -> {
                    while (waitSeconds(1)) {
                    }
                    liftTo = 3;

                    robot.drive.followTrajectoryAsync(t7);
                })

                .build();

        t7 = robot.drive.trajectoryBuilder(t6.end())
                .back(23.5)
                .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t8))


                .build();


        t8 = robot.drive.trajectoryBuilder(t7.end())
                .lineToLinearHeading(new Pose2d(t7.end().getX(), t7.end().getY()-13, Math.toRadians(359)))
                .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t9))
                .build();

        //second cone
        t9 = robot.drive.trajectoryBuilder(t8.end())
                .forward(2)
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

                    liftTo = 5;
                    robot.drive.followTrajectoryAsync(t10);
                })


                .build();


        t10 = robot.drive.trajectoryBuilder(t9.end())
                .back(2)
                .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t11))
                .build();
        t11 = robot.drive.trajectoryBuilder(t10.end())
                .strafeLeft(12)
                .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t12))

                .build();

        t12 = robot.drive.trajectoryBuilder(t11.end())
                .forward(tagForward)

                .build();

        t13 = robot.drive.trajectoryBuilder(t12.end())
                .back(23.5)
                .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t14))


                .build();
        t14 = robot.drive.trajectoryBuilder(t13.end())
                .lineToLinearHeading(new Pose2d(t13.end().getX(), t13.end().getY()-13, Math.toRadians(355)))
                .addDisplacementMarker(() -> robot.drive.followTrajectoryAsync(t15))
                .build();
        // third cone

        t15 = robot.drive.trajectoryBuilder(t14.end())
                .forward(1)
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
                    liftTo = 6;
                    //robot.drive.followTrajectoryAsync(t16);

                })


                .build();
        /**t16 = robot.drive.trajectoryBuilder(t15.end())
                .lineToConstantHeading(new Vector2d(t15.end().getX()-3, t15.end().getY()+12))
                .lineToConstantHeading(new Vector2d(t15.end().getX() - 3 + tagForward, t15.end().getY() + 12))

                                .build();**/




        robot.drive.followTrajectoryAsync(t1);
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
        if(liftTo == 1) {
            robot.lift.liftToMedium();
        } else if (liftTo == 2) {
//            if (System.currentTimeMillis() - startTime <= 10000) {
            robot.lift.liftToLow();
        } else if (liftTo == 3) {
            robot.lift.liftToHigh();
        } else if (liftTo == 4) {
            robot.lift.liftToTopStack();
        } else if(liftTo == 5){
            robot.lift.liftToMiddleOfStack();
        } else {
            startTime = System.currentTimeMillis();
            if (robot.lift.rightLift.getCurrentPosition() > 50) {
                robot.lift.oldBotLiftToPosition(0, 0, 0.4);
            }
        }
        if(liftTo>=1)robot.lift.autonRequest();

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
        lift_thingies();
        intaking();
        telemetry.addData("STARTED", liftTo);
        telemetry.addData("PARKING ID: ", finalID);
        telemetry.update();
    }
}