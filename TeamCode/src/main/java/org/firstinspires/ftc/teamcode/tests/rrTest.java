package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="rrTest")
public class rrTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        // --- RESET ALL MOTOR POWERS TO 0 --- //
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);

        waitForStart();

        if (isStopRequested()) return;

        //temporary variables
        int fieldCentricTrigger = 0;
        boolean liftToggled = true;
        robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        double startTime = System.currentTimeMillis();
        while (((robot.lift.rightLift.getCurrentPosition() < 1000 || robot.lift.rightLift.getCurrentPosition() >1000)&&(robot.lift.leftLift.getCurrentPosition() < 1000 || robot.lift.leftLift.getCurrentPosition() >1000))){
            robot.lift.rightLift.setTargetPosition(1000);
            robot.lift.leftLift.setTargetPosition(1000);
            robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.rightLift.setPower(1);
            robot.lift.leftLift.setPower(1);
            if(System.currentTimeMillis()-startTime>1000){
                double startTime2 = System.currentTimeMillis();
                while(true){
                    robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lift.rightLift.setPower(1);
                    robot.lift.leftLift.setPower(1);

                    if(System.currentTimeMillis()-startTime2>10000) {
                        break;
                    }

                }
                break;
            }
        }




    }
    private void liftToPosition(DcMotorEx left, DcMotorEx right, int pos_left, int pos_right)
    {
        double startTime = System.currentTimeMillis();
        while (((right.getCurrentPosition() < pos_right-10 || right.getCurrentPosition() >pos_right+10)&&(left.getCurrentPosition() < pos_left-10 || left.getCurrentPosition() >pos_left+10)) || System.currentTimeMillis() - startTime > 3000){
            right.setTargetPosition(pos_right);
            left.setTargetPosition(pos_left);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setPower(1);
            left.setPower(1);
            telemetry.addData("time Diff: ",System.currentTimeMillis()-startTime);
        }
    }
}
