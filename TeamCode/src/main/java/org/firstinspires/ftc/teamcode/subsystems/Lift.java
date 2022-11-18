package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;

    public double startTime;
    public int holdingPosLeft;
    public int holdingPosRight;
    public boolean kill;
    public boolean liftReached = true;//only used in auton


    public Lift(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startTime = System.currentTimeMillis();
        holdingPosLeft = -1;
        holdingPosRight = -1;
        kill = false;
    }

    public void liftToPosition(int posLeft, int posRight)
    {
        liftReached = (Math.abs(rightLift.getCurrentPosition() - posRight) < 15) && (Math.abs(leftLift.getCurrentPosition() - posLeft) < 15);
        rightLift.setTargetPosition(posLeft);
        leftLift.setTargetPosition(posRight);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(1);
        leftLift.setPower(1);
    }

    public void macros(Gamepad gamepad1) {
        double temp = startTime;
        boolean temp_ = kill;
        startTime = System.currentTimeMillis();
        kill = false;
        if (gamepad1.square) {
            holdingPosLeft = 680;
            holdingPosRight = 680;
        } else if (gamepad1.circle) {
            holdingPosLeft = 380;
            holdingPosRight = 380;
        } else if (gamepad1.dpad_up) {
            holdingPosLeft = 135;
            holdingPosRight = 135;
        } else if (gamepad1.right_trigger > 0.5) {

            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(1);
            rightLift.setPower(1);
            holdingPosLeft = -1;
            holdingPosRight = -1;
        } else if (gamepad1.left_trigger > 0.5) {

            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(-1);
            rightLift.setPower(-1);
            holdingPosLeft = -1;
            holdingPosRight = -1;
        } else if (rightLift.getCurrentPosition() > 30 && rightLift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            if (holdingPosLeft == -1) {
                holdingPosLeft = rightLift.getCurrentPosition();
                holdingPosRight = rightLift.getCurrentPosition();
            }
        }
        else
        {
            kill = temp_;
            startTime = temp;
        }
        if(gamepad1.right_bumper || ((System.currentTimeMillis() - startTime)>15000)) {
            holdingPosRight = 0;
            holdingPosLeft = 0;
            liftToPosition(0, 0);
            rightLift.setPower(0);
            leftLift.setPower(0);
            kill = true;
        }
        if(holdingPosLeft != -1 && !kill) {
            liftToPosition(holdingPosLeft, holdingPosRight);
        }


    }
    public void goToMediumGoal()
    {
        liftToPosition(630, 630);
    }
    public void goToLowGoal()
    {
        liftToPosition(380, 380);
    }

    public void reset() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}