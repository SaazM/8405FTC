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
    public boolean liftReached = true;
    public boolean isHolding = false;



    public Lift(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");


        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newBotStart();

        startTime = System.currentTimeMillis();
        holdingPosLeft = -1;
        holdingPosRight = -1;
        kill = false;
    }

    public void liftToPosition(int posLeft, int posRight, double power) {
        liftReached = (Math.abs(rightLift.getCurrentPosition() - posRight) < 20) || (Math.abs(leftLift.getCurrentPosition() - posLeft) < 20);
        rightLift.setTargetPosition(posLeft);
        leftLift.setTargetPosition(posRight);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(0.3);
        leftLift.setPower(0.3);
    }

    public void newMacros(Gamepad gamepad1) {
        double temp = startTime;
        boolean temp_ = kill;
        boolean tempHolding = isHolding;
        startTime = System.currentTimeMillis();
        kill = false;
        isHolding = false;
        if (gamepad1.square) {
            liftToMedium();
        } else if (gamepad1.circle) {
            liftToLow();
        } else if (gamepad1.dpad_up) {
            liftToTopStack();
        } else if (gamepad1.triangle) {
            liftToHigh();
        } else if (gamepad1.right_trigger > 0.5) {
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(0.3);
            rightLift.setPower(0.3);
            holdingPosLeft = -1;
            holdingPosRight = -1;
        } else if (gamepad1.left_trigger > 0.5) {
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(-0.3);
            rightLift.setPower(-0.3);
            holdingPosLeft = -1;
            holdingPosRight = -1;
        }
        else if (rightLift.getCurrentPosition() > 30 && rightLift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            isHolding = true;
            if (holdingPosLeft == -1) {
                holdingPosLeft = leftLift.getCurrentPosition();
                holdingPosRight = rightLift.getCurrentPosition();
            }
        }
        else {
            kill = temp_;
            startTime = temp;
            isHolding = tempHolding;
        }
        if (gamepad1.right_bumper || ((System.currentTimeMillis() - startTime)>20000)) {
            rightLift.setPower(0);
            leftLift.setPower(0);
            kill = true;
        }
        if (holdingPosLeft != -1 && !kill) {
            if (isHolding) {
                liftToPosition(holdingPosLeft, holdingPosRight, 0.05);
            } else {
                liftToPosition(holdingPosLeft, holdingPosRight, 0.3);
            }
        }
    }

    public void liftToMedium() { liftToPosition(2025, 2000, 0.8); }

    public void liftToLow() { liftToPosition(1025, 1000, 0.8); }

    public void liftToTopStack() { liftToPosition(195, 180, 0.8); }

    public void liftToHigh() { liftToPosition(2675, 2650, 0.8); }

    public void newBotStart() {
        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setDirection(DcMotorEx.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}