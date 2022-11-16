package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;

    public double startTime;
    public int holdingPosLeft;
    public int holdingPosRight;
    public boolean kill;

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
        rightLift.setTargetPosition(posLeft);
        leftLift.setTargetPosition(posRight);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(0.5);
        leftLift.setPower(0.5);
    }

    public void macros() {
        if (gamepad1.square) {
            startTime = System.currentTimeMillis();
            kill = false;
            holdingPosLeft = 600;
            holdingPosRight = 600;
        } else if (gamepad1.circle) {
            startTime = System.currentTimeMillis();
            kill = false;
            holdingPosLeft = 380;
            holdingPosRight = 380;
        } else if (gamepad1.dpad_up) {
            startTime = System.currentTimeMillis();
            kill = false;
            holdingPosLeft = 135;
            holdingPosRight = 135;
        } else if (gamepad1.right_trigger > 0.5) {
            startTime = System.currentTimeMillis();
            kill = false;
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(0.5);
            rightLift.setPower(0.5);
            holdingPosLeft = -1;
            holdingPosRight = -1;
        } else if (gamepad1.left_trigger > 0.5) {
            startTime = System.currentTimeMillis();
            kill = false;
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(-0.5);
            rightLift.setPower(-0.5);
            holdingPosLeft = -1;
            holdingPosRight = -1;
        } else if (rightLift.getCurrentPosition() > 30 && rightLift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            if (holdingPosLeft == -1) {
                holdingPosLeft = rightLift.getCurrentPosition();
                holdingPosRight = rightLift.getCurrentPosition();
            }
        }

        if(holdingPosLeft != -1 && !kill) {
            liftToPosition(holdingPosLeft, holdingPosRight);
        }

        if(gamepad1.dpad_down || ((System.currentTimeMillis() - startTime)>15000)) {
            liftToPosition(0, 0);
            rightLift.setPower(0);
            leftLift.setPower(0);
            kill = true;
        }
    }

    public void liftToPositionAuton(int pos) {
        startTime = System.currentTimeMillis();
        while (((rightLift.getCurrentPosition() < pos - 10 || rightLift.getCurrentPosition() > pos + 10) && (leftLift.getCurrentPosition() < pos - 10 || leftLift.getCurrentPosition() > pos + 10))) {
            rightLift.setTargetPosition(pos);
            leftLift.setTargetPosition(pos);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(1);
            leftLift.setPower(1);
            if (System.currentTimeMillis() - startTime > 3000) {
                double startTime2 = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime2 <= 2000) {
                    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightLift.setPower(1);
                    leftLift.setPower(1);
                }
                break;
            }
        }
    }

    public void reset() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
