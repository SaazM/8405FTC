package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;

    public Lift(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftToPosition(int pos_left, int pos_right)
    {
        rightLift.setTargetPosition(pos_right);
        leftLift.setTargetPosition(pos_left);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(0.5);
        leftLift.setPower(0.5);
    }

    public void macros() {
        double startTime;
        boolean kill = false;
        int holding_pos_left =-1;
        int holding_pos_right = -1;

        if(gamepad1.square) {
            startTime = System.currentTimeMillis();
            kill = false;
            holding_pos_left = 600;
            holding_pos_right = 600;

        } else if(gamepad1.circle) {
            startTime = System.currentTimeMillis();
            kill = false;
            holding_pos_left = 380;
            holding_pos_right = 380;
        } else if(gamepad1.dpad_up) {
            startTime = System.currentTimeMillis();
            kill = false;
            holding_pos_left = 135;
            holding_pos_right = 135;
        } else if(gamepad1.right_trigger > 0.5) {
            startTime = System.currentTimeMillis();
            kill=false;
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(0.5);
            rightLift.setPower(0.5);
            holding_pos_left = -1;
            holding_pos_right = -1;
        } else if(gamepad1.left_trigger > 0.5) {
            startTime = System.currentTimeMillis();
            kill = false;
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setPower(-0.5);
            rightLift.setPower(-0.5);
            holding_pos_left = -1;
            holding_pos_right = -1;
        } else if(rightLift.getCurrentPosition() > 30 && rightLift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            if(holding_pos_left ==-1)
            {
                holding_pos_left = rightLift.getCurrentPosition();
                holding_pos_right = rightLift.getCurrentPosition();
            }
        }

        if(holding_pos_left != -1 && !kill) {
            liftToPosition(holding_pos_left, holding_pos_right);
        }

        if(gamepad1.dpad_down || ((System.currentTimeMillis() - startTime)>15000)) {
            liftToPosition(0, 0);
            rightLift.setPower(0);
            leftLift.setPower(0);
            kill=true;
        }
    }
}
