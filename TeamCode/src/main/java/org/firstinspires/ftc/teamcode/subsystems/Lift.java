package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Lift {
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    public DigitalChannel limitSwitch;

    public double startTime;
    public int holdingPosLeft;
    public int holdingPosRight;
    public boolean kill;
    public boolean liftReached = true;
    public boolean isHolding = false;
    private double powerToVelocity = 435 * 384.5 / 60; //converts power into ticks per second
    private double manualLiftPower = 0.8;
    private double holdLiftPower = 0.3;
    private double macroLiftPower = 0.8;
    private double liftLimit = 2750; //upper lift limit


    public Lift(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");


        rightLift.setDirection(DcMotor.Direction.FORWARD);

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(7.5, 0, 0, 0));

        newBotStart();

        startTime = System.currentTimeMillis();
        holdingPosLeft = -1;
        holdingPosRight = -1;
        kill = false;
    }

    public void liftToPosition(int posLeft, int posRight, double power) {//power here is a fallacy; it just is percentage of lift velocity capability
        liftReached = (Math.abs(rightLift.getCurrentPosition() - posRight) < 20);
        rightLift.setTargetPosition(posRight);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (Math.abs(posRight - rightLift.getCurrentPosition()) <= 20) {
            isHolding = true;
        }
        if (posRight < rightLift.getCurrentPosition() && posRight >= 0) {
            rightLift.setPower(power * 0.75);
        } else if (posRight > rightLift.getCurrentPosition() && posRight <= liftLimit) {
            rightLift.setPower(power);
        } else {
            rightLift.setPower(0);
        }
    }

    public void liftTeleOp(Gamepad gamepad) {
        double startTimeTemp = startTime;
        boolean prevKill = kill;
        boolean prevHolding = isHolding;
        startTime = System.currentTimeMillis();
        kill = false;
        isHolding = false;
        /**
         if(limitSwitch.getState())
         {
         rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         }
         **/
        if (gamepad.square) { // medium goal macro
            liftToMedium();
        } else if (gamepad.circle) { // low goal macro
            liftToLow();
        } else if (gamepad.dpad_up) { // stack macro
            liftToTopStack();
        } else if (gamepad.triangle) { // high goal macro
            liftToHigh();
        } else if (gamepad.right_trigger > 0.5) { // move lift up
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (rightLift.getCurrentPosition() < liftLimit - 40) {
                rightLift.setVelocity(gamepad.right_trigger*manualLiftPower * 0.95 * powerToVelocity);
            }

            if (rightLift.getCurrentPosition() > liftLimit) {
                rightLift.setVelocity(0);
            }

            holdingPosLeft = -1;
            holdingPosRight = -1;
        } else if (gamepad.left_trigger > 0.5) { // move lift down
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if (rightLift.getCurrentPosition() < 0) {
//                rightLift.setPower(0);
//            } else if (rightLift.getCurrentPosition() > 100) {//if lift is below 100, slow down so we dont blow up the lift
//                rightLift.setVelocity(-manualLiftPower * 0.95 * powerToVelocity * 0.3);
//            } else {
//                rightLift.setVelocity(-manualLiftPower * 0.3 * powerToVelocity * 0.3);
//            }
            if(rightLift.getCurrentPosition() > 100)
            {
                rightLift.setVelocity(gamepad.left_trigger*-manualLiftPower * 0.6 * powerToVelocity);
            }
            else

            {rightLift.setVelocity(-manualLiftPower * 0.5 * powerToVelocity * 0.3);}


            holdingPosRight = -1;
        } else if (rightLift.getCurrentPosition() > 20 && rightLift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) { // holding
            isHolding = true;
            if (holdingPosRight == -1) {
                holdingPosLeft = leftLift.getCurrentPosition();
                holdingPosRight = rightLift.getCurrentPosition();
            }
        } else { // prevents holding when lift is at bottom
            kill = prevKill;
            startTime = startTimeTemp;
            isHolding = prevHolding;
//            holdingPosRight = -1;
//            holdingPosLeft = -1;
            isHolding = true;
        }
        if (gamepad.right_bumper || ((System.currentTimeMillis() - startTime) > 120000)  || Math.min(leftLift.getCurrentPosition(), rightLift.getCurrentPosition())<0) { // kills lift power
            rightLift.setPower(0);
            leftLift.setPower(0);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            holdingPosLeft = -1;
            holdingPosRight = -1;
            kill = true;
            isHolding = false;
        }

        if (holdingPosRight != -1 && !kill) { // holds power

            if (isHolding) { // maintains current height
                liftToPosition(holdingPosLeft, holdingPosRight, holdLiftPower);
            } else {
                liftToPosition(holdingPosLeft, holdingPosRight, macroLiftPower);
            }
        }
    }
    public void liftToZero(){holdingPosRight = 0;}
    public void liftToMedium() {
        holdingPosRight = 1860;
    }

    public void liftToLow() {
        holdingPosRight = 930;
    }

    public void liftToTopStack() {
        holdingPosRight = 320;
    }

    public void liftToMiddleOfStack() {
        holdingPosRight = 200;
    }

    public void liftToHigh() {
        holdingPosRight = 2700;
    }

    public void autonRequest() {liftToPosition(100,holdingPosRight,macroLiftPower);}


    public void newBotStart() {
        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setDirection(DcMotorEx.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}