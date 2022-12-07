package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Lift {

    private enum LIFT_MODE {
        MANUAL,MACRO,HOLD,NONE
    }
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    public DigitalChannel limitSwitch;

    public double startTime;
    public int holdingPosLeft;
    public int holdingPosRight;

    //LIFT CONSTANTS
    private final double powerToVelocity = 435 * 384.5 / 60; //converts power into ticks per second
    private final double manualLiftPower = 0.8;
    private final double holdLiftPower = 0.3;
    private final double macroLiftPower = 0.8;
    private final double liftLimit = 2750; //upper lift limit

    private LIFT_MODE currentMode;



    public Lift(HardwareMap hardwareMap) {
        currentMode = LIFT_MODE.NONE;
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

    }

    public void liftToPosition(int posLeft, int posRight, double power) {//power here is a fallacy; it just is percentage of lift velocity capability
        rightLift.setTargetPosition(posRight);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (Math.abs(posRight - rightLift.getCurrentPosition()) <= 20) {
            currentMode = LIFT_MODE.HOLD;
        }
        //DETERMINE VALIDITY OF POSITION
        if(posRight >= 0 && posRight<=liftLimit)
        {
            rightLift.setPower(power);
        }

    }

    public void liftTeleOp(Gamepad gamepad) {

        //MACROS
        if (gamepad.square) { // medium goal macro
            liftToMedium();
            currentMode = LIFT_MODE.MACRO;
        } else if (gamepad.circle) { // low goal macro
            liftToLow();
            currentMode = LIFT_MODE.MACRO;
        } else if (gamepad.dpad_up) { // stack macro
            liftToTopStack();
            currentMode = LIFT_MODE.MACRO;
        } else if (gamepad.triangle) { // high goal macro
            liftToHigh();
            currentMode = LIFT_MODE.MACRO;
        }

        //MANUAL
        if (gamepad.right_trigger > 0.5) { // move lift up
            currentMode = LIFT_MODE.MANUAL;
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (rightLift.getCurrentPosition() < liftLimit - 20) {
                rightLift.setVelocity(gamepad.right_trigger*manualLiftPower * 0.95 * powerToVelocity);
            }
            if (rightLift.getCurrentPosition() > liftLimit) {
                rightLift.setVelocity(0);
            }
        } else if (gamepad.left_trigger > 0.5) { // move lift down
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentMode = LIFT_MODE.MANUAL;
            if (rightLift.getCurrentPosition() > 100) {
                rightLift.setVelocity(gamepad.left_trigger * -manualLiftPower * 0.6 * powerToVelocity);
            } else {
                rightLift.setVelocity(gamepad.left_trigger *-manualLiftPower * 0.2 * powerToVelocity);
            }
        }

        //HOLDING
        else if (rightLift.getCurrentPosition() > 100 && currentMode == LIFT_MODE.MANUAL) { // hold after manual ends
            currentMode = LIFT_MODE.HOLD;
            holdingPosLeft = leftLift.getCurrentPosition();
            holdingPosRight = rightLift.getCurrentPosition();

        }

        //ANALYSIS OF MODE
        if (currentMode != LIFT_MODE.MANUAL && currentMode != LIFT_MODE.NONE) { // goes to position asked for if needed
            if (currentMode == LIFT_MODE.HOLD){
                liftToPosition(holdingPosLeft, holdingPosRight, holdLiftPower);
            }
            else {
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