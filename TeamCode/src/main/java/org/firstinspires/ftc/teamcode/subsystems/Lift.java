package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.*;

public class Lift {

    private enum LIFT_MODE {
        MANUAL,MACRO,HOLD,NONE,RESET, KILLED
    }
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    //public DigitalChannel limitSwitch;

    public double startTime;
    public int holdingPosLeft;
    public int holdingPosRight;
    public boolean kill;

    //LIFT CONSTANTS
    private LinkedList<Double> storeCurrents = new LinkedList<>();
    public double rollingAverageCurrent = 0;
    private final double powerToVelocity = 435 * 384.5 / 60; //converts power into ticks per second
    private final double manualLiftPower = 0.8;
    private final double holdLiftPower = 0.3;
    private final double macroLiftPower = 0.5;
    private final double liftLimit = 2750; //upper lift limit
    private final double hertz = 20;

    public LIFT_MODE currentMode;

    private double startedHoldingTime = 0;



    public Lift(HardwareMap hardwareMap) {
        currentMode = LIFT_MODE.NONE;
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        //limitSwitch = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch");
        //limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        rightLift.setDirection(DcMotor.Direction.FORWARD);

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(7.5, 0, 0, 0));

        newBotStart();

        startTime = System.currentTimeMillis();
        holdingPosLeft = -1;
        holdingPosRight = -1;

    }

    public void oldBotLiftToPosition(int posLeft, int posRight, double power) {//power here is a fallacy; it just is percentage of lift velocity capability
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
        if(currentMode != LIFT_MODE.KILLED)
        {
            //MACROS
            if (gamepad.square) { // medium goal macro
                liftToMedium();
                currentMode = LIFT_MODE.MACRO;
            } else if (gamepad.circle) { // low goal macro
                liftToLow();
                currentMode = LIFT_MODE.MACRO;
            } else if (gamepad.left_bumper) { // stack macro
                liftToTopStack();
                currentMode = LIFT_MODE.MACRO;
            } else if (gamepad.triangle) { // high goal macro
                liftToHigh();
                currentMode = LIFT_MODE.MACRO;
            } else if (gamepad.right_bumper)
            {
                currentMode = LIFT_MODE.RESET;
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
                }else if(rightLift.getCurrentPosition() > 0){
                    rightLift.setVelocity(gamepad.left_trigger * -manualLiftPower * 0.2 * powerToVelocity);
                }
                else
                {
                    currentMode = LIFT_MODE.RESET;
                }
            }

            //HOLDING
            else if (rightLift.getCurrentPosition() > 100 && currentMode == LIFT_MODE.MANUAL) { // hold after manual ends
                currentMode = LIFT_MODE.HOLD;
                holdingPosLeft = leftLift.getCurrentPosition();
                holdingPosRight = rightLift.getCurrentPosition();
            }

            //ANALYSIS OF MODE
            if (currentMode == LIFT_MODE.HOLD || currentMode == LIFT_MODE.MACRO) { // goes to position asked for if needed
                if (currentMode == LIFT_MODE.HOLD){
                    oldBotLiftToPosition(holdingPosLeft, holdingPosRight, holdLiftPower);
                }
                else {
                    oldBotLiftToPosition(holdingPosLeft, holdingPosRight, macroLiftPower);
                }
            }
            else if(currentMode == LIFT_MODE.RESET)
            {
                rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLift.setMotorDisable();
            }

            //SETS
            if(currentMode == LIFT_MODE.HOLD)
            {
                if(startedHoldingTime == 0)
                {
                    startedHoldingTime = System.currentTimeMillis();
                }
                if(System.currentTimeMillis() - startedHoldingTime >= 20000)
                {
                    currentMode = LIFT_MODE.KILLED;
                }
            }
            else
            {
                startedHoldingTime = 0;
            }

            //Rolling Average Current
            if(storeCurrents.size() <= hertz*10)//10 seconds until start doing a rolling average
            {

                rollingAverageCurrent = rollingAverageCurrent * storeCurrents.size()/(storeCurrents.size()+1);
                rollingAverageCurrent += rightLift.getCurrent(CurrentUnit.AMPS)/(storeCurrents.size()+1);
                storeCurrents.add(rightLift.getCurrent(CurrentUnit.AMPS));
            }
            else
            {
                rollingAverageCurrent -= storeCurrents.pop()/(storeCurrents.size());
                storeCurrents.add(rightLift.getCurrent(CurrentUnit.AMPS));
                rollingAverageCurrent += rightLift.getCurrent(CurrentUnit.AMPS)/(storeCurrents.size());
            }
        }
        else
        {
            rightLift.setMotorDisable();
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

    public void autonRequest() {
        oldBotLiftToPosition(100,holdingPosRight,macroLiftPower);}


    public void newBotStart() {
        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setDirection(DcMotorEx.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void oldBotRestart() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void oldBotMacros(Gamepad gamepad1) {
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
            oldBotLiftToPosition(holdingPosLeft, holdingPosRight);
        }

        if(gamepad1.dpad_down || ((System.currentTimeMillis() - startTime)>15000)) {
            oldBotLiftToPosition(0, 0);
            rightLift.setPower(0);
            leftLift.setPower(0);
            kill = true;
        }
    }

    public void oldBotLiftToPosition(int posLeft, int posRight) {
        rightLift.setTargetPosition(posLeft);
        leftLift.setTargetPosition(posRight);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(0.5);
        leftLift.setPower(0.5);
    }
}