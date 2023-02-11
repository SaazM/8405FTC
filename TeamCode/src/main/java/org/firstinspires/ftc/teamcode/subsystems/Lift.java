package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.*;

public class Lift {
    // -- DECLARATIONS -- //
    public enum LIFT_MODE {
        MANUAL,MACRO,HOLD,NONE,RESET, KILLED
    }
    public DcMotorEx leftLift;
    public DcMotorEx rightLift;
    public double startTime;
    public int holdingPosLeft;
    public int holdingPosRight;
    public boolean kill;

    // -- LIFT CONSTANTS -- //
    public double rollingAverageCurrent = 0;
    private final double manualLiftPowerUp = 0.8;
    private final double manualLiftPowerDown = 0.5;
    private final double holdLiftPower = 0.3;
    private final double macroLiftPower = 1;
    private final double liftLimit = 2750; //upper lift limit
    public ElapsedTime killTimer = null;
    public LIFT_MODE currentMode;
    private double startedHoldingTime = 0;
    private Gamepad gamepad;


    public Lift(HardwareMap hardwareMap, Gamepad gamepad) {
        currentMode = LIFT_MODE.NONE;
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.gamepad = gamepad;
        newBotStart();
        startTime = System.currentTimeMillis();
        holdingPosRight = -1;
        holdingPosLeft = -1;
    }

    public void liftToPosition(int posRequest, int posRequestLeft, double power) {
        if (Math.abs(posRequest - rightLift.getCurrentPosition()) <= 20) {
            currentMode = LIFT_MODE.HOLD;
        }
        //DETERMINE VALIDITY OF POSITION
        if(posRequest > 10  && posRequest < liftLimit)
        {
            rightLift.setTargetPosition(posRequest);
            leftLift.setTargetPosition(posRequestLeft);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setPower(power);
            leftLift.setPower(power);
        }
        else
        {
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setPower(0);
            leftLift.setPower(0);
        }

    }

    private void setLiftPower(double power)
    {
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    private void liftMacro()
    {
        if (gamepad.right_bumper)
        {
            currentMode = LIFT_MODE.RESET;
        }
        else
        {
            if (gamepad.square) { // medium goal macro
                liftToMedium();
            } else if (gamepad.circle) { // low goal macro
                liftToLow();
            } else if (gamepad.left_bumper) { // stack macro
                liftToTopStack();
            } else if (gamepad.triangle) { // high goal macro
                liftToHigh();
            } else if(gamepad.dpad_up)
            {
                liftToTopStack();
            }
        }


    }
    private void liftManual()
    {
        if (gamepad.right_trigger > 0.5) { // move lift up
            if (rightLift.getCurrentPosition() < liftLimit - 20) {
                setLiftPower(gamepad.right_trigger*manualLiftPowerUp);

            }

            if (rightLift.getCurrentPosition() > liftLimit) {
                setLiftPower(0);
            }

        }
        else if (gamepad.left_trigger > 0.5) { // move lift down
            if (rightLift.getCurrentPosition() > 100) {
                setLiftPower(gamepad.left_trigger * -manualLiftPowerDown);
            } else if(rightLift.getCurrentPosition() > 0){
                setLiftPower(gamepad.left_trigger * -manualLiftPowerDown * 0.2);
            }
            else
            {
                setLiftPower(0);
            }
        }

    }
    private void liftAnalysis(boolean isAuton)
    {
        if (currentMode == LIFT_MODE.HOLD || currentMode == LIFT_MODE.MACRO) { // goes to position asked for if needed

            if (currentMode == LIFT_MODE.HOLD){
                liftToPosition(holdingPosRight, holdingPosLeft, holdLiftPower);
            }
            else {
                liftToPosition(holdingPosRight, holdingPosLeft, macroLiftPower);
            }
        }
        else if(currentMode == LIFT_MODE.RESET)
        {
            if(rightLift.getCurrentPosition() > 250)
            {
                setLiftPower(-0.6);
            }
            else
            {
                if(killTimer == null){
                    killTimer = new ElapsedTime();
                    killTimer.reset();

                }
                if(killTimer.milliseconds() <= 1000)
                {
                    setLiftPower(-0.4);
                }
                else
                {

                    rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
        }

        if(currentMode != LIFT_MODE.RESET)
        {
            killTimer = null;

        }



        if(currentMode == LIFT_MODE.HOLD && !isAuton)
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
    }
    public void liftTeleOp(Gamepad gamepad) {
        this.gamepad = gamepad;
        if(currentMode != LIFT_MODE.KILLED)
        {
            if(gamepad.square || gamepad.circle || gamepad.left_bumper || gamepad.triangle || gamepad.right_bumper)
            {
                currentMode = LIFT_MODE.MACRO;
                liftMacro();
            }



            // MANUAL
            if(gamepad.right_trigger > 0.5 || gamepad.left_trigger > 0.5)
            {
                currentMode = LIFT_MODE.MANUAL;
                liftManual();

            }

            // HOLDING
            else if (rightLift.getCurrentPosition() > 100 && currentMode == LIFT_MODE.MANUAL) { // hold after manual ends
                currentMode = LIFT_MODE.HOLD;

                holdingPosRight = rightLift.getCurrentPosition();
                holdingPosLeft = leftLift.getCurrentPosition();
            }

            //ANALYSIS OF MODE
            liftAnalysis(false);

        }
        else
        {
            rightLift.setMotorDisable();
            leftLift.setMotorDisable();
        }
    }


    public void liftToMedium() {
        holdingPosRight = 1860;
        holdingPosLeft = 1860;
        currentMode = LIFT_MODE.MACRO;
    }

    public void liftToLow() {
        holdingPosRight = 930;
        holdingPosLeft = 930;
        currentMode = LIFT_MODE.MACRO;
    }

    public void liftToTopStack() {
        holdingPosRight = 380;
        holdingPosLeft = 380;
        currentMode = LIFT_MODE.MACRO;
    }

    public void liftToMiddleOfStack() {
        holdingPosRight = 230;
        holdingPosLeft = 230;
        currentMode = LIFT_MODE.MACRO;
    }
    public void liftToBottomOfStack() {
        holdingPosRight = 150;
        holdingPosLeft = 150;
        currentMode = LIFT_MODE.MACRO;
    }

    public void liftToHigh() {
        holdingPosRight = 2650;
        holdingPosLeft = 2650;
        currentMode = LIFT_MODE.MACRO;
    }
    public void liftToHigh2() {
        holdingPosRight = 2570;
        holdingPosLeft = 2570;
        currentMode = LIFT_MODE.MACRO;
    }

    public void liftToBottom() {
        holdingPosRight = 0;
        holdingPosLeft = 0;
        currentMode = LIFT_MODE.MACRO;
    }


    public void autonRequest()
    {
        liftAnalysis(true);
    }


    public void newBotStart() {
        leftLift.setDirection(DcMotorEx.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}