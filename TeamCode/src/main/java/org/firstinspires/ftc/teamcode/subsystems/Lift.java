package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public double rollingAverageCurrent = 0;
    private final double manualLiftPowerUp = 0.5;
    private final double manualLiftPowerDown = 0.1;
    private final double holdLiftPower = 0.3;
    private final double macroLiftPower = 0.5;
    private final double liftLimit = 2750; //upper lift limit
    public ElapsedTime killTimer = null;
    private final double hertz = 20;

    public LIFT_MODE currentMode;

    private double startedHoldingTime = 0;
    private Gamepad gamepad;
    private PID pid;



    public Lift(HardwareMap hardwareMap, Gamepad gamepad) {
        currentMode = LIFT_MODE.NONE;
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.gamepad = gamepad;
        newBotStart();
        startTime = System.currentTimeMillis();
        holdingPosRight = -1;
        pid = new PID(7.5,0,0);

    }

    public void liftToPosition(int posRequest, double power) {//power here is a fallacy; it just is percentage of lift velocity capability
        double adjustPower = power * pid.getValue(posRequest - rightLift.getCurrentPosition());
        setLiftPower(adjustPower);
        if (Math.abs(posRequest - rightLift.getCurrentPosition()) <= 20) {
            currentMode = LIFT_MODE.HOLD;
        }
        //DETERMINE VALIDITY OF POSITION
        if(posRequest > 0 && posRequest<liftLimit)
        {
            rightLift.setPower(power);
            leftLift.setPower(power);
        }

    }
    private void setLiftPower(double power)
    {

        leftLift.setPower(power);
        //rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            rightLift.setMotorEnable();
            leftLift.setMotorEnable();
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        rightLift.setMotorEnable();
        leftLift.setMotorEnable();
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                if(!rightLift.isMotorEnabled())rightLift.setMotorEnable();
                setLiftPower(gamepad.left_trigger * -manualLiftPowerDown);
            }else if(rightLift.getCurrentPosition() > 0){
                if(!rightLift.isMotorEnabled())rightLift.setMotorEnable();
                setLiftPower(gamepad.left_trigger * -manualLiftPowerDown * 0.2);
            }
            else
            {
                //currentMode = LIFT_MODE.RESET;
            }
        }

    }
    private void liftAnalysis(boolean isAuton)
    {
        if (currentMode == LIFT_MODE.HOLD || currentMode == LIFT_MODE.MACRO) { // goes to position asked for if needed

            if (currentMode == LIFT_MODE.HOLD){
                //liftToPosition(holdingPosRight, holdLiftPower);
                setLiftPower(0);
            }
            else {
                liftToPosition(holdingPosRight, macroLiftPower);
            }
        }
        else if(currentMode == LIFT_MODE.RESET)
        {
            if(killTimer == null){
                killTimer = new ElapsedTime();
                killTimer.reset();

            }
            if(killTimer.milliseconds() >= 1000 &&
                killTimer.milliseconds() <= 2000)
                {

                    rightLift.setMotorEnable();
                    rightLift.setPower(-0.6);
                    leftLift.setMotorEnable();
                    leftLift.setPower(-0.6);
                }

            else if(killTimer.milliseconds() < 1000 || killTimer.milliseconds() > 2000)
            {
                leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLift.setMotorDisable();
                rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLift.setMotorDisable();
            }
            //
            // pid.reset();

        }

        if(currentMode != LIFT_MODE.RESET)
        {
            killTimer = null;
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            //whenever new action is taken, lift PID should be reset
            //MACROS

            if(gamepad.square || gamepad.circle || gamepad.left_bumper || gamepad.triangle || gamepad.right_bumper)
            {
                //pid.reset();
                currentMode = LIFT_MODE.MACRO;
                liftMacro();
            }



            //MANUAL

            if(gamepad.right_trigger > 0.5 || gamepad.left_trigger > 0.5)
            {
                //pid.reset();
                currentMode = LIFT_MODE.MANUAL;
                liftManual();

            }

            //HOLDING
            else if (rightLift.getCurrentPosition() > 100 && currentMode == LIFT_MODE.MANUAL) { // hold after manual ends
                currentMode = LIFT_MODE.HOLD;

                //pid.reset();
                holdingPosRight = rightLift.getCurrentPosition();
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
    public void autonRequest()
    {
        liftAnalysis(true);
    }


    public void newBotStart() {
        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setDirection(DcMotorEx.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}