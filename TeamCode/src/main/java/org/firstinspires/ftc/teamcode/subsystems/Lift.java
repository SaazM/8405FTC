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
    private final double manualLiftPowerUp = 0.8;
    private final double manualLiftPowerDown = 0.4;
    private final double holdLiftPower = 0.3;
    private final double macroLiftPower = 0.5;
    private final double liftLimit = 2750; //upper lift limit
    private final double hertz = 20;

    public LIFT_MODE currentMode;

    private double startedHoldingTime = 0;
    private Gamepad gamepad;



    public Lift(HardwareMap hardwareMap, Gamepad gamepad) {
        currentMode = LIFT_MODE.NONE;
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        //limitSwitch = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch");
        //limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        rightLift.setDirection(DcMotor.Direction.FORWARD);

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(7.5, 0, 0, 0));
        this.gamepad = gamepad;
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
    private void setLiftSpeed(double velocity)
    {
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setVelocity(velocity);
        leftLift.setVelocity(velocity);

    }
    private void liftMacro()
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

        if (gamepad.right_bumper)
        {
            currentMode = LIFT_MODE.RESET;
        }
    }
    private void liftManual()
    {
        if (gamepad.right_trigger > 0.5) { // move lift up
            if (rightLift.getCurrentPosition() < liftLimit - 20) {
                setLiftSpeed(gamepad.right_trigger*manualLiftPowerUp * powerToVelocity);
            }
            if (rightLift.getCurrentPosition() > liftLimit) {
                setLiftSpeed(0);
            }
        } else if (gamepad.left_trigger > 0.5) { // move lift down
            if (rightLift.getCurrentPosition() > 100) {
                setLiftSpeed(gamepad.left_trigger * -manualLiftPowerDown * powerToVelocity);
            }else if(rightLift.getCurrentPosition() > 0){
                setLiftSpeed(gamepad.left_trigger * -manualLiftPowerDown * 0.2 * powerToVelocity);
            }
            else
            {
                currentMode = LIFT_MODE.RESET;
            }
        }
    }
    private void liftAnalysis(boolean isAuton)
    {
        if (currentMode == LIFT_MODE.HOLD || currentMode == LIFT_MODE.MACRO) { // goes to position asked for if needed
            if (currentMode == LIFT_MODE.HOLD){
                liftToPosition(holdingPosLeft, holdingPosRight, holdLiftPower);
            }
            else {
                liftToPosition(holdingPosLeft, holdingPosRight, macroLiftPower);
            }
        }
        else if(currentMode == LIFT_MODE.RESET)
        {
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMotorDisable();
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
        if(currentMode != LIFT_MODE.KILLED)
        {
            //MACROS
            if(gamepad.square || gamepad.circle || gamepad.left_bumper || gamepad.triangle || gamepad.right_bumper)
            {
                currentMode = LIFT_MODE.MACRO;
                liftMacro();
            }

            //MANUAL
            else if(gamepad.right_trigger > 0.5 || gamepad.left_trigger > 0.5)
            {
                currentMode = LIFT_MODE.MANUAL;
                liftManual();
            }

            //HOLDING
            else if (rightLift.getCurrentPosition() > 100 && currentMode == LIFT_MODE.MANUAL) { // hold after manual ends
                currentMode = LIFT_MODE.HOLD;
                holdingPosLeft = leftLift.getCurrentPosition();
                holdingPosRight = rightLift.getCurrentPosition();
            }
            //ANALYSIS OF MODE
            liftAnalysis(false);
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