package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftTest {
    public DcMotorEx lift;

    public double startTime;
    public int holdingPosLeft;
    public int holdingPosRight;
    public boolean kill;
    public boolean liftReached = true;//only used in auton


    public LiftTest(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "rightLift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startTime = System.currentTimeMillis();
        holdingPosLeft = -1;
        holdingPosRight = -1;
        kill = false;
    }

    public void liftToPosition(int posLeft, int posRight)
    {
        liftReached = (Math.abs(lift.getCurrentPosition() - posRight) < 20);
        lift.setTargetPosition(posLeft);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }

    public void goToMediumGoal() { liftToPosition(630, 630); }

    public void goToLowGoal()  { liftToPosition(380, 380); }

    public void goToHighGoal() { liftToPosition(700, 700); }

    public void liftToMedium() {
//        liftToPositionAuton(620);
        liftToPosition(630, 630);
    }

    public void liftToLow() {
//        liftToPositionAuton(440);
        liftToPosition(380, 380);
    }

    public void liftToTopStack() {
//        liftToPositionAuton(150);
        liftToPosition(150, 150);
    }

    public void reset() {
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}