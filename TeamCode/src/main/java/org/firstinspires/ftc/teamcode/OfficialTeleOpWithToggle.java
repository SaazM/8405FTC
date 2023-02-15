package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Current;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name="Official Teleop")
public class OfficialTeleOpWithToggle extends LinearOpMode {
    private final double inches_per_revolution = 60/25.4*Math.PI; //60 mm * (1 inches)/(25.4 mm) is the diameter of the wheel in inches, *pi for circumference
    private final double ticks_per_revolution = 360*6.0; //4 ticks per cycle & 360 cycle per revolution
    private final double mm_to_inches = 0.03937008;
    private boolean rounded = true; //toggle to make it more exact
    private final double round_coefficient = 10; //round to the nearest []th

    public double eerp(double t, double degree, double a, double b) {
        return a + (b - a) * Math.pow(t, degree);
    }

    public double deadband(double deadzone, double minval, double input, double degree) {
        int sign = input >= 0 ? 1 : -1;

        if (Math.abs(input) <= deadzone) {
            return 0;
        } else {
            return input;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1);

        robot.lift.newBotStart();

        waitForStart();
//        robot.drive.odomRetraction.setPosition(0);

        double startTime = System.currentTimeMillis();
        double endTime = -1;
        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y; // remember this is reversed
            double strafe = gamepad1.left_stick_x * 1.1; // counteract imperfect strafing
            double turn = gamepad1.right_stick_x;

            if (gamepad1.dpad_down) { // odom DOWN
                robot.aligner.outAligner();
            } else if (gamepad1.dpad_up) { // odom RETRACT
                robot.aligner.retractAligner();
            }
            else if(gamepad1.dpad_left)
            {
                robot.aligner.alignAligner();
            }

            if (gamepad1.right_stick_button) {
                robot.drive.switchSpeed();
            } else {
                robot.drive.switchingSpeed = false;
            }


            if (gamepad1.cross) {
                robot.intake.outtake();
                endTime = System.currentTimeMillis() + 1500;
            } else if(endTime > 0 && System.currentTimeMillis() < endTime) {
                robot.intake.outtake();
            } else {
                robot.intake.intake();
                endTime = -1;
            }

            double liftPos = (double) (robot.lift.rightLift.getCurrentPosition());

            robot.drive.moveTeleOp(power, strafe, turn, liftPos);

            //robot.lift.isHolding = false;
            robot.lift.liftTeleOp(gamepad1);


            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("speed multiplier: ", robot.drive.speedMultiplier);
            telemetry.addData("IsFieldCentric? ", robot.drive.isFieldCentric);
            telemetry.addData("Right Lift Requested Position", robot.lift.holdingPosRight);
            telemetry.addData("RIGHT Lift Position", robot.lift.rightLift.getCurrentPosition());
            telemetry.addData("Right Lift PID: ", robot.lift.rightLift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("CURRENT DRIVE MODE: ", robot.drive.leftFront.getMode());
            telemetry.addData("HoldingPosLeft: ", robot.lift.holdingPosLeft);
            telemetry.addData("LEFT Lift Position", robot.lift.leftLift.getCurrentPosition());
            telemetry.addData("Left Lift PID: ", robot.lift.leftLift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("Power: ", power);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Hertz", 1000.0/(System.currentTimeMillis() - startTime));
            telemetry.addData("RIGHT LIFT MOTOR: ", gamepad1.right_bumper);
            telemetry.addData("RIGHT LIFT MOTOR: ", robot.lift.rightLift.isMotorEnabled());
            telemetry.addData("RIGHT LIFT MOTOR POWER: ", robot.lift.rightLift.getPower());
            telemetry.addData("LEFT LIFT MOTOR: ", robot.lift.leftLift.isMotorEnabled());
            telemetry.addData("LEFT LIFT MOTOR POWER: ", robot.lift.leftLift.getPower());
            if(robot.lift.killTimer != null){telemetry.addData("kill timer: ", robot.lift.killTimer.seconds());}
            //telemetry.addData("Lift Limit Switch", robot.lift.limitSwitch.getState());


            telemetry.update();
            startTime = System.currentTimeMillis();
        }
    }
}