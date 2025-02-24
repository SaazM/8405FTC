package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Disabled
@TeleOp(name="Old TeleOp")
public class OldBotTeleOp extends LinearOpMode {
    private final double inches_per_revolution = 60/25.4*Math.PI; //60 mm * (1 inches)/(25.4 mm) is the diameter of the wheel in inches, *pi for circumference
    private final double ticks_per_revolution = 360*6.0; //4 ticks per cycle & 360 cycle per revolution
    private final double mm_to_inches = 0.03937008;
    private boolean rounded = true;//toggle to make it more exact
    private final double round_coefficient = 10;//round to the nearest []th

    public double eerp(double t, double degree, double a, double b) {
        return a + (b - a) * Math.pow(t, degree);
    }
    public double deadband(double deadzone, double minval, double input, double degree) {
        int sign = input >= 0 ? 1 : -1;
        if (Math.abs(input) <= deadzone) {
            return 0;
        } else {
            return sign * eerp(Math.abs(input), degree, minval, 1);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, gamepad1);

        waitForStart();

        if (isStopRequested()) return;

        //robot.lift.oldBotRestart();

        double startTime = System.currentTimeMillis();
        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y; // remember this is reversed
            double strafe = gamepad1.left_stick_x * 1.1; // counteract imperfect strafing
            double turn = gamepad1.right_stick_x;

            if (gamepad1.left_bumper) {
                robot.drive.switchSpeed();
            } else {
                robot.drive.switchingSpeed = false;
            }

            //robot.drive.moveTeleOp(power, strafe, turn);

//            if (gamepad1.cross) {
//                robot.intake.moveClaw();
//            } else {
//                robot.intake.resetCounter();
//            }

            if (gamepad1.dpad_down) {
                robot.drive.switchDrive();
            }

            //robot.lift.oldBotMacros(gamepad1);

            telemetry.addData("IMU Heading: ", -robot.drive.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("speed multiplier: ", robot.drive.speedMultiplier);
            telemetry.addData("LEncoder", robot.lift.leftLift.getCurrentPosition());
            telemetry.addData("REncoder", robot.lift.rightLift.getCurrentPosition());
            telemetry.addData("Kill: ", robot.lift.kill);
            telemetry.addData("Open? ", robot.intake.open);
            telemetry.addData("Start Time: ", startTime);
            telemetry.addData("Time: ", System.currentTimeMillis());
            telemetry.addData("IsFieldCentric? ", robot.drive.isFieldCentric);

            telemetry.update();
        }
    }
}