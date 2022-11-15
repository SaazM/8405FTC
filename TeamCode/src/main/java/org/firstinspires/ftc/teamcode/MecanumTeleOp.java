package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsytems.Robot;

@TeleOp(name="DriveOfficial")
public class MecanumTeleOp extends LinearOpMode {
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
        // Figure out if we're left or right
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        robot.lift.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double toPosition = -1;
        boolean isBusy = false;
        double startTime = System.currentTimeMillis();
        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y; // Remember, this is reversed!
            double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double turn = gamepad1.right_stick_x;
            //Field Centric
            if (gamepad1.right_bumper) {
                robot.drive.isFieldCentric = !robot.drive.isFieldCentric;
            }

            //Speed Multiplier
            double speedMultiplier = 0.5;

            if (gamepad1.left_bumper) {
                robot.drive.setSpeedMultiplier(1); // reset it back to slow mode
            } else {
                robot.drive.setSpeedMultiplier(0.5);
            }

            if (robot.drive.isFieldCentric) {
                robot.drive.fieldCentric(power, strafe, turn);
            } else {
                robot.drive.driveCentric(power, strafe, turn);
            }

            //Claw Movements
            if(gamepad1.cross){
                robot.intake.close();
            }
            if(gamepad1.triangle){
                robot.intake.open();
            }

            //Lift Macros
            robot.lift.macros();

            //telemetry.addData("Power: ", power);
            //telemetry.addData("Strafe: ", strafe); //0 is straight forward, 1 is straight to the side
            telemetry.addData("IMU Heading: ", -robot.drive.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Field Centric: ", robot.drive.isFieldCentric);
            telemetry.addData("speed multiplier: ", robot.drive.speedMultiplier);
            telemetry.addData("LEncoder", robot.lift.leftLift.getCurrentPosition());
            telemetry.addData("REncoder", robot.lift.rightLift.getCurrentPosition());
            telemetry.addData("Kill: ", robot.lift.getKill());
            telemetry.addData("Start Time: ", startTime);
            telemetry.addData("Time: ", System.currentTimeMillis());

            telemetry.update();
        }
    }
}