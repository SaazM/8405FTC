package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

        // --- RESET ALL MOTOR POWERS TO 0 --- //
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);

        waitForStart();

        if (isStopRequested()) return;

        //temporary variables

        robot.motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int holding_pos_left = -1;
        int holding_pos_right = -1;
        double toPosition = -1;
        boolean isBusy = false;
        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y; // Remember, this is reversed!
            double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double turn = gamepad1.right_stick_x;
            //Field Centric
            if (gamepad1.right_bumper) {
                robot.isFieldCentric = !robot.isFieldCentric;
            }

            //Speed Multiplier
            double speedMultiplier = 0.5;

            if (gamepad1.left_bumper && robot.speedMultiplier != speedMultiplier) {
                robot.setSpeedMultipler(speedMultiplier); // reset it back to slow mode
            } else if (gamepad1.left_bumper) {
                robot.setSpeedMultipler(1);
            }


            if (robot.isFieldCentric) {
                robot.fieldCentric(power, strafe, turn);
            } else {
                robot.mecanum(power, strafe, turn);
            }


            //Claw Movements
            if (gamepad1.left_trigger > 0.1) {
                robot.claw(true);
            }


            if (gamepad1.right_trigger > 0.1){
                robot.claw(false);
            }


            //Lift Macros

            if(gamepad1.square)
            {
                holding_pos_left = 600;
                holding_pos_right = 600;

            }
            else if(gamepad1.circle)
            {
                holding_pos_left = 380;
                holding_pos_right = 380;


            }
            else if(gamepad1.cross)
            {
                holding_pos_left = 100;
                holding_pos_right = 100;

            }
            else if(gamepad1.dpad_up)
            {
                robot.motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.motorLiftLeft.setPower(0.5);
                robot.motorLiftRight.setPower(0.5);
                holding_pos_left = -1;
                holding_pos_right = -1;
            }
            else if(gamepad1.dpad_down)
            {
                robot.motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.motorLiftLeft.setPower(-0.5);
                robot.motorLiftRight.setPower(-0.5);
                holding_pos_left = -1;
                holding_pos_right = -1;
            }

            else if(gamepad1.dpad_right)
            {
                robot.motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorLiftLeft.setPower(0);
                robot.motorLiftRight.setPower(0);
                holding_pos_left = -1;
                holding_pos_right = -1;
            }
            else if(robot.motorLiftRight.getCurrentPosition() > 30 && robot.motorLiftRight.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
            {
                if(holding_pos_left ==-1)
                {
                    holding_pos_left = robot.motorLiftLeft.getCurrentPosition();
                    holding_pos_right = robot.motorLiftRight.getCurrentPosition();
                }
            }

            if(holding_pos_left != -1)
            {
                liftToPosition(robot, holding_pos_left, holding_pos_right);
            }


            //telemetry.addData("Power: ", power);
            //telemetry.addData("Strafe: ", strafe); //0 is straight forward, 1 is straight to the side
            telemetry.addData("IMU Heading: ", -robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Field Centric: ", robot.isFieldCentric);
            telemetry.addData("speed multiplier: ", robot.speedMultiplier);
            telemetry.addData("LEncoder", robot.motorLiftLeft.getCurrentPosition());
            telemetry.addData("REncoder", robot.motorLiftRight.getCurrentPosition());
            telemetry.addData("Lift in Moving: ", robot.motorLiftRight.isBusy());
            telemetry.update();
        }
    }
    private static void liftToPosition(Robot robot, int pos)
    {
        robot.motorLiftRight.setTargetPosition(pos);
        robot.motorLiftLeft.setTargetPosition(pos);
        robot.motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLiftRight.setPower(0.5);
        robot.motorLiftLeft.setPower(0.5);
    }
    private static void liftToPosition(Robot robot, int pos_left, int pos_right)
    {
        robot.motorLiftRight.setTargetPosition(pos_right);
        robot.motorLiftLeft.setTargetPosition(pos_left);
        robot.motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLiftRight.setPower(0.5);
        robot.motorLiftLeft.setPower(0.5);
    }
}