package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.TRACK_WIDTH;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.List;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.kA;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunnerfiles.DriveConstantsCurrent.kV;

public class Drive extends MecanumDrive {
    public BNO055IMU imu;
    public boolean isFieldCentric;

    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(2, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0);
    public static double LATERAL_MULTIPLIER = 1;//-61.39376023178146/-52.0 * -61.16032488575252/-62.0;//should be 1.153846, but b/c we tuned based around 1, i will keep it at 1
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;




    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;
    private double powerToVelocity = 435*384.5/60;

    public double speedMultiplier = 1;
    private double accelLimit = 1;//max acceleration in ticks per second
    private double lastUpdate = System.currentTimeMillis();

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public TrajectoryFollower follower;

    private List<DcMotorEx> motors;

    private VoltageSensor batteryVoltageSensor;

    public Drive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID, new Pose2d(0.3, 0.3, Math.toRadians(0.5)), 0.75);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftFront.setDirection(DcMotor.Direction.REVERSE); // motor direction
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Braking behavior
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // We don't want to use PID for the motors using the encoders

        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        isFieldCentric = false;

        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    public void changeFollowerAccuracy(double timeout, double translational_error, double turn_error) {
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID, new Pose2d(translational_error, translational_error, Math.toRadians(turn_error)), timeout);
    }

    public double moveTeleOp(double power, double strafe, double turn, double liftPos) {
        //double limiter = (3100 - liftPos) / 3100;
        double limiter = driveAntiTip(liftPos/3100, 435);

        if (isFieldCentric) {
            fieldCentric(power*limiter, strafe*limiter, turn);
        } else {
            robotCentric(power*limiter * 3/4 + power/4, strafe, turn);
            robotCentric(power*limiter, strafe*limiter, turn);
        }
        return limiter;
    }

    public void robotCentric(double power, double strafe, double turn) {
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        setDrivePowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    public void fieldCentric(double power, double strafe, double turn) {
        double botHeading = -imu.getAngularOrientation().thirdAngle;
        double rotationX = strafe * Math.cos(botHeading) - power * Math.sin(botHeading);
        double rotationY = strafe * Math.sin(botHeading) + power * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (rotationY + rotationX + turn) / denominator;
        double backLeftPower = (rotationY - rotationX + turn) / denominator;
        double frontRightPower = (rotationY - rotationX - turn) / denominator;
        double backRightPower = (rotationY + rotationX - turn) / denominator;

        setDrivePowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }
    public double driveAntiTip(double liftPosition,  double startLimitPoint)
    {
        return Math.min(1, Math.exp(-1.5*(liftPosition - startLimitPoint/3100)));
    }
    public void setDrivePowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        double dampenBy = 0.3;
        double frontLeftRequest = frontLeftPower * speedMultiplier * powerToVelocity;
        double rearLeftRequest = backLeftPower * speedMultiplier * powerToVelocity;
        double frontRightRequest = frontRightPower * speedMultiplier * powerToVelocity;
        double rearRightRequest = backRightPower * speedMultiplier * powerToVelocity;
//        if(frontLeftRequest >= 0 && frontRightRequest >= 0)
//        {
//            frontLeftRequest = Math.min(1, frontLeftRequest*1.75);
//            rearLeftRequest = Math.min(1, rearLeftRequest*1.75);
//            frontRightRequest = Math.min(1, frontRightRequest*1.75);
//            rearRightRequest = Math.min(1, rearRightRequest*1.75);
//        }
//
//        if(Math.abs(leftFront.getVelocity())>200 && Math.abs(rightFront.getVelocity()) > 200 && (Math.signum(leftFront.getVelocity()) == Math.signum(rightFront.getVelocity()))){
//            if(Math.abs(leftFront.getVelocity() - frontLeftRequest) > 200){
//                frontLeftRequest *= dampenBy;
//                rearLeftRequest *= dampenBy;
//                frontRightRequest *= dampenBy;
//                rearRightRequest *= dampenBy;
//            }
//        }
 
//        if(leftFront.getVelocity()>500 && rightFront.getVelocity()>500 ){
//            if(frontLeftRequest<0){
//                frontLeftRequest *= 0.1;
//                rearLeftRequest *= 0.1;
//                frontRightRequest *= 0.1;
//                rearRightRequest *= 0.1;
//            }
//        }
//        if(leftFront.getVelocity()<-500 && rightFront.getVelocity()<-500){
//            if(frontLeftRequest>0){
//                frontLeftRequest *= 0.1;
//                rearLeftRequest *= 0.1;
//                frontRightRequest *= 0.1;
//                rearRightRequest *= 0.1;
//            }
//        }

        /**
        if(leftFront.getVelocity()<-200 && rightFront.getVelocity() < -200){
            if(frontLeftRequest>0){
                frontLeftRequest *= dampenBy;
                rearLeftRequest *= dampenBy;
                frontRightRequest *= dampenBy;
                rearRightRequest *= dampenBy;
            }
        }
         **/

//        if(leftFront.getVelocity()<-200){
//            if(frontLeftRequest>0){
//                frontLeftRequest *= 0.4;
//                rearLeftRequest *= 0.4;
//                frontRightRequest *= 0.4;
//                rearRightRequest *= 0.4;
//            }
//        }
        leftFront.setVelocity(frontLeftRequest);
        leftRear.setVelocity(rearLeftRequest);
        rightFront.setVelocity(frontRightRequest);
        rightRear.setVelocity(rearRightRequest);
    }

    public void switchDrive() {
        isFieldCentric = !isFieldCentric;
    }

    public void slowMode() {
        speedMultiplier = 0.5;
    }

    public void fastMode() {
        speedMultiplier = 0.8;
    }

    public void turnToPosition(double rotation) {
        while(this.imu.getAngularOrientation().firstAngle <= rotation) {
            this.robotCentric(0, 0, 1);
        }
    }

    /* ---------- ROADRUNNER METHODS ---------- */

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double maxVelo, double maxAccel) {
        MinVelocityConstraint myVelConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAccel),
                new MecanumVelocityConstraint(maxVelo, TRACK_WIDTH)
        ));
        ProfileAccelerationConstraint myAccelConstraint = new ProfileAccelerationConstraint(maxAccel);
        return new TrajectoryBuilder(startPose, myVelConstraint, myAccelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setVelocity(v * powerToVelocity);
        leftRear.setVelocity(v1 * powerToVelocity);
        rightRear.setVelocity(v2 * powerToVelocity);
        rightFront.setVelocity(v3 * powerToVelocity);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
