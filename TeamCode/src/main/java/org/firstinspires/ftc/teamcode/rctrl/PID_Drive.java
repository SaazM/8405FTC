package org.firstinspires.ftc.teamcode.rctrl;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class PID_Drive {
    private PIDController controllerX;
    public static double xP = 0.0, xI = 0.0, xD = 0.0;
    private PIDController controllerY;
    public static double yP = 0.0, yI = 0.0, yD = 0.0;
    private  PIDController controllerHeading;
    public static double hP = 0.0, hI = 0.0, hD = 0.0;

    private MotionProfile motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(1, 0, 0), 1, 1, 1);
    private final ElapsedTime elapsedTime = new ElapsedTime();

    public boolean targetReached = false;

    private Drive drive;

    public PID_Drive() {
        controllerX = new PIDController(xP, xI, xD);
        controllerY = new PIDController(yP, yI, yD);
        controllerHeading = new PIDController(hP, hI, hD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new Drive(hardwareMap);
    }

    public void pointToPoint(Pose2d currPos, Pose2d targetPos, Pose2d startPos) {
        // distance from robot to target
        double distanceToTarget = Math.abs(Math.hypot(targetPos.getX()-currPos.getX(), targetPos.getY()-currPos.getX()));
        // distance from start to target
        double initialDistance = Math.abs(Math.hypot(targetPos.getX()-startPos.getX(), targetPos.getY()-startPos.getY()));
        // distance from robot to start
        double distanceFromStart = initialDistance-distanceToTarget;
        // angle from start to target
        double initialAngle = Math.atan2(targetPos.getY()-startPos.getY(), targetPos.getX()-startPos.getX());
        elapsedTime.reset();

        // get state from the motion profile
        MotionState state = motionProfile.get(elapsedTime.seconds());
        // generate position using motion profile (desired position at any given moment)
        Vector2d statePos = new Vector2d(startPos.getX()+(state.getX() * Math.cos(initialAngle)), startPos.getY()+(state.getX() * Math.sin(initialAngle)));
        // distance to state position from start position
        double distanceToState = Math.abs(Math.hypot(statePos.getX()-startPos.getX(), statePos.getY()-startPos.getY()));

        // calculate desired power, strafe, and turn using PID
        double power = controllerX.calculate(distanceToState * Math.cos(initialAngle) - (currPos.getX()-startPos.getX()));
        double strafe = controllerY.calculate(distanceToState * Math.sin(initialAngle) - (currPos.getY()-startPos.getY()));
        double turn = controllerHeading.calculate(AngleUnit.normalizeRadians(targetPos.getHeading()-currPos.getHeading()));

        // use values on mecanum drive
        drive.move(power, strafe, turn);

        targetReached = motionProfile.duration() < elapsedTime.seconds();
    }
}
