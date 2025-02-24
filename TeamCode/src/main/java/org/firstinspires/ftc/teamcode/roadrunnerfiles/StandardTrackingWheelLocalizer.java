package org.firstinspires.ftc.teamcode.roadrunnerfiles;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192.0;
    public static double WHEEL_RADIUS = 63/64.0; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 9.00992126; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4.45; // in; offset of the lateral wheel
    private static double Y_MULT = 71.25/72.18;
    private static double X_MULT = 1;
    public Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {

        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder")); //port E3
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder")); //port E0
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft")); //port C3
        frontEncoder.setDirection(Encoder.Direction.FORWARD);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition() * X_MULT),
                encoderTicksToInches(rightEncoder.getCurrentPosition() * X_MULT),
                encoderTicksToInches(frontEncoder.getCurrentPosition() * Y_MULT)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity() * X_MULT),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity() * X_MULT),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity() * Y_MULT)
        );
    }
}