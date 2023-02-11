package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry {
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor perpendicularEncoder;
    private double trackWidth;
    private double forwardOffset;
    private final double wheel_circumference_inches = 63/32.0*Math.PI;
    private final double mm_to_inches = 0.03937008;
    private final double ticks_per_revolution = 8192.0;

    private double prev_left_encoder_pos = 0.0;
    private double prev_right_encoder_pos = 0.0;
    private double prev_perpendicular_encoder_pos = 0.0;
    private double x_pos = 0.0;
    private double y_pos = 0.0;
    private double heading = 0.0;

    // instantiate odometry class
    public Odometry(DcMotor getLeftEncoder,
                    DcMotor getRightEncoder,
                    DcMotor getPerpEncoder) {
        leftEncoder = getLeftEncoder;
        rightEncoder = getRightEncoder;
        perpendicularEncoder = getPerpEncoder;
        trackWidth = 109.375*2*mm_to_inches; // get actual measurement
        forwardOffset = -(112.6125+2)*mm_to_inches; // get actual measurement - 112.6125
    }

    // run odometry and return list of doubles in form [x, y, heading]
    public void runOdom() {
        double left_encoder_pos = leftEncoder.getCurrentPosition()/ticks_per_revolution * wheel_circumference_inches;//inches
        double right_encoder_pos = rightEncoder.getCurrentPosition()/ticks_per_revolution * wheel_circumference_inches;
        double perpendicular_encoder_pos = perpendicularEncoder.getCurrentPosition()/ticks_per_revolution * wheel_circumference_inches;

        double delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        double delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        double delta_perpendicular_encoder_pos = perpendicular_encoder_pos - prev_perpendicular_encoder_pos;

        double phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackWidth;
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        double delta_perp_pos = delta_perpendicular_encoder_pos - forwardOffset * phi;
        double delta_x;
        double delta_y;
        delta_x = delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading);
        delta_y = delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading);
        /**
        if (phi != 0) {
            delta_x = Math.cos(heading) * ((Math.sin(phi)/phi)*delta_middle_pos + (Math.cos(phi)-1)/phi * delta_perp_pos) -
                    Math.sin(heading)*((1-Math.cos(phi))/phi * delta_middle_pos + (Math.sin(phi)/phi)*delta_perp_pos);

            delta_y = Math.sin(heading)*((Math.sin(phi)/phi)*delta_middle_pos + (Math.cos(phi)-1)/phi * delta_perp_pos) +
                    Math.cos(heading)*((1-Math.cos(phi))/phi * delta_middle_pos + (Math.sin(phi)/phi)*delta_perp_pos);
        } else {
            delta_x = delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading);
            delta_y = delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading);
        }
         **/

        x_pos += delta_x;
        y_pos += delta_y;
        heading += phi;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_perpendicular_encoder_pos = perpendicular_encoder_pos;
    }

    public double getX() {
        return y_pos;
    }

    public double getY() {
        return x_pos;
    }

    public double getHeading() {
        return heading;
    }
    
    public void setX(double value) {
        x_pos = value;
    }
    
    public void setY(double value) {
        y_pos = value;
    }
    
    public void setHeading(double value) {
        heading = value;
    }

    public void reset() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public double getForwardOffset() {
        return forwardOffset;
    }
}
