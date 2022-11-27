package org.firstinspires.ftc.teamcode.roadrunnerfiles;

import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;


import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

public class TrajectoryBuilder {
    //MAJOR UNCERTAINTY IF THIS WORKS, RELIES ON A FEW ASSUMPTIONS
    ArrayList<com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder> requests;
    Pose2d currentPose;
    Robot robot;
    public TrajectoryBuilder (Robot robot)
    {
        requests = new ArrayList<>();
        currentPose = new Pose2d();
        this.robot = robot;
    }
    public void turn(double angle)//degrees
    {

        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .lineToSplineHeading(new Pose2d(currentPose.getX(), currentPose.getY(), (currentPose.getHeading() + angle * Math.PI/180)%(2*Math.PI)));//this might not work

        requests.add(t);
        currentPose = t.build().end();//ASSUMPTION 1: THIS WORKS AND T DOES NOT DESTROY ITSELF
    }

    public void forward(double measure)
    {
        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .forward(measure);

        requests.add(t);
        currentPose = t.build().end();//ASSUMPTION 1: THIS WORKS AND T DOES NOT DESTROY ITSELF
    }

    public void back(double measure)
    {
        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .back(measure);

        requests.add(t);
        currentPose = t.build().end();//ASSUMPTION 1: THIS WORKS AND T DOES NOT DESTROY ITSELF
    }

    public void strafeLeft(double measure)
    {
        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .strafeLeft(measure);

        requests.add(t);
        currentPose = t.build().end();//ASSUMPTION 1: THIS WORKS AND T DOES NOT DESTROY ITSELF
    }

    public void strafeRight(double measure)
    {
        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .strafeRight(measure);

        requests.add(t);
        currentPose = t.build().end();//ASSUMPTION 1: THIS WORKS AND T DOES NOT DESTROY ITSELF
    }

    public void lineToLinearHeading(Pose2d arg)
    {
        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .lineToLinearHeading(arg);

        requests.add(t);
        currentPose = t.build().end();
    }
    public void lineToSplineHeading(Pose2d arg)
    {
        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .lineToSplineHeading(arg);

        requests.add(t);
        currentPose = t.build().end();
    }

    public void splineToLinearHeading(Pose2d arg, double endTangent)
    {

        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .splineToLinearHeading(arg, endTangent);

        requests.add(t);
        currentPose = t.build().end();

    }
    public void splineToSplineHeading(Pose2d arg, double endTangent)
    {

        com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                .splineToSplineHeading(arg, endTangent);

        requests.add(t);
        currentPose = t.build().end();

    }

    public void addDisplacementMarker(MarkerCallback arg) throws Exception {
        if(requests.size() == 0)
        {
            throw new Exception("Can't add marker at this point! need movement somewhere");
        }
        else
        {
            requests.get(requests.size()-1).addDisplacementMarker(arg);
        }
    }

    public void waitSeconds(double seconds) throws Exception//i cannot be doing this lmao[the way i'm doing it is making it so the drive either reaches the seconds waiting OR hits perfection on position, which can't happen so]
    {
        if(requests.size() == 0)
        {
            throw new Exception("Can't wait at this point! need movement somewhere");
        }
        else
        {
            HolonomicPIDVAFollower original = (HolonomicPIDVAFollower) robot.drive.follower;
            requests.get(requests.size()-1).addDisplacementMarker(() -> robot.drive.changeFollowerAccuracy(seconds, 0, 0));
            com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder t = robot.drive.trajectoryBuilder(currentPose)
                    .forward(0.001)
                    .addDisplacementMarker(() -> robot.drive.follower = original);
            requests.add(t);
            currentPose = t.build().end();
        }
    }
    public void reset()
    {
        requests = new ArrayList<>();
        currentPose = new Pose2d();
    }

    public Trajectory compile()//connects all the trajectories and then spits out the initial trajectory node
    {
        for(int i = requests.size()-2; i>=0; i--)
        {
            int tempI = i;
            requests.get(i).addDisplacementMarker(() -> requests.get(tempI +1).build());
        }
        return requests.get(0).build();
    }
}

