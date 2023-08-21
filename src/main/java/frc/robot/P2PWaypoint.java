package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class P2PWaypoint {
    private Pose2d pose;
    private double endRadius;

    public P2PWaypoint() {

    }

    public void setEndRadius(double endRadius) {
        this.endRadius = endRadius;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }
}
