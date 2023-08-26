package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class P2PTrajectory {

    // waypoint class
    public class P2PWaypoint {
        private Pose2d pose;
        private double endRadius;

        public P2PWaypoint() {
        }

        public void setEndRadius(double endRadius) {
            this.endRadius = endRadius;
        }

        public double getEndRadius(){
            return endRadius;
        }

        public Pose2d getPose(){
            return pose;
        }

        public void setPose(Pose2d pose) {
            this.pose = pose;
        }
    }

    private P2PWaypoint[] waypoints;
    private int waypointIndex;

    public P2PTrajectory(P2PWaypoint[] wayPoints) {

    }

    public P2PWaypoint[] getWaypoints() {
        return waypoints;
    }

    public P2PWaypoint getWayPoint(int index) {
        if (index >= 0 && index < waypoints.length) {
            return waypoints[index];
        }
        return null;
    }

    public void setWaypoints(P2PWaypoint[] waypoints) {
        this.waypoints = waypoints;
    }

    public void setWaypoint(P2PWaypoint waypoint, int index) {
        if (index >= 0 && index < waypoints.length) {
            waypoints[index] = waypoint;
        }
    }

    public Pose2d getCurrentWayPoint(){
        return waypoints[waypointIndex];
    }

    public boolean isEndPoint() {
        return waypointIndex == waypoints.length - 1;
    }


}
