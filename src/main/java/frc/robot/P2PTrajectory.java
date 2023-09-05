package frc.robot;

import java.lang.reflect.Array;

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

        public double getEndRadius() {
            return endRadius;
        }

        public Pose2d getPose() {
            return pose;
        }

        public void setPose(Pose2d pose) {
            this.pose = pose;
        }
        
        
    }
    
    private P2PWaypoint[] waypoints;
    private int waypointIndex;

    // constructor
    public P2PTrajectory(P2PWaypoint[] wayPoints, int length) {
        wayPoints = new P2PWaypoint[length];
    }

    /**
     * 
     * @return current waypoint 
     */
    public P2PWaypoint getCurrentWaypoint() {
        return waypoints[waypointIndex];
    }

    /**
     * change to next waypoint
     */
    public void nextWaypoint() {
        if (waypointIndex + 1 < waypoints.length - 1) {
            waypointIndex++;
        }
    }

    // ************** Getters & Setters **************
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

    /**
     * 
     * @return true when the waypoint is the last one in the trajectory
     */
    public boolean isCurrentEndPoint() {
        return waypointIndex == waypoints.length - 1;
    }

}
