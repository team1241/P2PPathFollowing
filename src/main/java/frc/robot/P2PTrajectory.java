package frc.robot;

public class P2PTrajectory {
    private P2PWaypoint[] waypoints;
    private int waypointIndex;

    public P2PTrajectory() {

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

}
