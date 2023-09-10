package frc.robot.Example;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.P2PTrajectory;
import frc.robot.P2PWaypoint;

public class TrajectoryConstants {
    // define points on field in this class or a separate location constants class
    private static final Pose2d PRETAXI_BUMP_RED_POINT = new Pose2d(0, 0, null);
    private static final Pose2d TAXI_BUMP_RED_POINT = new Pose2d(0, 0, null);
    private static final Pose2d SECOND_GAMEPIECE_PICKUP_POINT = new Pose2d(0, 0, null);

    // create trajectory object with field points for use
    public static final P2PTrajectory TO_SECOND_GAMEPIECE_BUMP_RED = new P2PTrajectory(
            new P2PWaypoint[] {
                    new P2PWaypoint(PRETAXI_BUMP_RED_POINT, 1),
                    new P2PWaypoint(TAXI_BUMP_RED_POINT, 5),
                    new P2PWaypoint(SECOND_GAMEPIECE_PICKUP_POINT, 0) });

}
