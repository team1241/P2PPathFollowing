package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.P2PTrajectory.P2PWaypoint;

public class P2PPathController {
    private PIDController pxController; // posiiton x controller
    private PIDController pyController; // position y controller
    private PIDController thetaController; // heading controller;
    private SlewRateLimiter vxSlewLimiter;
    private SlewRateLimiter vySlewLimiter;
    private SlewRateLimiter omegaSlewLimiter;
    public P2PTrajectory currentTrajectory;
    public Pose2d currentPose;

    // Constructor
    public P2PPathController(P2PTrajectory trajectory, double positionkP, double positionkI, double positionkD,
            double positionTolerance,
            double thetakP, double thetakI, double thetakD, double thetaTolerance, double slewRateVelocity,
            double slewRateOmega) {

        pxController = new PIDController(positionkP, positionkI, positionkD);
        pxController.setTolerance(positionTolerance);

        pyController = new PIDController(positionkP, positionkI, positionkD);
        pyController.setTolerance(positionTolerance);

        thetaController = new PIDController(thetakP, thetakI, thetakD);
        thetaController.setTolerance(thetaTolerance);

        vxSlewLimiter = new SlewRateLimiter(slewRateVelocity);
        vySlewLimiter = new SlewRateLimiter(slewRateVelocity);
        omegaSlewLimiter = new SlewRateLimiter(slewRateOmega);

    }

    // *************** Calculation Methods ***************

    /**
     * 
     * @param currentPose      current position of robot
     * @param velocitySetpoint
     * @return chassis speeds to get to the waypoint
     */
    public ChassisSpeeds getGoalSpeeds(Pose2d robotPose, double velocitySetpoint) {
        this.currentPose = robotPose;
        
        if (currentTrajectory.isCurrentEndPoint()) {
            return PoseToPoseControl();
        } else {
            return VelocityHeadingControl(velocitySetpoint);

        }
    }

    /**
     * 
     * @param currentPose
     * @return chassis speeds using normal pose to pose method
     */
    public ChassisSpeeds PoseToPoseControl() {
        Pose2d targetPose = currentTrajectory.getCurrentWaypoint().getPose();

        double vx = vxSlewLimiter.calculate(pxController.calculate(currentPose.getX(), targetPose.getX()));
        double vy = vySlewLimiter.calculate(pyController.calculate(currentPose.getY(), targetPose.getY()));
        double omega = omegaSlewLimiter.calculate(thetaController.calculate(currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()));

        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * 
     * @param currentPose
     * @param velocitySetpoint
     * @return chassis speeds using constant velocity
     */
    public ChassisSpeeds VelocityHeadingControl(double velocitySetpoint) {
        if (inSetpointRadius()) {
            currentTrajectory.nextWaypoint();
        }

    }

    // *************** Util Methods ***************
    /**
     * 
     * @param currentPose
     * @return distance of robot to the next waypoint
     */
    private double getDistanceToWaypoint() {
        Pose2d waypoint = currentTrajectory.getCurrentWaypoint().getPose();
        double dx = waypoint.getX() - currentPose.getX();
        double dy = waypoint.getY() - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public boolean inSetpointRadius() {
        return getDistanceToWaypoint() < currentTrajectory.getCurrentWaypoint().getEndRadius();
    }

    /**
     * 
     * @param currentPose
     * @return angular velocity for robot to get to the next waypoint
     */
    private double getAngleToWaypoint() {
        return 0.1;
    }

    // *************** Other Methods ***************

    /**
     * 
     * @param currentPose
     * @return true when the pid is settled
     */
    public boolean isSettled() {
        return pxController.atSetpoint() && pyController.atSetpoint() && thetaController.atSetpoint()
                && currentTrajectory.isCurrentEndPoint();
    }

    /**
     * 
     * @param newTrajectory set current trajectory to the new one
     */
    public void setTrajectory(P2PTrajectory newTrajectory) {
        this.currentTrajectory = newTrajectory;
    }

    // *************** Configs ***************

    public void setPositionPID(double kP, double kI, double kD) {
        pxController.setPID(kP, kI, kD);
        pyController.setPID(kP, kI, kD);
    }

    public void setThetaPID(double kP, double kI, double kD) {
        thetaController.setPID(kP, kI, kD);
    }

    public void setVelocitySlewRate(double rate) {
        vxSlewLimiter = new SlewRateLimiter(rate);
        vySlewLimiter = new SlewRateLimiter(rate);
    }

    public void setOmegaSlewRate(double rate) {
        vxSlewLimiter = new SlewRateLimiter(rate);
        vySlewLimiter = new SlewRateLimiter(rate);
    }

}