package frc.robot.TheorySwerveLib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/*
 * Extend this class to implement soecific functionality
 */

public abstract class TheoryModule {

    // region ************* Variable Declarations *************

    private SwerveModuleState desiredState;

    // endregion

    // Constructor
    protected TheoryModule() {
        desiredState = new SwerveModuleState();
    }

    // region ************* Methods *************

    // Motor IO: implement these methods for functionality
    public abstract double getAngle();

    public abstract double getAngularVelocity();

    public abstract double getWheelPositionRaw();

    public abstract double getWheelVelocityRaw();

    protected abstract void setTurnClosedLoop(double setpoint);

    protected abstract void setWheelClosedLoop(double setpoint);

    public abstract void setWheelBrake();

    public abstract void setTurnBrake();

    public abstract void setWheelCoast();

    public abstract void setTurnCoast();

    public abstract void setWheelRamp(double time);

    // Encoder Measurements

    /**
     * 
     * @return distance wheel traveled (meters)
     */
    public double getWheelPosition() {
        return wheelTicksToWheelRev(getWheelPositionRaw()) * ModuleConstants.WHEEL_DIAMETER * Math.PI;
    };

    /**
     * 
     * @return wheel velocity in m/s
     */
    public double getWheelVelocity() {
        return wheelTicksToWheelRev(getWheelVelocityRaw() * (1000 / ModuleConstants.WHEEL_VELOCITY_MEASURE_PERIOD_MS))
                * ModuleConstants.WHEEL_DIAMETER * Math.PI;
    };

    /**
     * 
     * @return rotation of wheel in degrees
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    // Module State

    public void setModuleState(SwerveModuleState state) {
        // dampen speed if angle error is large
        double angError = state.angle.minus(getRotation2d()).getRadians();
        double factor = Math.pow(Math.cos(angError), 2.0);
        SwerveModuleState dampState = new SwerveModuleState(state.speedMetersPerSecond * factor, state.angle);

        // optimize angle setpoint for shortest travel
        SwerveModuleState.optimize(dampState, getRotation2d());
        desiredState = dampState;

        // set power/setpoint to motors
        setTurnClosedLoop(desiredState.angle.getDegrees());
        setWheelClosedLoop(desiredState.speedMetersPerSecond);
    }

    /**
     * 
     * @return current module state as a SwerveModuleState
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getWheelVelocity(), getRotation2d());
    }

    /**
     * 
     * @return desired state of the module as a SwerveModuleState
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * 
     * @return current module state as a SwerveModuleState
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getWheelPosition(), getRotation2d());
    }

    // Util
    /**
     * 
     * @return revolutions of the wheel motor from encoder ticks
     */
    protected double wheelTicksToWheelRev(double ticks) {
        return ticks / ModuleConstants.WHEEL_TICKS_PER_REV / ModuleConstants.WHEEL_RATIO;
    }

    /**
     * 
     * @return revolutions of the turn motor from encoder ticks
     */
    protected double turnTicksToWheelRev(double ticks) {
        return ticks / ModuleConstants.TURN_TICKS_PER_REV / ModuleConstants.TURN_RATIO;
    }

    // endregion

}// TheoryModule
