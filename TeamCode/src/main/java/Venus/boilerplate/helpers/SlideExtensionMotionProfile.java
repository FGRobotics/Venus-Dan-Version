package Venus.boilerplate.helpers;

public class SlideExtensionMotionProfile {

    private final double maxVelocity;
    private final double minVelocity;
    private final double accelerationRate;  // units/sec^2
    private final double decelerationRate;  // units/sec^2
    private final double positionTolerance;

    public SlideExtensionMotionProfile(double maxVelocity, double minVelocity, double accelerationRate, double decelerationRate, double positionTolerance) {
        this.maxVelocity = maxVelocity;
        this.minVelocity = minVelocity;
        this.accelerationRate = (maxVelocity - minVelocity) / accelerationRate;
        this.decelerationRate = (maxVelocity - minVelocity) / decelerationRate;
        this.positionTolerance = positionTolerance;
    }

    /**
     * Main velocity profile logic.
     * @param currentPosition Encoder units
     * @param targetPosition Encoder units
     * @return Velocity setpoint in units per second
     */
    public double computeVelocity(int currentPosition, int targetPosition) {
        double remaining = targetPosition - currentPosition;
        double absRemaining = Math.abs(remaining);

        // Stop early if inside tolerance
        if (absRemaining <= positionTolerance) return (minVelocity/2) * Math.signum(remaining);

        // Calculate braking (deceleration) distance: d = (v^2) / (2a)
        double stoppingDistance = (maxVelocity * maxVelocity) / (2 * decelerationRate);

        // Phase 1: Accelerate
        if (absRemaining > stoppingDistance) {
            // Use quadratic curve for velocity ramp-up
            double velocity = Math.sqrt(2 * accelerationRate * absRemaining);
            velocity = Math.min(velocity, maxVelocity);
            velocity = Math.max(velocity, minVelocity);
            return velocity * Math.signum(remaining); // Keep direction
        }

        // Phase 2: Decelerate
        else {
            // Use quadratic slowdown
            double velocity = Math.sqrt(2 * decelerationRate * absRemaining);
            velocity = Math.max(velocity, minVelocity);
            return velocity * Math.signum(remaining);
        }
    }
}
