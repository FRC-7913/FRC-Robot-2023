package frc7913.robot

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object PortConstants {
    const val leftFrontMotor = 2
    const val leftRearMotor = 3

    const val rightFrontMotor = 0
    const val rightRearMotor = 1

    // Encoder A and Encoder B ports (in that order) seem to work with REV On-Shaft encoders
    val leftEncoder: Pair<Int, Int> = Pair(1, 2)
    val rightEncoder: Pair<Int, Int> = Pair(5, 6)
}

object DriveConstants {
    const val leftMotorsInverted = true
    const val rightMotorsInverted = false

    const val leftEncoderInverted = false // TODO: Actually set this
    const val rightEncoderInverted = false // TODO: Actually set this

    // The robot moves forward 1 foot per encoder rotation
    // There are 256 pulses per encoder rotation
    const val encoderDistancePerPulse: Double = 1.0 / 256.0 // TODO
}

// Many of these should be retrieved from the Robot Characterization Toolsuite
object CharacterizationConstants {
    const val volts: Double = 0.22 // TODO: Actually get value
    const val voltSecondsPerMeter: Double = 1.98 // TODO: Actually get value
    const val voltSecondsSquaredPerMeter: Double = 0.2 // TODO: Actually get value

    const val pDriveVel: Double = 8.5 // TODO: Actually get value

    const val trackWidthMeters: Double = 0.69 // TODO: Actually measure
}

object TrajectoryConstants {
    const val maxSpeed: Double = 3.0
    const val maxAcceleration: Double = 1.0
}

object RamseteConstants {
    // May need tuning, these are defaults that work well for most robots
    const val b: Double = 2.0
    const val zeta: Double = 0.7
}
