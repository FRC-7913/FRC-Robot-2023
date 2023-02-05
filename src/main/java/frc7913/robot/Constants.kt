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

    val leftEncoder: Pair<Int, Int> = TODO("Left encoder ports not configured")
    val rightEncoder: Pair<Int, Int> = TODO("Right encoder ports not configured")
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

// TODO: Put this declaration in a more relevant place
data class LimelightTransform(
    val translationX: Double,
    val translationY: Double,
    val translationZ: Double,
    val rotationX: Double,
    val rotationY: Double,
    val rotationZ: Double,
) {
    constructor() : this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    fun toArray() = DoubleArray(6) {
        when (it) {
            0 -> translationX
            1 -> translationY
            2 -> translationZ
            3 -> rotationX
            4 -> rotationY
            5 -> rotationZ
            else -> 0.0 // This line shouldn't run, but is necessary for an exhaustive "when" statement
        }
    }

    companion object {
        infix fun from(array: DoubleArray) =
            LimelightTransform(
                array[0],
                array[1],
                array[2],
                array[3],
                array[4],
                array[5]
            )
    }
}
