package frc7913.robot

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc7913.lib.PIDGains

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object DriveConstants {

    const val leftMotorsPort = 0

    const val rightMotorsPort = 1

    // Encoder A and Encoder B ports (in that order) seem to work with REV On-Shaft encoders
    val leftEncoderPorts: Pair<Int, Int> = Pair(1, 2)
    val rightEncoderPorts: Pair<Int, Int> = Pair(5, 6)

    const val leftMotorsInverted = true
    const val rightMotorsInverted = false

    const val leftEncoderInverted = false // TODO: Actually set this
    const val rightEncoderInverted = false // TODO: Actually set this

    // The robot moves forward 1 foot per encoder rotation
    // There are 256 pulses per encoder rotation
    const val encoderDistancePerPulse: Double = 1.0 / 256.0 // TODO

    // These values come from the Robot Characterization Toolsuite
    const val volts: Double = 0.22 // TODO: Actually get value
    const val voltSecondsPerMeter: Double = 1.98 // TODO: Actually get value
    const val voltSecondsSquaredPerMeter: Double = 0.2 // TODO: Actually get value

    const val pDriveVel: Double = 8.5 // TODO: Actually get value

    const val trackWidthMeters: Double = 0.69 // TODO: Actually measure

    const val maxSpeed: Double = 3.0
    const val maxAcceleration: Double = 1.0

    // May need tuning, these are defaults that work well for most robots
    const val ramseteB: Double = 2.0
    const val ramseteZeta: Double = 0.7
}

object ArmConstants {
    const val armCanId = 5

    const val armInverted = false

    const val currentLimit = 40
    const val softLimitReverse = 0.0
    const val softLimitForward = 4.6

    const val armGearRatio = 1.0 / (48.0 * 4.0)

    const val positionFactor =
        armGearRatio * 2.0 * Math.PI // multiply SM value by this number and get arm position in radians
    const val velocityFactor = armGearRatio * 2.0 * Math.PI / 60.0

    const val armFreeSpeed = 5676.0 * velocityFactor

    const val armZeroCosineOffset =
        -Math.PI / 6 // radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
    val armFeedforward = ArmFeedforward(0.0, 0.4, 12.0 / armFreeSpeed, 0.0)

    val armPositionPIDGains = PIDGains(0.6, 0.0, 0.0)

    val armMotionConstraint = TrapezoidProfile.Constraints(2.0, 2.0)

    enum class Positions(val position: Double) {
        Home(0.0),
        Scoring(3.05),
        Intake(4.52),
        Feeder(2.95),

        ;
        companion object {
            fun getFromPosition(from: Double): Positions? {
                for (at in values()) {
                    if (at.position == from)
                        return at
                }
                return null
            }
        }
    }
}

object GripperConstants {
    const val gripperCanId = 6

    const val softLimitReverse = -34.0
    const val softLimitForward = 5.0

    const val closePosition = 0.0
    const val openPosition = -34.0
    const val safePosition = -29.0

    const val currentLimit = 10

    val gripperPositionPIDGains = PIDGains(0.2, 0.0, 0.0)
}
