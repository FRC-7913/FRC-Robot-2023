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

object PortConstants {
    const val leftMotors = 0

    const val rightMotors = 1

    const val armMotor = 2

    const val gripperMotor = 3

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

object ArmConstants {
    const val kArmCanId = 5
    const val kArmInverted = false
    const val kCurrentLimit = 40
    const val kSoftLimitReverse = 0.0
    const val kSoftLimitForward = 4.6
    const val kArmGearRatio = 1.0 / (48.0 * 4.0)
    const val kPositionFactor =
        kArmGearRatio * 2.0 * Math.PI // multiply SM value by this number and get arm position in radians
    const val kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0
    const val kArmFreeSpeed = 5676.0 * kVelocityFactor
    const val kArmZeroCosineOffset =
        -Math.PI / 6 // radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
    val kArmFeedforward = ArmFeedforward(0.0, 0.4, 12.0 / kArmFreeSpeed, 0.0)
    val kArmPositionGains = PIDGains(0.6, 0.0, 0.0)
    val kArmMotionConstraint = TrapezoidProfile.Constraints(2.0, 2.0)

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
    const val kGripperCanId = 6
    const val kSoftLimitReverse = -34.0
    const val kSoftLimitForward = 5.0
    const val kClosePosition = 0.0
    const val kOpenPosition = -34.0
    const val kSafePosition = -29.0
    const val kCurrentLimit = 10
}
