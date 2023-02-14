package frc7913.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc7913.robot.LimelightHelpers

object Limelight : SubsystemBase() {

    /**
     * Returns a [Pose2d] representing the robot pose relative to the field.
     * (0,0) is the center of the field.
     * X+ is towards red alliance grid, Y+ is toward loading zone side of field.
     *
     * @return the robot position relative to the center of the field
     */
    fun getRobotPose(): Pose2d {
        val rawBotPose = LimelightHelpers.getBotpose("limelight")
        return Pose2d(
            rawBotPose[0],
            rawBotPose[1],
            Rotation2d(
                Math.toRadians(rawBotPose[5])
            )
        )
    }

    /**
     * Gets the currently in-view AprilTag.
     *
     * @return an [AprilTag] object of the currently-targeted AprilTag
     */
    fun getCurrentInViewTag(): AprilTag {
        return AprilTag.fromId(
            LimelightHelpers.getFiducialID("limelight").toInt()
        )
    }

    /**
     * Returns a relative [Pose2d] for the currently targeted tag.
     * This is in robot space, where X+ is forward and Y+ is right
     *
     * @return a [Pose2d] representing
     */
    fun getRelativePositionOfTag(): Pose2d {
        val rawTargetOffset = LimelightHelpers.getTargetPose_RobotSpace("limelight")
        return Pose2d(
            rawTargetOffset[0],
            rawTargetOffset[1],
            Rotation2d.fromDegrees(
                rawTargetOffset[5]
            )
        )
    }
}

/**
 * Represents an AprilTag on the field.
 * Use fromID() to get the correct tag based on a tag id.
 * @param id the Int id of the tag
 */
enum class AprilTag(val id: Int, val pose: Pose2d) {
    RedGridLeft(
        3,
        Pose2d(7.24310, 0.41621, Rotation2d.fromDegrees(0.0))
    ),
    RedGridCoop(
        2,
        Pose2d(7.24310, -1.26019, Rotation2d.fromDegrees(0.0))
    ),
    RedGridRight(
        1,
        Pose2d(7.24310, -2.93659, Rotation2d.fromDegrees(0.0))
    ),

    BlueGridLeft(
        8,
        Pose2d(-7.24310, -2.93659, Rotation2d.fromDegrees(180.0))
    ),
    BlueGridCoop(
        7,
        Pose2d(-7.24310, -1.26019, Rotation2d.fromDegrees(180.0))
    ),
    BlueGridRight(
        6,
        Pose2d(-7.24310, 0.41621, Rotation2d.fromDegrees(180.0))
    ),

    RedLoading(
        5,
        Pose2d(-7.90832, 2.74161, Rotation2d.fromDegrees(0.0))
    ),
    BlueLoading(
        4,
        Pose2d(7.90832, 2.74141, Rotation2d.fromDegrees(180.0))
    );

    companion object {
        fun fromId(id: Int): AprilTag {
            for (tag in values()) {
                if (tag.id == id) {
                    return tag
                }
            }
            println("AprilTag from id $id was requested, but that doesn't exist (returning id 1)")
            return fromId(1)
        }
    }
}
