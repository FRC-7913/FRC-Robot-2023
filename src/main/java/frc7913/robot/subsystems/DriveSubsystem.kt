package frc7913.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj.ADXRS450_Gyro
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc7913.robot.CharacterizationConstants
import frc7913.robot.DriveConstants
import frc7913.robot.PortConstants
import frc7913.robot.RamseteConstants
import frc7913.robot.TrajectoryConstants

object DriveSubsystem : SubsystemBase() {

    private val leftSide = MotorControllerGroup(
        Spark(PortConstants.leftFrontMotor), // Front
        Spark(PortConstants.leftRearMotor) // Back
    )

    private val rightSide = MotorControllerGroup(
        Spark(PortConstants.rightFrontMotor), // Front
        Spark(PortConstants.rightRearMotor) // Back
    )

    init {
        leftSide.inverted = DriveConstants.leftMotorsInverted
        rightSide.inverted = DriveConstants.rightMotorsInverted
    }

    val driveTrain = DifferentialDrive(leftSide, rightSide)

    init {
        driveTrain.isSafetyEnabled = true
        driveTrain.expiration = 0.1
    }

    private val leftEncoder = Encoder(
        PortConstants.leftEncoder.first,
        PortConstants.leftEncoder.second,
        DriveConstants.leftEncoderInverted,
    )
    private val rightEncoder = Encoder(
        PortConstants.rightEncoder.first,
        PortConstants.rightEncoder.second,
        DriveConstants.rightEncoderInverted,
    )

    private val gyro: Gyro = ADXRS450_Gyro()

    init {
        leftEncoder.distancePerPulse = DriveConstants.encoderDistancePerPulse
        rightEncoder.distancePerPulse = DriveConstants.encoderDistancePerPulse

        resetEncoders()
    }

    private val odometry = DifferentialDriveOdometry(gyro.rotation2d, leftEncoder.distance, rightEncoder.distance)

    override fun periodic() {
        odometry.update(
            gyro.rotation2d, leftEncoder.distance, rightEncoder.distance
        )
    }

    /**
     * Navigates the robot along a given path
     *
     * @param start the starting position and orientation
     * @param intermediate points for the robot to meet in between
     * @param end the final position and orientation for the robot at the end of the path
     */
    fun navigate(
        start: Pose2d,
        vararg intermediate: Translation2d,
        end: Pose2d,
        voltageConstraints: DifferentialDriveVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforward(
                CharacterizationConstants.volts,
                CharacterizationConstants.voltSecondsPerMeter,
                CharacterizationConstants.voltSecondsSquaredPerMeter
            ),
            DifferentialDriveKinematics(CharacterizationConstants.trackWidthMeters),
            10.0
        ),
        config: TrajectoryConfig = TrajectoryConfig(
            TrajectoryConstants.maxSpeed,
            TrajectoryConstants.maxAcceleration
        ).setKinematics(DifferentialDriveKinematics(CharacterizationConstants.trackWidthMeters)),
    ): Command {
        config.addConstraint(voltageConstraints)

        val trajectory = TrajectoryGenerator.generateTrajectory(
            start,
            intermediate.toList(),
            end,
            config
        )

        val ramseteCommand = RamseteCommand(
            trajectory,
            ::pose,
            RamseteController(RamseteConstants.b, RamseteConstants.zeta),
            SimpleMotorFeedforward(
                CharacterizationConstants.volts,
                CharacterizationConstants.voltSecondsPerMeter,
                CharacterizationConstants.voltSecondsSquaredPerMeter
            ),
            DifferentialDriveKinematics(CharacterizationConstants.trackWidthMeters),
            ::wheelSpeed,
            PIDController(CharacterizationConstants.pDriveVel, 0.0, 0.0),
            PIDController(CharacterizationConstants.pDriveVel, 0.0, 0.0),
            ::tankDriveVolts,
            DriveSubsystem
        )

        resetOdometry(start)

        // Stop after the ramsete command finishes
        return ramseteCommand.andThen({ tankDriveVolts(0.0, 0.0) }, DriveSubsystem)
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        leftSide.setVoltage(leftVolts)
        rightSide.setVoltage(rightVolts)
        driveTrain.feed()
    }

    /**The currently estimated pose of the robot*/
    val pose: Pose2d get() = odometry.poseMeters

    /**The current wheel speed*/
    val wheelSpeed get() = DifferentialDriveWheelSpeeds(leftEncoder.rate, rightEncoder.rate)

    /**The average encoder distance*/
    val averageEncoderDistance get() = (leftEncoder.distance + rightEncoder.distance) / 2

    /**The heading of the robot in degrees, from -180 to 180*/
    val heading get() = gyro.rotation2d.degrees

    /**The turn rate of the robot*/
    val turnRate get() = -gyro.rate

    /**
     * Runs two print commands giving the distance and rate of each encoder
     */
    fun printEncoders() {
        println("Left distance is ${ leftEncoder.distance }, rate is ${leftEncoder.rate}")
        println("Right distance is ${ rightEncoder.distance }, rate is ${rightEncoder.rate}")
    }

    /**
     * Resets odometry to the specified pose
     * @param pose The pose to reset to
     */
    fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        odometry.resetPosition(
            gyro.rotation2d, leftEncoder.distance, rightEncoder.distance, pose
        )
    }

    /**Resets encoders to a currently read position of 0*/
    fun resetEncoders() {
        leftEncoder.reset()
        rightEncoder.reset()
    }

    /**Zeros the heading of the robot*/
    fun zeroHeading() {
        gyro.reset()
    }
}
