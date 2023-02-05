package frc7913.robot.subsystems

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.ADXRS450_Gyro
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc7913.robot.DriveConstants
import frc7913.robot.PortConstants

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
}
