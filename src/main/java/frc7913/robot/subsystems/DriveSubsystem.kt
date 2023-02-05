package frc7913.robot.subsystems

import edu.wpi.first.wpilibj.drive.DifferentialDrive
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
}
