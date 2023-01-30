package frc7913.robot.subsystems

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.SubsystemBase

object DriveSubsystem : SubsystemBase() {

    private val leftSide = MotorControllerGroup(
        Spark(2), // Front
        Spark(3) // Back
    )

    private val rightSide = MotorControllerGroup(
        Spark(0), // Front
        Spark(1) // Back
    )

    init {
        leftSide.inverted = true
        rightSide.inverted = false
    }

    val driveTrain = DifferentialDrive(leftSide, rightSide)

    init {
        driveTrain.isSafetyEnabled = true
        driveTrain.expiration = 0.1
        driveTrain.setMaxOutput(0.6)
    }
}
