package frc7913.robot.subsystems

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc7913.robot.leftFrontMotorPort
import frc7913.robot.leftRearMotorPort
import frc7913.robot.rightFrontMotorPort
import frc7913.robot.rightRearMotorPort

object DriveSubsystem : SubsystemBase() {

    private val leftSide = MotorControllerGroup(
        Spark(leftFrontMotorPort), // Front
        Spark(leftRearMotorPort) // Back
    )

    private val rightSide = MotorControllerGroup(
        Spark(rightFrontMotorPort), // Front
        Spark(rightRearMotorPort) // Back
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
