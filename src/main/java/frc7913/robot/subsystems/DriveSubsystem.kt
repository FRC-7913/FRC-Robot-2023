package frc7913.robot.subsystems

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.SubsystemBase

object DriveSubsystem : SubsystemBase() {

    private val leftLead = Spark(2)
    private val leftFollow = Spark(3)
    private val leftSide = MotorControllerGroup(leftLead, leftFollow)

    private val rightLead = Spark(0)
    private val rightFollow = Spark(1)
    private val rightSide = MotorControllerGroup(rightLead, rightFollow)

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
