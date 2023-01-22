package frc7913.robot.subsystems

import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX
import edu.wpi.first.wpilibj2.command.SubsystemBase

object DriveTrain : SubsystemBase() {

    private val leftLead = PWMVictorSPX(TODO("Front left drivetrain motor port unknown"))
    private val leftFollow = PWMVictorSPX(TODO("Rear left drivetrain motor port unknown"))
    private val leftSide = MotorControllerGroup(leftLead, leftFollow)

    private val rightLead = PWMVictorSPX(TODO("Front right drivetrain motor port unknown"))
    private val rightFollow = PWMVictorSPX(TODO("Rear right drivetrain motor port unknown"))
    private val rightSide = MotorControllerGroup(rightLead, rightFollow)

    init {
        leftSide.inverted = TODO("Unknown if left side of drivetrain should be inverted")
        rightSide.inverted = TODO("Unknown if right side of drivetrain should be inverted")
    }

    val driveTrain = DifferentialDrive(leftSide, rightSide)

    init {
        driveTrain.isSafetyEnabled = true
        driveTrain.expiration = 0.1
        driveTrain.setMaxOutput(0.6)
    }
}
