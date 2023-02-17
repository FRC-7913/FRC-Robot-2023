package frc7913.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.SoftLimitDirection
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import com.revrobotics.SparkMaxRelativeEncoder
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc7913.robot.GripperConstants

object GripperSubsystem : SubsystemBase() {
    private val motor: CANSparkMax
    private val encoder: RelativeEncoder
    private val motorController: SparkMaxPIDController
    private var armSetpoint: Double
    private var prevSetpoint = 0.0

    /** Creates a new ExampleSubsystem.  */
    init {
        motor = CANSparkMax(GripperConstants.kGripperCanId, CANSparkMaxLowLevel.MotorType.kBrushless)
        motor.inverted = false
        motor.setSmartCurrentLimit(GripperConstants.kCurrentLimit)
        motor.enableSoftLimit(SoftLimitDirection.kForward, true)
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true)
        motor.setSoftLimit(SoftLimitDirection.kForward, GripperConstants.kSoftLimitForward.toFloat())
        motor.setSoftLimit(SoftLimitDirection.kReverse, GripperConstants.kSoftLimitReverse.toFloat())
        encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42)
        motorController = motor.pidController
        // PIDGains.setSparkMaxGains(motorController, GripperConstants.kPositionPIDGains)
        motor.burnFlash()
        armSetpoint = GripperConstants.kClosePosition
    }

    val isSafe: Boolean
        get() = encoder.position > GripperConstants.kSafePosition

    fun openGripper() {
        armSetpoint = GripperConstants.kOpenPosition
    }

    fun closeGripper() {
        armSetpoint = GripperConstants.kClosePosition
    }

    override fun periodic() { // This method will be called once per scheduler run
        if (armSetpoint != prevSetpoint) {
            motorController.setReference(armSetpoint, CANSparkMax.ControlType.kPosition)
        }
        prevSetpoint = armSetpoint
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addDoubleProperty(
            "Setpoint",
            { armSetpoint }
        ) { `val`: Double -> armSetpoint = `val` }
        builder.addDoubleProperty("Position", { encoder.position }, null)
    }
}
