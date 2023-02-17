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
    private val m_motor: CANSparkMax
    private val m_encoder: RelativeEncoder
    private val m_controller: SparkMaxPIDController
    private var m_setpoint: Double
    private var m_prevSetpoint = 0.0

    /** Creates a new ExampleSubsystem.  */
    init {
        m_motor = CANSparkMax(GripperConstants.kGripperCanId, CANSparkMaxLowLevel.MotorType.kBrushless)
        m_motor.inverted = false
        m_motor.setSmartCurrentLimit(GripperConstants.kCurrentLimit)
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, true)
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true)
        m_motor.setSoftLimit(SoftLimitDirection.kForward, GripperConstants.kSoftLimitForward.toFloat())
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, GripperConstants.kSoftLimitReverse.toFloat())
        m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42)
        m_controller = m_motor.pidController
        // PIDGains.setSparkMaxGains(m_controller, GripperConstants.kPositionPIDGains)
        m_motor.burnFlash()
        m_setpoint = GripperConstants.kClosePosition
    }

    val isSafe: Boolean
        get() = m_encoder.position > GripperConstants.kSafePosition

    fun openGripper() {
        m_setpoint = GripperConstants.kOpenPosition
    }

    fun closeGripper() {
        m_setpoint = GripperConstants.kClosePosition
    }

    override fun periodic() { // This method will be called once per scheduler run
        if (m_setpoint != m_prevSetpoint) {
            m_controller.setReference(m_setpoint, CANSparkMax.ControlType.kPosition)
        }
        m_prevSetpoint = m_setpoint
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addDoubleProperty(
            "Setpoint",
            { m_setpoint }
        ) { `val`: Double -> m_setpoint = `val` }
        builder.addDoubleProperty("Position", { m_encoder.position }, null)
    }
}
