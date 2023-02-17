package frc7913.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.SoftLimitDirection
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import com.revrobotics.SparkMaxRelativeEncoder
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc7913.lib.PIDGains
import frc7913.robot.ArmConstants

object ArmSubsystem : SubsystemBase() {

    private val m_motor = CANSparkMax(ArmConstants.kArmCanId, CANSparkMaxLowLevel.MotorType.kBrushless)

    init {
        m_motor.inverted = false
        m_motor.setSmartCurrentLimit(ArmConstants.kCurrentLimit)
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, true)
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true)
        m_motor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kSoftLimitForward.toFloat())
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kSoftLimitReverse.toFloat())
    }

    private val m_encoder: RelativeEncoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42)

    init {
        m_encoder.positionConversionFactor = ArmConstants.kPositionFactor
        m_encoder.velocityConversionFactor = ArmConstants.kVelocityFactor
    }

    private val m_controller: SparkMaxPIDController = m_motor.pidController

    var m_setpoint: Double = ArmConstants.Positions.Home.position
        set(value) {
            if (value != m_setpoint) {
                field = value
                updateMotionProfile()
            }
        }
    private var m_profile: TrapezoidProfile? = null
    private val m_timer: Timer
    private var targetState: TrapezoidProfile.State? = null
    private var feedforward = 0.0
    private var manualValue = 0.0

    /** Creates a new ArmSubsystem.  */
    init {
        PIDGains.setSparkMaxGains(m_controller, ArmConstants.kArmPositionGains)
        m_motor.burnFlash()

        m_timer = Timer()
        m_timer.start()
        m_timer.reset()

        updateMotionProfile()
    }

    fun setTargetPosition(setpoint: ArmConstants.Positions) {
        m_setpoint = setpoint.position
    }

    private fun updateMotionProfile() {
        val state = TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity())
        val goal = TrapezoidProfile.State(m_setpoint, 0.0)
        m_profile = TrapezoidProfile(ArmConstants.kArmMotionConstraint, goal, state)
        m_timer.reset()
    }

    fun runAutomatic() {
        val elapsedTime = m_timer.get()
        targetState = if (m_profile!!.isFinished(elapsedTime)) {
            TrapezoidProfile.State(m_setpoint, 0.0)
        } else {
            m_profile!!.calculate(elapsedTime)
        }
        feedforward = ArmConstants.kArmFeedforward.calculate(
            m_encoder.getPosition() + ArmConstants.kArmZeroCosineOffset,
            targetState!!.velocity
        )
        m_controller.setReference(targetState!!.position, CANSparkMax.ControlType.kPosition, 0, feedforward)
    }

    fun runManual(_power: Double) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
        m_setpoint = m_encoder.getPosition()
        targetState = TrapezoidProfile.State(m_setpoint, 0.0)
        m_profile = TrapezoidProfile(ArmConstants.kArmMotionConstraint, targetState, targetState)
        // update the feedforward variable with the newly zero target velocity
        feedforward = ArmConstants.kArmFeedforward.calculate(
            m_encoder.getPosition() + ArmConstants.kArmZeroCosineOffset,
            targetState!!.velocity
        )
        m_motor.set(_power + feedforward / 12.0)
        manualValue = _power
    }

    override fun periodic() { // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() { // This method will be called once per scheduler run during simulation
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addDoubleProperty("Final Setpoint", { m_setpoint }, null)
        builder.addDoubleProperty("Position", { m_encoder.getPosition() }, null)
        builder.addDoubleProperty("Applied Output", { m_motor.getAppliedOutput() }, null)
        builder.addDoubleProperty("Elapsed Time", { m_timer.get() }, null)
    /*builder.addDoubleProperty("Target Position", () -> targetState.position, null);
    builder.addDoubleProperty("Target Velocity", () -> targetState.velocity, null);*/builder.addDoubleProperty(
            "Feedforward",
            { feedforward }, null
        )
        builder.addDoubleProperty("Manual Value", { manualValue }, null)
        // builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
        // builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
        // addChild("Controller", m_controller);
    }
}
