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

    private val motor = CANSparkMax(ArmConstants.armCanId, CANSparkMaxLowLevel.MotorType.kBrushless)

    init {
        motor.inverted = false
        motor.setSmartCurrentLimit(ArmConstants.currentLimit)
        motor.enableSoftLimit(SoftLimitDirection.kForward, true)
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true)
        motor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.softLimitForward.toFloat())
        motor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.softLimitReverse.toFloat())
    }

    private val encoder: RelativeEncoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42)

    init {
        encoder.positionConversionFactor = ArmConstants.positionFactor
        encoder.velocityConversionFactor = ArmConstants.velocityFactor
    }

    private val controller: SparkMaxPIDController = motor.pidController

    var setpoint: Double = ArmConstants.Positions.Home.position
        set(value) {
            if (value != setpoint) {
                field = value
                updateMotionProfile()
            }
        }
    private var profile: TrapezoidProfile? = null
    private val timer: Timer
    private var targetState: TrapezoidProfile.State? = null
    private var feedforward = 0.0
    private var manualValue = 0.0

    /** Creates a new ArmSubsystem.  */
    init {
        PIDGains.setSparkMaxGains(controller, ArmConstants.armPositionPIDGains)
        motor.burnFlash()

        timer = Timer()
        timer.start()
        timer.reset()

        updateMotionProfile()
    }

    fun setTargetPosition(setpoint: ArmConstants.Positions) {
        this.setpoint = setpoint.position
    }

    private fun updateMotionProfile() {
        val state = TrapezoidProfile.State(encoder.position, encoder.velocity)
        val goal = TrapezoidProfile.State(setpoint, 0.0)
        profile = TrapezoidProfile(ArmConstants.armMotionConstraint, goal, state)
        timer.reset()
    }

    fun runAutomatic() {
        val elapsedTime = timer.get()
        targetState = if (profile!!.isFinished(elapsedTime)) {
            TrapezoidProfile.State(setpoint, 0.0)
        } else {
            profile!!.calculate(elapsedTime)
        }
        feedforward = ArmConstants.armFeedforward.calculate(
            encoder.position + ArmConstants.armZeroCosineOffset,
            targetState!!.velocity
        )
        controller.setReference(targetState!!.position, CANSparkMax.ControlType.kPosition, 0, feedforward)
    }

    fun runManual(_power: Double) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
        setpoint = encoder.position
        targetState = TrapezoidProfile.State(setpoint, 0.0)
        profile = TrapezoidProfile(ArmConstants.armMotionConstraint, targetState, targetState)
        // update the feedforward variable with the newly zero target velocity
        feedforward = ArmConstants.armFeedforward.calculate(
            encoder.position + ArmConstants.armZeroCosineOffset,
            targetState!!.velocity
        )
        motor.set(_power + feedforward / 12.0)
        manualValue = _power
    }

    override fun periodic() { // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() { // This method will be called once per scheduler run during simulation
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addDoubleProperty("Final Setpoint", { setpoint }, null)
        builder.addDoubleProperty("Position", { encoder.position }, null)
        builder.addDoubleProperty("Applied Output", { motor.appliedOutput }, null)
        builder.addDoubleProperty("Elapsed Time", { timer.get() }, null)
    /*builder.addDoubleProperty("Target Position", () -> targetState.position, null);
    builder.addDoubleProperty("Target Velocity", () -> targetState.velocity, null);*/builder.addDoubleProperty(
            "Feedforward",
            { feedforward }, null
        )
        builder.addDoubleProperty("Manual Value", { manualValue }, null)
        // builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
        // builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
        // addChild("Controller", controller);
    }
}
