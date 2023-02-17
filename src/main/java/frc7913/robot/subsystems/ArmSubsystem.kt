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

    private val motor = CANSparkMax(ArmConstants.kArmCanId, CANSparkMaxLowLevel.MotorType.kBrushless)

    init {
        motor.inverted = false
        motor.setSmartCurrentLimit(ArmConstants.kCurrentLimit)
        motor.enableSoftLimit(SoftLimitDirection.kForward, true)
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true)
        motor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kSoftLimitForward.toFloat())
        motor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kSoftLimitReverse.toFloat())
    }

    private val encoder: RelativeEncoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42)

    init {
        encoder.positionConversionFactor = ArmConstants.kPositionFactor
        encoder.velocityConversionFactor = ArmConstants.kVelocityFactor
    }

    private val motorController: SparkMaxPIDController = motor.pidController

    var armSetpoint: Double = ArmConstants.Positions.Home.position
        set(value) {
            if (value != armSetpoint) {
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
        PIDGains.setSparkMaxGains(motorController, ArmConstants.kArmPositionGains)
        motor.burnFlash()

        timer = Timer()
        timer.start()
        timer.reset()

        updateMotionProfile()
    }

    fun setTargetPosition(setpoint: ArmConstants.Positions) {
        armSetpoint = setpoint.position
    }

    private fun updateMotionProfile() {
        val state = TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity())
        val goal = TrapezoidProfile.State(armSetpoint, 0.0)
        profile = TrapezoidProfile(ArmConstants.kArmMotionConstraint, goal, state)
        timer.reset()
    }

    fun runAutomatic() {
        val elapsedTime = timer.get()
        targetState = if (profile!!.isFinished(elapsedTime)) {
            TrapezoidProfile.State(armSetpoint, 0.0)
        } else {
            profile!!.calculate(elapsedTime)
        }
        feedforward = ArmConstants.kArmFeedforward.calculate(
            encoder.getPosition() + ArmConstants.kArmZeroCosineOffset,
            targetState!!.velocity
        )
        motorController.setReference(targetState!!.position, CANSparkMax.ControlType.kPosition, 0, feedforward)
    }

    fun runManual(_power: Double) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
        armSetpoint = encoder.getPosition()
        targetState = TrapezoidProfile.State(armSetpoint, 0.0)
        profile = TrapezoidProfile(ArmConstants.kArmMotionConstraint, targetState, targetState)
        // update the feedforward variable with the newly zero target velocity
        feedforward = ArmConstants.kArmFeedforward.calculate(
            encoder.getPosition() + ArmConstants.kArmZeroCosineOffset,
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
        builder.addDoubleProperty("Final Setpoint", { armSetpoint }, null)
        builder.addDoubleProperty("Position", { encoder.getPosition() }, null)
        builder.addDoubleProperty("Applied Output", { motor.getAppliedOutput() }, null)
        builder.addDoubleProperty("Elapsed Time", { timer.get() }, null)
    /*builder.addDoubleProperty("Target Position", () -> targetState.position, null);
    builder.addDoubleProperty("Target Velocity", () -> targetState.velocity, null);*/builder.addDoubleProperty(
            "Feedforward",
            { feedforward }, null
        )
        builder.addDoubleProperty("Manual Value", { manualValue }, null)
        // builder.addDoubleProperty("Setpoint", () -> setpoint, (val) -> setpoint = val);
        // builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
        // addChild("Controller", motorController);
    }
}
