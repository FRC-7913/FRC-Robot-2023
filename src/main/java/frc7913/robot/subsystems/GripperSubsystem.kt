package frc7913.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.SoftLimitDirection
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import com.revrobotics.SparkMaxRelativeEncoder
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc7913.lib.PIDGains
import frc7913.robot.GripperConstants

object GripperSubsystem : SubsystemBase() {

    private val motor = CANSparkMax(GripperConstants.gripperCanId, CANSparkMaxLowLevel.MotorType.kBrushless)

    init {
        motor.inverted = false
        motor.setSmartCurrentLimit(GripperConstants.currentLimit)
        motor.enableSoftLimit(SoftLimitDirection.kForward, true)
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true)
        motor.setSoftLimit(SoftLimitDirection.kForward, GripperConstants.softLimitForward.toFloat())
        motor.setSoftLimit(SoftLimitDirection.kReverse, GripperConstants.softLimitReverse.toFloat())
    }

    private val encoder: RelativeEncoder =
        motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42)

    private val controller: SparkMaxPIDController = motor.pidController
    private var setpoint: Double
    private var prevSetpoint = 0.0

    /** Creates a new ExampleSubsystem.  */
    init {
        PIDGains.setSparkMaxGains(controller, GripperConstants.gripperPositionPIDGains)
        motor.burnFlash()
        setpoint = GripperConstants.closePosition
    }

    val isSafe: Boolean
        get() = encoder.position > GripperConstants.safePosition

    fun openGripper() {
        setpoint = GripperConstants.openPosition
    }

    fun closeGripper() {
        setpoint = GripperConstants.closePosition
    }

    override fun periodic() { // This method will be called once per scheduler run
        if (setpoint != prevSetpoint) {
            controller.setReference(setpoint, CANSparkMax.ControlType.kPosition)
        }
        prevSetpoint = setpoint
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addDoubleProperty(
            "Setpoint",
            { setpoint }
        ) { `val`: Double -> setpoint = `val` }
        builder.addDoubleProperty("Position", { encoder.position }, null)
    }
}
