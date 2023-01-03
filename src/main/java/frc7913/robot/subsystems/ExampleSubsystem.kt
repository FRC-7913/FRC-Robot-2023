package frc7913.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase

// By making a subsystem an object, we ensure there is only ever one instance of it
object ExampleSubsystem : SubsystemBase() {
    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    fun exampleAction() {
        // This action is called by the ExampleCommand
        println("ExampleSubsystem.exampleAction has been called")
    }
}
