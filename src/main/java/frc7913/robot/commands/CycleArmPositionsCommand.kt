package frc7913.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc7913.robot.ArmConstants
import frc7913.robot.subsystems.ArmSubsystem

fun CycleArmPositionsCommand(): Command = Commands.runOnce(
    {
        val currentPositionIndex = ArmConstants.Positions.values()
            .indexOf(
                ArmConstants.Positions.getFromPosition(ArmSubsystem.setpoint)
            )
        ArmSubsystem.setTargetPosition(
            try {
                ArmConstants.Positions.values()[currentPositionIndex]
            } catch (e: ArrayIndexOutOfBoundsException) { // If the element is at -1 (does not exist
                ArmConstants.Positions.Home
            }
        )
    },
    ArmSubsystem,
)
