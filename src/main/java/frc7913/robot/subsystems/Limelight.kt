package frc7913.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase

object Limelight : SubsystemBase() {
    data class Transform(
        val translationX: Double,
        val translationY: Double,
        val translationZ: Double,
        val rotationX: Double,
        val rotationY: Double,
        val rotationZ: Double,
    ) {
        constructor() : this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        fun toArray() = DoubleArray(6) {
            when (it) {
                0 -> translationX
                1 -> translationY
                2 -> translationZ
                3 -> rotationX
                4 -> rotationY
                5 -> rotationZ
                else -> 0.0 // This line shouldn't run, but is necessary for an exhaustive "when" statement
            }
        }

        companion object {
            infix fun from(array: DoubleArray) =
                Transform(
                    array[0],
                    array[1],
                    array[2],
                    array[3],
                    array[4],
                    array[5]
                )
        }
    }
}
