package frc7913.lib

import com.revrobotics.SparkMaxPIDController

data class PIDGains(
    val p: Double,
    val i: Double,
    val d: Double
) {
    companion object {
        fun setSparkMaxGains(controller: SparkMaxPIDController, gains: PIDGains) {
            controller.apply {
                p = gains.p
                i = gains.i
                d = gains.d
            }
        }
    }
}
