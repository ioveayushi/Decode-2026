package org.firstinspires.ftc.teamcode.parts

import androidx.core.math.MathUtils.clamp
import com.qualcomm.robotcore.hardware.Servo

class Light(val servo: Servo) {
    var on: Boolean = false
        set(value) {
            field = value

            servo.position = if (on) color else 0.0
        }

    var color: Double = 0.0
        set(value) {
            field = value
            servo.position = color
        }

    var blinkInterval: Double = 0.0
    private var lastBlink: Int = 0

    private val maxPWM: Double = 0.723
    private val minPWM: Double = 0.279

    fun scaleColors(color: Double): Double {
        return (clamp(color, minPWM, maxPWM) - minPWM) / (maxPWM - minPWM)
    }

    fun setUs(us: Int) {
        servo.position = usToPos(us);
    }

    // Needs to be run for blinking
    fun update() {
        if (System.currentTimeMillis() <= lastBlink + (blinkInterval/2 * 1000).toInt())  {
            on = !on
        }
    }

    private fun usToPos(us: Int): Double { return ((us - 500) / 2000.0).coerceIn(0.0, 1.0) }
}