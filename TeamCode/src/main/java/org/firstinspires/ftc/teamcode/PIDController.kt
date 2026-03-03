package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sign

class PIDController(var kp: Double, var ki: Double, var kd: Double, var maxIntegralSum: Double = 0.25, var ks: Double = 0.06) {
    private var lastError = 0.0
    private var integralSum = 0.0
    private var timer = ElapsedTime()
    private var lastTime = 0.0

    fun calculate(target: Double, current: Double): Double {
        val currentTime = timer.seconds()
        val deltaTime = currentTime - lastTime

        val error = target - current
        val derivative = if (deltaTime > 0) (error - lastError) / deltaTime else 0.0

        integralSum += error * deltaTime
        integralSum = clamp(integralSum, -maxIntegralSum, maxIntegralSum)

        lastError = error
        lastTime = currentTime

        val output = (kp * error) + (ki * integralSum) + (kd * derivative)

        return sign(output) * max(abs(output), ks)
    }

    fun setValues(kp: Double, ki: Double, kd: Double, maxIntegralSum: Double = this.maxIntegralSum) {
        this.kp = kp
        this.ki = ki
        this.kd = kd
        this.maxIntegralSum = maxIntegralSum
    }

    fun setValues(values: PIDValues) {
        kp = values.kp
        ki = values.ki
        kd = values.kd
        maxIntegralSum = values.maxIntegralSum
    }

    fun reset() {
        integralSum = 0.0
        lastError = 0.0
        timer.reset()
        lastTime = 0.0
    }
}

class PIDValues(val kp: Double, val ki: Double, val kd: Double, val maxIntegralSum: Double = 0.25) {}