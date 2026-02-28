package org.firstinspires.ftc.teamcode

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

fun clamp(v: Double, lo: Double, hi: Double): Double = max(lo, min(hi, v))

fun rampTowards(current: Double, target: Double, maxDelta: Double): Double {
    val delta = (target - current).coerceIn(-maxDelta, maxDelta)
    return current + delta
}

fun rampWithBrake(current: Double, target: Double, dt: Double, brakeRate: Double = 18.0, accelRate: Double = 8.0): Double {
    val rate = if (abs(target) < abs(current)) brakeRate else accelRate
    return rampTowards(current, target, rate * dt)
}

fun lerp(a: Double, b: Double, t: Double) = a + (b - a) * t

fun interp1D(x: Double, xs: DoubleArray, ys: DoubleArray): Double {
    if (xs.isEmpty() || ys.isEmpty() || xs.size != ys.size) return ys.firstOrNull() ?: 0.0
    if (x <= xs.first()) return ys.first()
    if (x >= xs.last()) return ys.last()
    for (i in 0 until xs.size - 1) {
        val x0 = xs[i]
        val x1 = xs[i + 1]
        if (x in x0..x1) {
            val t = (x - x0) / (x1 - x0)
            return lerp(ys[i], ys[i + 1], t)
        }
    }
    return ys.last()
}

fun slew(current: Double, target: Double, dt: Double, ratePerSec: Double): Double {
    val maxStep = ratePerSec * dt
    val delta = (target - current).coerceIn(-maxStep, maxStep)
    return current + delta
}

//private fun startSpdxMove(dir: Int) {
//    if (spdxMoving) return
//    spdxDir = if (dir >= 0) 1 else -1
//    spdxMoving = true
//    spdxMoveTimer = 0.0
//    spdxBrakeTimer = 0.0
//    val startPressed = mag.isPressed
//    spdxStage = if (startPressed) 0 else 1
//}