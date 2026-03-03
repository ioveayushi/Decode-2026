package org.firstinspires.ftc.teamcode.parts

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.PIDController
import org.firstinspires.ftc.teamcode.PIDValues

enum class LightColor(val index: Int) {
    RED(0),
    PURPLE(5),
    GREEN(2)
}

@Config
class Spindexer(val servo: AxonServo, val distanceSensor: DistanceSensor, val colorSensor: ColorSensor, val light: Light, val touchSensor: TouchSensor): Updatable {
    var kp = 0.4
    var ki = 0.5
    var kd = 0.1
    private val pid: PIDValues = PIDValues(kp, ki, kd, 10.0)

    private val prismColorsUs = intArrayOf(1050, 1200, 1350, 1500, 1650, 1800, 1940)

    private var ballPresent: Boolean = false
    private val inDistance: Double = 3.4
    private val outDistance: Double = 4.3
    private val inAlpha: Int = 28
    private val outAlpha: Int = 18
    private var hueFilt = 0f

    private var homePosition: Double = 0.0

    init {
        servo.pid.setValues(PIDValues(kp, ki, kd))
    }

    override fun update() {
        servo.pid.setValues(pid)
    }

    fun checkBall() {
        val r = colorSensor.red()
        val g = colorSensor.green()
        val b = colorSensor.blue()
        val a = colorSensor.alpha()

        val hsv = FloatArray(3)
        Color.RGBToHSV(r, g, b, hsv)

        val distance = distanceSensor.getDistance(DistanceUnit.CM)
        val distanceValid = distance.isFinite() && distance > 0.0

        ballPresent = if (ballPresent) {
            (distanceValid && distance < inDistance) && (a > inAlpha)
        } else {
            !((!distanceValid || distance > outDistance) || a < outAlpha)
        }

        val hueValid = ballPresent && a >= inAlpha && hsv[1] >= 0.12f && hsv[2] >= 0.12f
        hueFilt = if (hueValid) 0.8f * hueFilt + 0.2f * hsv[0] else hsv[0]

        if (!hueValid) {
            setLightColor(LightColor.RED)
        } else if (hueFilt >= 190f) {
            setLightColor(LightColor.PURPLE)
        } else {
            setLightColor(LightColor.GREEN)
        }
    }

    fun rotate(rotation: Int) {
        servo.targetPosition = (servo.targetPosition + 1.0/3.0) % 1.0
    }

    fun home() {
        servo.targetPosition = (servo.targetPosition + 1.0) % 1.0

        while (!touchSensor.isPressed) {
            continue
        }

        homePosition = servo.position
    }

    fun setLightColor(color: LightColor) {
        light.setUs(prismColorsUs[color.index])
    }
}