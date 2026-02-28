package org.firstinspires.ftc.teamcode.parts

import android.graphics.Color
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

enum class LightColor(val index: Int) {
    RED(0),
    PURPLE(5),
    GREEN(2)
}

class Spindexer(val servo: AxonServo, val distanceSensor: DistanceSensor, val colorSensor: ColorSensor, val light: Light, val touchSensor: TouchSensor) {
    private val inDist = 3.4
    private val outDist = 4.3
    private val inAlpha = 28
    private val outAlpha = 18
    private var hueFilt = 0f

    private var ballPresent = false

    private val prismColorsUs = intArrayOf(1050, 1200, 1350, 1500, 1650, 1800, 1940)

    private var spdxIndex = 0
    private var spdxMoving = false
    private var spdxMoveTimer = 0.0
    private var spdxBrakeTimer = 0.0
    private var spdxDir = 1
    private var spdxStage = 0

    private val spdxSpinPower = 0.18
    private val spdxTimeout = 0.85
    private val spdxBrakePower = 0.28
    private val spdxBrakeTime = 0.08

    private var magStable = false
    private var magDebounceTimer = 0.0
    private val magDebounceS = 0.006

    var lastTime = System.nanoTime()

    fun update() {
        val now = System.nanoTime()
        val dt = (now - lastTime) / 1e9
        lastTime = now

        val distCmBall = distanceSensor.getDistance(DistanceUnit.CM)
        val distValid = distCmBall.isFinite() && distCmBall > 0.0
        val aa = colorSensor.alpha()

        val rr = colorSensor.red()
        val gg = colorSensor.green()
        val bb = colorSensor.blue()

        val hsv = FloatArray(3)
        Color.RGBToHSV(rr, gg, bb, hsv)
        val hue = hsv[0]
        val sat = hsv[1]
        val value = hsv[2]

        val closeNow = distValid && distCmBall < inDist
        val farNow = !distValid || distCmBall > outDist

        ballPresent = if (!ballPresent) {
            closeNow && aa > inAlpha
        } else {
            !(farNow || aa < outAlpha)
        }

        val hueValid = ballPresent && aa >= inAlpha && sat >= 0.12f && value >= 0.12f
        hueFilt = if (hueValid) 0.8f * hueFilt + 0.2f * hue else hue

        val ballColor = if (!hueValid) {
            "unknown"
        } else if (hueFilt >= 190f) {
            "purple"
        } else {
            "green"
        }

        val lightColor: LightColor = if (!ballPresent) LightColor.RED else if (ballColor == "green") LightColor.GREEN else LightColor.PURPLE
        light.setUs(prismColorsUs[lightColor.index])

        val magRaw = touchSensor.isPressed
        if (magRaw != magStable) {
            magDebounceTimer += dt
            if (magDebounceTimer >= magDebounceS) {
                magStable = magRaw
                magDebounceTimer = 0.0
            }
        } else {
            magDebounceTimer = 0.0
        }
        val magPressed = magStable

        if (spdxMoving) {
            if (spdxStage != 2) spdxMoveTimer += dt

            if (spdxStage == 0) {
                servo.servo.power = spdxSpinPower * spdxDir
                if (!magPressed || spdxMoveTimer >= 0.35) {
                    spdxStage = 1
                    spdxMoveTimer = 0.0
                }
            } else if (spdxStage == 1) {
                servo.servo.power = spdxSpinPower * spdxDir
                if (magPressed || spdxMoveTimer >= spdxTimeout) {
                    spdxStage = 2
                    spdxBrakeTimer = 0.0
                }
            } else {
                spdxBrakeTimer += dt
                servo.servo.power = -spdxBrakePower * spdxDir
                if (spdxBrakeTimer >= spdxBrakeTime) {
                    servo.servo.power = 0.0
                    spdxMoving = false
                    spdxIndex = (spdxIndex + spdxDir + 3) % 3
                }
            }
        } else {
            servo.servo.power = 0.0
        }
    }

    fun startSpdxMove(dir: Int) {
        if (spdxMoving) return
        spdxDir = if (dir >= 0) 1 else -1
        spdxMoving = true
        spdxMoveTimer = 0.0
        spdxBrakeTimer = 0.0
        val startPressed = touchSensor.isPressed
        spdxStage = if (startPressed) 0 else 1
    }
}