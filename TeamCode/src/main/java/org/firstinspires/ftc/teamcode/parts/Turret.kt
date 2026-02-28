package org.firstinspires.ftc.teamcode.parts

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.clamp
import org.firstinspires.ftc.teamcode.interp1D
import org.firstinspires.ftc.teamcode.rampTowards
import org.firstinspires.ftc.teamcode.slew
import kotlin.math.abs
import kotlin.math.tan

class Turret(val limelight: Limelight3A, val turretMotor: DcMotor, val flywheelMotor: DcMotorEx) {
    private var lastTxDeg = 0.0
    private var lastTyDeg = 0.0
    private var lastSeenTimeMs = System.currentTimeMillis()
    private val holdLastSeenMs = 250L

    private val deadbandDeg = 1.0
    private val kP_tt = 0.02
    private val minPower = 0.012
    private val maxPower_tt = 1.0

    private val camMountDeg = 20.0
    private val tagHeightCm = 74.93
    private val camHeightCm = 36.02493
    private var distFiltCm = 0.0
    private val distAlpha = 0.25
    private var lastDistSeenMs = System.currentTimeMillis()
    private val holdDistMs = 250L

    private val minVel = 0.0
    private val maxVel = 4500.0
    private var targetVelCmd = 2200.0
    private var targetVel = 2200.0
    private val velSlew = 7000.0
    private val powerSlewPerSec = 3.0
    private var flyPower = 0.0
    private val maxVelEst = 4000.0
    private val kP_vel = 0.00030

    var lastTime = System.nanoTime()

    fun update() {
        val now = System.nanoTime()
        val dt = (now - lastTime) / 1e9
        lastTime = now

        val result = limelight.getLatestResult()

        var hasTargetNow = false
        var txDeg = 0.0
        var tyDeg = 0.0

        if (result != null) {
            hasTargetNow = result.isValid
            txDeg = result.tx
            tyDeg = result.ty
            if (hasTargetNow) {
                lastTxDeg = txDeg
                lastTyDeg = tyDeg
                lastSeenTimeMs = System.currentTimeMillis()
            }
        }

        val nowMs = System.currentTimeMillis()
        val recentlySeen = (nowMs - lastSeenTimeMs) <= holdLastSeenMs

        var turretCmdPower = 0.0

        val txToUse = when {
            hasTargetNow -> txDeg
            recentlySeen -> lastTxDeg
            else -> 0.0
        }

        if (abs(txToUse) <= deadbandDeg) {
            turretCmdPower = 0.0
        } else {
            var pwr = kP_tt * txToUse
            pwr = if (pwr > 0) pwr + minPower else pwr - minPower
            turretCmdPower = clamp(pwr, -maxPower_tt, maxPower_tt)
        }

        turretMotor.power = turretCmdPower

        // Flywheel Speed Calculations
        if (hasTargetNow) {
            val angleRad = Math.toRadians(camMountDeg + tyDeg)
            val tanVal = tan(angleRad)
            if (tanVal > 0.01) {
                val d = (tagHeightCm - camHeightCm) / tanVal
                if (d.isFinite() && d > 0.0) {
                    distFiltCm = (1.0 - distAlpha) * distFiltCm + distAlpha * d
                    lastDistSeenMs = nowMs
                }
            }
        }
        val distFresh = (nowMs - lastDistSeenMs) <= holdDistMs

        val distPts = doubleArrayOf(80.0, 120.0, 160.0, 180.0, 200.0, 320.0)
        val velPts  = doubleArrayOf(1667.0, 1843.6459, 1879.0, 2007.901, 2080.0, 2390.0)

        if (distFresh) {
            val v = interp1D(distFiltCm, distPts, velPts)
            targetVelCmd = v.coerceIn(minVel, maxVel)
        }

        targetVel = slew(targetVel, targetVelCmd, dt, velSlew)
        val measuredVel = abs(flywheelMotor.velocity)

        val ff = (targetVel / maxVelEst).coerceIn(0.0, 1.0)

        val err = targetVel - measuredVel
        val corr = kP_vel * err

        var out = (ff + corr).coerceIn(0.0, 1.0)

        flyPower = rampTowards(flyPower, out, powerSlewPerSec * dt)
        flywheelMotor.power = -flyPower
    }
}