package org.firstinspires.ftc.teamcode.parts

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.rampWithBrake
import kotlin.math.abs

class Drive(val lf: DcMotor, val lb: DcMotor, val rf: DcMotor, val rb: DcMotor) {
    var flCmd = 0.0
    var blCmd = 0.0
    var frCmd = 0.0
    var brCmd = 0.0
    var lastTime = System.nanoTime()

    fun drive(x: Double, y: Double, rx: Double) {
        val now = System.nanoTime()
        val dt = (now - lastTime) / 1e9
        lastTime = now

        val denominator = maxOf(abs(y) + abs(x) + abs(rx), 1.0)

        val flTarget = (y + x + rx) / denominator
        val blTarget = (y - x + rx) / denominator
        val frTarget = (y - x - rx) / denominator
        val brTarget = (y + x - rx) / denominator

        val drivespeed = 1.0

        val flOut = flTarget * drivespeed
        val blOut = blTarget * drivespeed
        val frOut = frTarget * drivespeed
        val brOut = brTarget * drivespeed

        flCmd = rampWithBrake(flCmd, flOut, dt)
        blCmd = rampWithBrake(blCmd, blOut, dt)
        frCmd = rampWithBrake(frCmd, frOut, dt)
        brCmd = rampWithBrake(brCmd, brOut, dt)

        lf.power = flCmd
        lb.power = blCmd
        rf.power = frCmd
        rb.power = brCmd
    }
}