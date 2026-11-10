/*
 * =============================================================================
 * TELEOP OPMODE: Motor_1
 * =============================================================================
 * PURPOSE:
 * Basic single-motor TeleOp diagnostic script used to test motor response and wiring.
 *
 * CONTROLS:
 * - Gamepad 1 Left Stick X: Directly sets power to the front left drive motor ("fLeft").
 *
 * HARDWARE CONFIGURATION:
 * - Motor: "fLeft" (DcMotor)
 * =============================================================================
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Motor_1 extends LinearOpMode{

    @Override
    public void runOpMode(){
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "fLeft");

        waitForStart();

        while (opModeIsActive()) {

            motor1.setPower(gamepad1.left_stick_x);
        }

}

}
