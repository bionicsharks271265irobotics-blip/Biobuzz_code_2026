/*
 * =============================================================================
 * TELEOP OPMODE: Motor_ControlTest
 * =============================================================================
 * PURPOSE:
 * TeleOp test OpMode for verifying mecanum drivetrain motion combined with 
 * intake/auxiliary test motor velocity and tick position telemetry feedback.
 *
 * CONTROLS OVERVIEW:
 * - Gamepad 1 Left/Right Joysticks: Standard Mecanum drive, strafe, and turn controls.
 * - Gamepad 1 Right Bumper: Runs the intake test motor ("motor") at 100% power when 
 *   held, or 50% power when released.
 *
 * HARDWARE CONFIGURATION:
 * - Drivetrain Motors: "fLeft", "bLeft", "fRight", "bRight"
 * - Intake/Test Motor: "motor" (DcMotorEx)
 * =============================================================================
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * TeleOp Test OpMode for Intake / Auxiliary Test Motor.
 *
 * Purpose:
 * - Tests the intake system motor ("motor") with velocity and encoder tick telemetry feedback.
 * - Controls the drivetrain using mecanum drive.
 *
 * Hardware Mapping:
 * - "motor": Intake / Auxiliary Motor (DcMotorEx, REVERSE direction)
 * - "fLeft": Front Left drivetrain motor
 * - "bLeft": Back Left drivetrain motor
 * - "fRight": Front Right drivetrain motor
 * - "bRight": Back Right drivetrain motor
 *
 * Gamepad Controls:
 * - Left Stick: Drive and strafe
 * - Right Stick: Turn
 * - Right Bumper: Runs intake motor at 100% power
 * - Right Bumper released: Runs intake motor at 50% power
 */
@TeleOp(name = "Motor Control Test", group = "Test")
public class Motor_ControlTest extends LinearOpMode {

    /** Encoder ticks per full motor output shaft revolution. */
    static final double TICKS_PER_REV = 537.7;

    @Override
    public void runOpMode() {

        // =====================================================================
        // 1. HARDWARE MAPPING & INITIALIZATION
        // =====================================================================

        // --- DRIVETRAIN HARDWARE INITIALIZATION ---
        DcMotor fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        DcMotor bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        DcMotor fRight = hardwareMap.get(DcMotor.class, "fRight");
        DcMotor bRight = hardwareMap.get(DcMotor.class, "bRight");

        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        // Retrieve the intake / auxiliary test motor
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");

        // Reverse motor direction to align with standard robot orientation
        motor.setDirection(DcMotor.Direction.REVERSE);

        // Reset and initialize encoder for the intake motor
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Display initialization telemetry on Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for driver to press START on Driver Station
        waitForStart();

        // =====================================================================
        // 2. MAIN TELEOP CONTROL LOOP
        // =====================================================================
        while (opModeIsActive()) {

            // -----------------------------------------------------------------
            // A. MECANUM DRIVE KINEMATICS & VECTOR CALCULATIONS
            // -----------------------------------------------------------------

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(
                    Math.abs(y) + Math.abs(x) + Math.abs(rx),
                    1.0
            );

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            fLeft.setPower(frontLeftPower);
            bLeft.setPower(backLeftPower);
            fRight.setPower(frontRightPower);
            bRight.setPower(backRightPower);

            // -----------------------------------------------------------------
            // B. INTAKE MOTOR CONTROL & VELOCITY TELEMETRY
            // -----------------------------------------------------------------

            // Holding Right Bumper runs intake motor at 100% power;
            // releasing runs at 50% power
            double motorPower = gamepad1.right_bumper ? 1.0 : 0.5;
            motor.setPower(motorPower);

            // Calculate actual velocity and angular speed metrics
            double ticksPerSecond = motor.getVelocity();
            double ticksPerMinute = ticksPerSecond * 60.0;
            double currentPositionTicks = motor.getCurrentPosition();
            double calculatedRotations =
                    currentPositionTicks / TICKS_PER_REV;

            // -----------------------------------------------------------------
            // C. DRIVER STATION TELEMETRY FEEDBACK
            // -----------------------------------------------------------------

            telemetry.addData("Mode", "Drivetrain + Intake");
            telemetry.addData("Intake Motor Power", "%.2f", motorPower);
            telemetry.addData(
                    "Velocity (Ticks/Min)",
                    "%.1f",
                    ticksPerMinute
            );
            telemetry.addData(
                    "Current Position Ticks",
                    "%d",
                    (int) currentPositionTicks
            );
            telemetry.addData(
                    "Calculated Rotations",
                    "%.2f revs",
                    calculatedRotations
            );

            telemetry.update();
        }
    }
}
