/*
 * =============================================================================
 * TELEOP OPMODE: ShooterTest
 * =============================================================================
 * PURPOSE:
 * Diagnostic TeleOp OpMode for closed-loop PID velocity tuning and testing 
 * of the shooter flywheel motor.
 *
 * CONTROLS OVERVIEW:
 * - Gamepad 1 Button A: Increments shooter target velocity by +25 ticks/sec.
 * - Gamepad 1 Button B: Decrements shooter target velocity by -25 ticks/sec.
 *
 * HARDWARE CONFIGURATION:
 * - Shooter Motor: "shootermotor" (DcMotorEx)
 * =============================================================================
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * TeleOp Test OpMode for Closed-Loop Velocity Control of the Robot Shooter Motor.
 * 
 * <p><strong>Purpose:</strong></p>
 * <ul>
 *   <li>Tests closed-loop PID velocity control using the encoder on {@code "shootermotor"}.</li>
 *   <li>Allows real-time velocity tuning on the Driver Station via gamepad buttons.</li>
 * </ul>
 * 
 * <p><strong>Hardware Mapping:</strong></p>
 * <ul>
 *   <li>{@code "shootermotor"}: Shooter Motor (DcMotorEx, REVERSE direction, BRAKE behavior)</li>
 * </ul>
 * 
 * <p><strong>Gamepad Controls:</strong></p>
 * <ul>
 *   <li>{@code Gamepad 1 Button A}: Increases shooter target velocity (+25 ticks/sec)</li>
 *   <li>{@code Gamepad 1 Button B}: Decreases shooter target velocity (-25 ticks/sec)</li>
 * </ul>
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    // =========================================================================
    // HARDWARE & ENCODER CONSTANTS
    // =========================================================================
    /** Encoder ticks per full motor output shaft revolution (GoBILDA 5202/5203 Series 19.2:1 ratio). */
    public static final double TICKS_PER_REV = 537.7;

    // =========================================================================
    // HARDWARE DECLARATIONS & STATE VARIABLES
    // =========================================================================
    /** Shooter motor reference with expanded extended velocity controls (DcMotorEx). */
    private DcMotorEx shootermotor;

    /** Target velocity setting in ticks per second (defaults to 1500.0 ticks/sec). */
    private double targetTicksPerSecond = 1500.0;

    /** Edge-detection state variables for button debouncing. */
    private boolean previousA = false;
    private boolean previousB = false;

    @Override
    public void runOpMode() {
        // High-frequency telemetry updates (20ms interval = 50 updates/sec)
        telemetry.setMsTransmissionInterval(20);

        // =====================================================================
        // 1. HARDWARE MAPPING & INITIALIZATION
        // =====================================================================
        shootermotor = hardwareMap.get(DcMotorEx.class, "shootermotor");

        // Set direction and motor zero-power behavior to BRAKE
        shootermotor.setDirection(DcMotorEx.Direction.REVERSE);
        shootermotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset motor encoder position and activate closed-loop encoder velocity control
        shootermotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootermotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Display initial telemetry status
        telemetry.addData("Status", "Initialized - Ready for Start");
        telemetry.addData("Starting Target Ticks/Sec", "%.1f", targetTicksPerSecond);
        telemetry.addData("Motor Ticks Per Rev", "%.1f", TICKS_PER_REV);
        telemetry.update();

        // Wait for Start button on Driver Station
        waitForStart();

        // Reset runtime clock upon OpMode start
        resetRuntime();

        // =====================================================================
        // 2. MAIN CONTROL LOOP
        // =====================================================================
        while (opModeIsActive()) {
            
            // -----------------------------------------------------------------
            // A. GAMEPAD BUTTON EDGE-DETECTION (DEBOUNCING) & VELOCITY TUNING
            // -----------------------------------------------------------------
            // Button A: Increment target velocity by +25 ticks/sec on rising edge
            boolean currentA = gamepad1.a;
            if (currentA && !previousA) {
                targetTicksPerSecond += 25.0;
            }
            previousA = currentA;

            // Button B: Decrement target velocity by -25 ticks/sec on rising edge (clamped at >= 0)
            boolean currentB = gamepad1.b;
            if (currentB && !previousB) {
                targetTicksPerSecond = Math.max(0, targetTicksPerSecond - 25.0);
            }
            previousB = currentB;

            // -----------------------------------------------------------------
            // B. CLOSED-LOOP VELOCITY CONTROL & TELEMETRY FEEDBACK
            // -----------------------------------------------------------------
            // Direct closed-loop velocity command in encoder ticks per second
            shootermotor.setVelocity(targetTicksPerSecond);

            // Read actual measured velocity from encoder and convert to RPM
            double currentTicksPerSec = shootermotor.getVelocity();
            double currentRPM = (currentTicksPerSec / TICKS_PER_REV) * 60.0;

            // Display live diagnostics on Driver Station
            telemetry.addData("Status", "Running");
            telemetry.addData("Uptime (s)", "%.2f", getRuntime());
            telemetry.addData("Controls", "[A] +25 Ticks/Sec | [B] -25 Ticks/Sec");
            telemetry.addData("Target Velocity (Ticks/Sec)", "%.1f", targetTicksPerSecond);
            telemetry.addData("Actual Velocity (Ticks/Sec)", "%.2f", currentTicksPerSec);
            telemetry.addData("Calculated RPM", "%.1f RPM", currentRPM);
            telemetry.addData("Encoder Position (Ticks)", "%d", shootermotor.getCurrentPosition());
            telemetry.update();
        }

        // =====================================================================
        // 3. SAFE OPMODE SHUTDOWN
        // =====================================================================
        shootermotor.setVelocity(0);
        shootermotor.setPower(0);
    }
}
