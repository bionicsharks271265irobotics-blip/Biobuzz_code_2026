/*
 * =============================================================================
 * TELEOP OPMODE: FinalTeleOpCode
 * AUTHORS: Ronith Janjam and Neal Sheth
 * =============================================================================
 * PURPOSE:
 * Comprehensive competition TeleOp OpMode designed for FTC robot control.
 * Splits driving and subsystem management across two independent gamepads:
 *  - Gamepad 1 / Controller 1 (Driver)   : Field-oriented mecanum drivetrain navigation and speed modes.
 *  - Gamepad 2 / Controller 2 (Operator) : Subsystem control (Shooter Flywheel, Intake, Storage 1, Storage 2)
 *                                           and non-blocking automated single-ball feed sequence on Button B.
 *
 * CONTROLS OVERVIEW & GAMEPAD MAPPING:
 * -----------------------------------------------------------------------------
 * GAMEPAD 1 / CONTROLLER 1: DRIVER CONTROLLER (Drivetrain Navigation)
 *  - Left Stick Up/Down    : Drive Forward and Backward
 *  - Left Stick Left/Right : Strafe Sideways (Left and Right)
 *  - Right Stick Left/Right: Rotate / Turn Robot Left and Right
 *  - Right Bumper (RB)     : Turbo Fast Speed Mode (100% Power)
 *  - Left Bumper (LB)      : Precision Slow Speed Mode (35% Power)
 *  - Default Speed         : Normal Drive Mode (85% Power)
 * 
 * GAMEPAD 2 / CONTROLLER 2: OPERATOR CONTROLLER (Subsystem Management)
 *  - Left Bumper (LB)      : Run Storage Motor 1 FORWARD (Hold)
 *  - Left Trigger (LT)     : Run Storage Motor 1 BACKWARD (Hold)
 *  - Right Bumper (RB)     : Run Storage Motor 2 FORWARD (Hold)
 *  - Right Trigger (RT)    : Run Storage Motor 2 BACKWARD (Hold)
 *  - Button A              : Toggle Shooter Motor ON (Target TPS) / OFF
 *  - Button B              : Trigger Automated Single-Ball Feed Sequence
 *                            (Lifts 1 ball into Storage 2, then feeds to Shooter)
 *  - D-Pad UP              : Increase Target Shooter Velocity (+25 Ticks/Sec)
 *  - D-Pad DOWN            : Decrease Target Shooter Velocity (-25 Ticks/Sec)
 *  - Button X              : Toggle Intake Motor ON / OFF
 *  - Button Y              : Reverse Intake Motor Direction (Forward <-> Reverse)
 *
 * HARDWARE CONFIGURATION & HUB PORT MAP:
 * -----------------------------------------------------------------------------
 * CONTROL HUB (HUB 1):
 *  - Motor Port 0 : "fLeft"        (Front Left Wheel, DcMotorEx)
 *  - Motor Port 1 : "bLeft"        (Back Left Wheel, DcMotorEx)
 *  - Motor Port 2 : "intake"       (Intake Motor, DcMotorEx)
 *  - Motor Port 3 : "storage2"     (Upper Storage Motor 2, DcMotorEx)
 * 
 * EXPANSION HUB (HUB 2):
 *  - Motor Port 0 : "fRight"       (Front Right Wheel, DcMotorEx)
 *  - Motor Port 1 : "bRight"       (Back Right Wheel, DcMotorEx)
 *  - Motor Port 2 : "shooterMotor" (Shooter Flywheel Motor, DcMotorEx)
 *  - Motor Port 3 : "storage1"     (Lower Storage Motor 1, DcMotorEx)
 * =============================================================================
 */

package org.firstinspires.ftc.teamcode;

// Import core FTC SDK classes for hardware motor control and gamepads
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * =============================================================================
 *                      CONTROLLER BUTTON INSTRUCTIONS
 * =============================================================================
 * 
 * -----------------------------------------------------------------------------
 * GAMEPAD 1 / CONTROLLER 1: DRIVER (Drivetrain & Movement Only)
 * -----------------------------------------------------------------------------
 *  - Left Joystick (Push Up / Down)    : Drive Forward and Backward
 *  - Left Joystick (Push Left / Right)  : Strafe Sideways (Left and Right)
 *  - Right Joystick (Push Left / Right) : Rotate / Turn Robot Left and Right
 *  - Right Bumper (Hold RB)             : Turbo Fast Mode (100% Speed)
 *  - Left Bumper (Hold LB)              : Slow Precision Mode (35% Speed)
 *  - Default Speed (No Bumpers)         : Normal Driving Mode (85% Speed)
 * 
 * -----------------------------------------------------------------------------
 * GAMEPAD 2 / CONTROLLER 2: OPERATOR (All Subsystem Motors & Automated Shot)
 * -----------------------------------------------------------------------------
 *  - Left Bumper (Hold LB)              : Run Storage 1 FORWARD (Hold to run)
 *  - Left Trigger (Squeeze LT)          : Run Storage 1 BACKWARD (Hold to run)
 *  - Right Bumper (Hold RB)             : Run Storage 2 FORWARD (Hold to run)
 *  - Right Trigger (Squeeze RT)         : Run Storage 2 BACKWARD (Hold to run)
 *  - A Button (Press)                   : Toggle Shooter Motor ON (Target Velocity) / OFF
 *  - B Button (Press)                   : AUTOMATED STORAGE BALL FEED SEQUENCE
 *                                         (Lifts 1 ball from intake into storage 2, 
 *                                          then feeds ball into shooter)
 *  - D-Pad UP (Press)                   : Increase Target Shooter Speed (+25 Ticks/Sec)
 *  - D-Pad DOWN (Press)                 : Decrease Target Shooter Speed (-25 Ticks/Sec)
 *  - X Button (Press)                   : Toggle Intake Motor ON / OFF
 *  - Y Button (Press)                   : Reverse Intake Motor Direction
 * =============================================================================
 */

@TeleOp(name = "FinalTeleOpCode", group = "TeleOp")
public class FinalTeleOpCode extends LinearOpMode {

    // =========================================================================
    // ENCODER & VELOCITY TUNING CONSTANTS
    // =========================================================================
    /** GoBILDA motor encoder resolution (537.7 ticks per full shaft revolution). */
    public static final double TICKS_PER_REV = 537.7;

    /** Speed adjustment step per D-Pad press (+/- 25 encoder ticks per second). */
    public static final double SHOOTER_STEP = 25.0;

    /** Default starting target velocity for shooter flywheel (1650.0 ticks/sec). */
    public static final double DEFAULT_SHOOTER_VELOCITY = 1650.0;

    // =========================================================================
    // AUTOMATED FEED SEQUENCE TIMING CONSTANTS (IN SECONDS)
    // =========================================================================
    /** Duration Storage 1 runs to lift ONE ball from intake into Storage 2 (0.40s). */
    public static final double STORAGE1_TRANSFER_TIME = 0.40;

    /** Delay before Storage 2 feeds ball into the spinning flywheel (0.70s total). */
    public static final double STORAGE2_FEED_DELAY    = 0.70;

    /** Duration Storage 2 runs to feed ball into shooter flywheel (0.60s). */
    public static final double STORAGE2_FEED_TIME     = 0.60;

    /** Post-shot buffer duration before ending sequence (0.20s). */
    public static final double POST_SHOT_BUFFER_TIME  = 0.20;

    // =========================================================================
    // HARDWARE MOTOR REFERENCES
    // =========================================================================
    /** Drivetrain mecanum motors (Control Hub & Expansion Hub). */
    private DcMotorEx fLeft;  // Front Left Wheel
    private DcMotorEx bLeft;  // Back Left Wheel
    private DcMotorEx fRight; // Front Right Wheel
    private DcMotorEx bRight; // Back Right Wheel

    /** Shooter flywheel motor with closed-loop PID velocity control. */
    private DcMotorEx shooterMotor;

    /** Intake system motor. */
    private DcMotorEx intake;

    /** Dual independent storage transport motors. */
    private DcMotorEx storage1; // Lower Storage Unit 1
    private DcMotorEx storage2; // Upper Storage Unit 2

    // =========================================================================
    // SUBSYSTEM STATE VARIABLES
    // =========================================================================
    /** Tracks whether intake motor is active (ON/OFF). */
    private boolean isIntakeOn = false;

    /** Tracks intake motor spin direction (Default: Reversed = true). */
    private boolean isIntakeReversed = true;

    /** Tracks whether shooter flywheel is toggled ON to target velocity. */
    private boolean isShooterOn = false;

    /** Configured target velocity setting for shooter flywheel in ticks/sec. */
    private double targetShooterVelocity = DEFAULT_SHOOTER_VELOCITY;

    /** Tracks whether the non-blocking Button B automated feed sequence is active. */
    private boolean isAutoShooting = false;

    /** Non-blocking high-resolution timer for managing automated feed sequence. */
    private final ElapsedTime autoShootTimer = new ElapsedTime();

    // =========================================================================
    // GAMEPAD BUTTON DEBOUNCE STATE FLAGS (PREVENTS REPEATED TRIGGERING)
    // =========================================================================
    private boolean prevA2 = false;
    private boolean prevB2 = false;
    private boolean prevX2 = false;
    private boolean prevY2 = false;
    private boolean prevDpadUp2 = false;
    private boolean prevDpadDown2 = false;

    /**
     * Main OpMode execution loop called when driver selects OpMode on Driver Station.
     */
    @Override
    public void runOpMode() {

        // Set telemetry refresh rate to 50 Hz (20ms transmission interval)
        telemetry.setMsTransmissionInterval(20);

        // =====================================================================
        // 1. HARDWARE MAPPING & SUBSYSTEM INITIALIZATION
        // =====================================================================

        // --- DRIVETRAIN HARDWARE SETUP ---
        fLeft  = hardwareMap.get(DcMotorEx.class, "fLeft");
        bLeft  = hardwareMap.get(DcMotorEx.class, "bLeft");
        fRight = hardwareMap.get(DcMotorEx.class, "fRight");
        bRight = hardwareMap.get(DcMotorEx.class, "bRight");

        // Reverse left side drive motors to align forward pushing direction
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        // Apply BRAKE zero power behavior for crisp stopping
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure drive motors for direct un-capped voltage drive
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- SHOOTER MOTOR HARDWARE SETUP ---
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        } catch (Exception e) {
            try {
                shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
            } catch (Exception ex) {
                shooterMotor = null;
            }
        }

        if (shooterMotor != null) {
            shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // --- INTAKE MOTOR HARDWARE SETUP ---
        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        } catch (Exception e) {
            try {
                intake = hardwareMap.get(DcMotorEx.class, "intake1");
            } catch (Exception ex) {
                intake = null;
            }
        }
        if (intake != null) {
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- STORAGE MOTOR 1 SETUP ---
        try {
            storage1 = hardwareMap.get(DcMotorEx.class, "storage1");
        } catch (Exception e) {
            try {
                storage1 = hardwareMap.get(DcMotorEx.class, "intake1");
            } catch (Exception ex) {
                storage1 = null;
            }
        }
        if (storage1 != null) {
            storage1.setDirection(DcMotor.Direction.REVERSE);
            storage1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- STORAGE MOTOR 2 SETUP ---
        try {
            storage2 = hardwareMap.get(DcMotorEx.class, "storage2");
        } catch (Exception e) {
            try {
                storage2 = hardwareMap.get(DcMotorEx.class, "intake2");
            } catch (Exception ex) {
                storage2 = null;
            }
        }
        if (storage2 != null) {
            storage2.setDirection(DcMotor.Direction.FORWARD);
            storage2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Initialized - Ready for Start");
        telemetry.update();

        // Wait for driver to press START on Driver Station
        waitForStart();
        resetRuntime();

        // =====================================================================
        // 2. MAIN TELEOP LOOP (RUNS CONTINUOUSLY UNTIL STOP PRESSED)
        // =====================================================================
        while (opModeIsActive()) {

            /*
             * =================================================================
             * CONTROLLER 1 / GAMEPAD 1: MECANUM DRIVETRAIN NAVIGATION
             * =================================================================
             */
            double y  = -gamepad1.left_stick_y;       // Drive Forward / Backward
            double x  = gamepad1.left_stick_x * 1.1;  // Strafe Sideways (* 1.1 corrects strafe friction)
            double rx = gamepad1.right_stick_x;       // Turn / Rotate Robot

            // Speed multiplier selector
            double speedMultiplier = 0.85; // Default speed: 85%
            if (gamepad1.right_bumper) {
                speedMultiplier = 1.0;  // Turbo speed: 100%
            } else if (gamepad1.left_bumper) {
                speedMultiplier = 0.35; // Slow precision speed: 35%
            }

            // Mecanum drive kinematics calculation
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = ((y + x + rx) / denominator) * speedMultiplier;
            double backLeftPower   = ((y - x + rx) / denominator) * speedMultiplier;
            double frontRightPower = ((y - x - rx) / denominator) * speedMultiplier;
            double backRightPower  = ((y + x - rx) / denominator) * speedMultiplier;

            fLeft.setPower(frontLeftPower);
            bLeft.setPower(backLeftPower);
            fRight.setPower(frontRightPower);
            bRight.setPower(backRightPower);


            /*
             * =================================================================
             * CONTROLLER 2 / GAMEPAD 2: AUTOMATED STORAGE BALL FEED SEQUENCE (BUTTON B)
             * =================================================================
             */
            // Pressing Button B triggers non-blocking Storage 1 & 2 feed routine.
            // Note: Does not touch or change shooterMotor velocity!
            boolean currentB2 = gamepad2.b;
            if (currentB2 && !prevB2) {
                isAutoShooting = true;
                autoShootTimer.reset();
            }
            prevB2 = currentB2;

            double storage1Power = 0.0;
            double storage2Power = 0.0;

            if (isAutoShooting) {
                double elapsedTime = autoShootTimer.seconds();
                double totalCycleTime = STORAGE2_FEED_DELAY + STORAGE2_FEED_TIME + POST_SHOT_BUFFER_TIME;

                // Stage 1: Storage 1 runs for 0.40s to push 1 ball into Storage 2
                if (elapsedTime < STORAGE1_TRANSFER_TIME) {
                    storage1Power = 1.0;
                } else {
                    storage1Power = 0.0;
                }

                // Stage 2: Storage 2 feeds ball after delay (0.70s to 1.30s) into the 1650 TPS shooter
                if (elapsedTime >= STORAGE2_FEED_DELAY && elapsedTime < (STORAGE2_FEED_DELAY + STORAGE2_FEED_TIME)) {
                    storage2Power = 1.0;
                } else {
                    storage2Power = 0.0;
                }

                // Stage 3: Complete automated feed sequence at 1.50s total
                if (elapsedTime >= totalCycleTime) {
                    isAutoShooting = false;
                    storage1Power = 0.0;
                    storage2Power = 0.0;
                }
            } else {
                // -------------------------------------------------------------
                // MANUAL STORAGE CONTROLS (WHEN NOT AUTO-FEEDING)
                // -------------------------------------------------------------
                // Storage 1: Left Bumper (Forward) / Left Trigger (Backward)
                if (gamepad2.left_bumper) {
                    storage1Power = 1.0;
                } else if (gamepad2.left_trigger > 0.05) {
                    storage1Power = -1.0;
                }

                // Storage 2: Right Bumper (Forward) / Right Trigger (Backward)
                if (gamepad2.right_bumper) {
                    storage2Power = 1.0;
                } else if (gamepad2.right_trigger > 0.05) {
                    storage2Power = -1.0;
                }
            }

            if (storage1 != null) {
                storage1.setPower(storage1Power);
            }

            if (storage2 != null) {
                storage2.setPower(storage2Power);
            }


            /*
             * =================================================================
             * CONTROLLER 2 / GAMEPAD 2: INTAKE MOTOR CONTROL & DIRECTION REVERSAL (X / Y)
             * =================================================================
             */
            // Button X (Press): Toggle Intake Motor ON / OFF
            boolean currentX2 = gamepad2.x;
            if (currentX2 && !prevX2) {
                isIntakeOn = !isIntakeOn;
            }
            prevX2 = currentX2;

            // Button Y (Press): Reverse Intake Motor Direction
            boolean currentY2 = gamepad2.y;
            if (currentY2 && !prevY2) {
                isIntakeReversed = !isIntakeReversed;
            }
            prevY2 = currentY2;

            // Calculate intake power based on toggle & direction flags
            double baseIntakePower = isIntakeOn ? 1.0 : 0.0;
            double intakePower = isIntakeReversed ? -baseIntakePower : baseIntakePower;

            if (intake != null) {
                intake.setPower(intakePower);
            }


            /*
             * =================================================================
             * CONTROLLER 2 / GAMEPAD 2: SHOOTER MOTOR CONTROL (BUTTON A & D-PAD TUNING)
             * =================================================================
             */
            // Button A (Press): Toggle Shooter Motor ON (Target Velocity) / OFF
            boolean currentA2 = gamepad2.a;
            if (currentA2 && !prevA2) {
                isShooterOn = !isShooterOn;
            }
            prevA2 = currentA2;

            // D-Pad UP: Increases target shooter velocity by +25 ticks/sec (starts at 1650 TPS)
            boolean currentDpadUp2 = gamepad2.dpad_up;
            if (currentDpadUp2 && !prevDpadUp2) {
                targetShooterVelocity = Math.min(2800.0, targetShooterVelocity + SHOOTER_STEP);
            }
            prevDpadUp2 = currentDpadUp2;

            // D-Pad DOWN: Decreases target shooter velocity by -25 ticks/sec (down to 0 min)
            boolean currentDpadDown2 = gamepad2.dpad_down;
            if (currentDpadDown2 && !prevDpadDown2) {
                targetShooterVelocity = Math.max(0.0, targetShooterVelocity - SHOOTER_STEP);
            }
            prevDpadDown2 = currentDpadDown2;

            // Apply closed-loop PID velocity control to shooter motor
            if (shooterMotor != null) {
                if (isShooterOn) {
                    shooterMotor.setVelocity(targetShooterVelocity);
                } else {
                    shooterMotor.setVelocity(0.0);
                    shooterMotor.setPower(0.0);
                }
            }


            // =================================================================
            // E. DRIVER STATION TELEMETRY FEEDBACK DISPLAY
            // =================================================================
            String driveMode;
            if (gamepad1.right_bumper) {
                driveMode = "TURBO (100%)";
            } else if (gamepad1.left_bumper) {
                driveMode = "SLOW (35%)";
            } else {
                driveMode = "NORMAL (85%)";
            }

            // Display Auto-Feed sequence banner prominently at top of telemetry
            if (isAutoShooting) {
                double t = autoShootTimer.seconds();
                String stageDescription;
                if (t < STORAGE1_TRANSFER_TIME) {
                    stageDescription = "[STAGE 1/3] LIFTING BALL (Storage 1 -> Storage 2)";
                } else if (t < STORAGE2_FEED_DELAY) {
                    stageDescription = "[STAGE 2/3] STANDBY WAIT FOR FLYWHEEL";
                } else if (t < STORAGE2_FEED_DELAY + STORAGE2_FEED_TIME) {
                    stageDescription = "[STAGE 3/3] FEEDING BALL TO SHOOTER";
                } else {
                    stageDescription = "[STAGE 3/3] SEQUENCE COMPLETE";
                }

                telemetry.addLine("*****************************************");
                telemetry.addLine(">>> AUTO BALL-FEED SEQUENCE RUNNING! <<<");
                telemetry.addData("Active Stage", "%s", stageDescription);
                telemetry.addData("Time Elapsed", "%.2fs / 1.50s", t);
                telemetry.addLine("*****************************************");
                telemetry.addLine();
            } else {
                telemetry.addData("AUTO-FEED SYSTEM", "READY (Press Gamepad 2 'B' to Feed Ball)");
            }

            telemetry.addData("=== DRIVETRAIN (Gamepad 1 / Controller 1) ===", "");
            telemetry.addData("Drive Speed Mode", driveMode);

            telemetry.addData("\n=== SUBSYSTEMS (Gamepad 2 / Controller 2) ===", "");

            telemetry.addData("Storage 1 (LB/LT)", "Power: %.2f (%s)", storage1Power,
                    storage1Power > 0 ? "FORWARD" : (storage1Power < 0 ? "BACKWARD" : "STOPPED"));
            telemetry.addData("Storage 2 (RB/RT)", "Power: %.2f (%s)", storage2Power,
                    storage2Power > 0 ? "FORWARD" : (storage2Power < 0 ? "BACKWARD" : "STOPPED"));

            String intakeDirLabel = isIntakeReversed ? "NORMAL" : "REVERSED";
            telemetry.addData("Intake (X: On/Off | Y: Dir)", isIntakeOn ? "ON (Power: %.2f | Dir: %s)" : "OFF (Dir: %s)",
                    intakePower, intakeDirLabel, intakeDirLabel);

            telemetry.addData("Shooter Flywheel (A: Toggle)", isShooterOn ? String.format("RUNNING (%.0f TPS Constant)", targetShooterVelocity) : "STOPPED");
            telemetry.addData("Target Shooter Velocity", "%.0f ticks/sec (D-Pad +/-25 TPS)", targetShooterVelocity);

            // Real-time live TPS reading for all 8 motors
            telemetry.addLine();
            telemetry.addLine("=================================");
            telemetry.addLine(" ALL MOTOR TICKS PER SECOND (TPS)");
            telemetry.addLine("=================================");

            // Drivetrain Motor Velocity Telemetry
            telemetry.addData("[1] fLeft (Front Right Wheel)", "%.1f ticks/sec", fLeft.getVelocity());
            telemetry.addData("[2] bLeft (Back Right Wheel)", "%.1f ticks/sec", bLeft.getVelocity());
            telemetry.addData("[3] fRight (Front Left Wheel)", "%.1f ticks/sec", fRight.getVelocity());
            telemetry.addData("[4] bRight (Back Left Wheel)", "%.1f ticks/sec", bRight.getVelocity());

            // Shooter Motor Velocity Telemetry
            if (shooterMotor != null) {
                telemetry.addData("[5] Shooter Motor", "%.1f ticks/sec", shooterMotor.getVelocity());
            } else {
                telemetry.addData("[5] Shooter Motor", "NOT CONNECTED");
            }

            // Intake Motor Velocity Telemetry
            if (intake != null) {
                telemetry.addData("[6] Intake Motor", "%.1f ticks/sec", intake.getVelocity());
            } else {
                telemetry.addData("[6] Intake Motor", "NOT CONNECTED");
            }

            // Storage Motor Velocity Telemetry
            if (storage1 != null) {
                telemetry.addData("[7] Storage Motor 1", "%.1f ticks/sec", storage1.getVelocity());
            } else {
                telemetry.addData("[7] Storage Motor 1", "NOT CONNECTED");
            }

            if (storage2 != null) {
                telemetry.addData("[8] Storage Motor 2", "%.1f ticks/sec", storage2.getVelocity());
            } else {
                telemetry.addData("[8] Storage Motor 2", "NOT CONNECTED");
            }

            telemetry.update();
        }

        // =====================================================================
        // 3. SAFE OPMODE SHUTDOWN (ON STOP BUTTON PRESS)
        // =====================================================================
        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
        if (intake != null) intake.setPower(0);
        if (storage1 != null) storage1.setPower(0);
        if (storage2 != null) storage2.setPower(0);
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0);
            shooterMotor.setPower(0);
        }
    }
}
