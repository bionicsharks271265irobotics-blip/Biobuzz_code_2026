/*
 * =============================================================================
 * AUTONOMOUS DIAGNOSTIC OPMODE: DiagnosticTest
 * =============================================================================
 * PURPOSE:
 * Comprehensive autonomous diagnostic routine for testing all 8 robot motors 
 * (Drivetrain, Shooter, Intake, Storage1, Storage2) AND the test Servo motor ("servo").
 *
 * Extracted hardware configurations and mapping details directly from FinalTeleOpCode.java 
 * and SERVO_TEST.java.
 *
 * HARDWARE PORT MAPPING (Control Hub & Expansion Hub):
 * -----------------------------------------------------------------------------
 * CONTROL HUB (HUB 1):
 *  - Motor Port 0 : "fLeft"        (Front Right Drivetrain Wheel on Telemetry)
 *  - Motor Port 1 : "bLeft"        (Back Right Drivetrain Wheel on Telemetry)
 *  - Motor Port 2 : "intake"       (Intake Motor, fallback "intake1")
 *  - Motor Port 3 : "storage2"     (Storage Unit Motor 2, fallback "intake2")
 *  - Servo Port   : "servo"        (Test Servo Motor)
 *
 * EXPANSION HUB (HUB 2):
 *  - Motor Port 0 : "fRight"       (Front Left Drivetrain Wheel on Telemetry)
 *  - Motor Port 1 : "bRight"       (Back Left Drivetrain Wheel on Telemetry)
 *  - Motor Port 2 : "shooterMotor" (Shooter Flywheel Motor, fallback "shootermotor")
 *  - Motor Port 3 : "storage1"     (Storage Unit Motor 1, fallback "intake1")
 * -----------------------------------------------------------------------------
 *
 * AUTONOMOUS ROUTINE:
 *  - Upon pressing START, automatically runs a diagnostic test sequence across 
 *    all 8 motors and the Servo consecutively, recording peak velocities and positions.
 *  - Once automated sequence finishes, keeps showing real-time diagnostic dashboard.
 *  - Gamepad 1 controls remain available for manual testing if a controller is connected.
 * =============================================================================
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DiagnosticTest", group = "Diagnostics")
public class DiagnosticTest extends LinearOpMode {

    // Encoder & Servo constants matching FinalTeleOpCode & SERVO_TEST
    public static final double TICKS_PER_REV = 537.7;
    public static final double DEFAULT_SHOOTER_VELOCITY = 1500.0;
    public static final double TEST_POWER = 0.50; // 50% power for standard motors

    public static final double SERVO_HOME_POSITION = 0.0;     // Home position (0.0)
    public static final double SERVO_OUTWARD_POSITION = 0.90;  // 90° Outward position (0.90)

    // Total 9 hardware devices (8 motors + 1 servo)
    public static final int TOTAL_DEVICES = 9;

    private final String[] deviceNames = {
            "Front Right Drive (fLeft)",
            "Back Right Drive (bLeft)",
            "Front Left Drive (fRight)",
            "Back Left Drive (bRight)",
            "Shooter Motor (shooterMotor)",
            "Intake Motor (intake)",
            "Storage Motor 1 (storage1)",
            "Storage Motor 2 (storage2)",
            "Test Servo Motor (servo)"
    };

    private final String[] hardwareConfigNames = {
            "fLeft",
            "bLeft",
            "fRight",
            "bRight",
            "shooterMotor",
            "intake",
            "storage1",
            "storage2",
            "servo"
    };

    private final String[] hubLocations = {
            "Control Hub Motor Port 0",
            "Control Hub Motor Port 1",
            "Expansion Hub Motor Port 0",
            "Expansion Hub Motor Port 1",
            "Expansion Hub Motor Port 2",
            "Control Hub Motor Port 2",
            "Expansion Hub Motor Port 3",
            "Control Hub Motor Port 3",
            "Control Hub Servo Port"
    };

    // Motor arrays (8 motors)
    private final DcMotorEx[] motors = new DcMotorEx[8];
    private final boolean[] motorConnected = new boolean[8];
    private final double[] peakVelocities = new double[8];
    private final int[] startEncoderPositions = new int[8];

    // Servo reference
    private Servo ballservo;
    private boolean servoConnected = false;

    // Status array for all 9 devices
    private final String[] diagnosticStatus = new String[TOTAL_DEVICES];

    // Navigation and state variables
    private int selectedDeviceIndex = 0;
    private boolean isAutomatedSequenceRunning = false;
    private final ElapsedTime timer = new ElapsedTime();

    // Debounce flags for Gamepad 1
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevY = false;
    private boolean prevX = false;
    private boolean prevRB = false;

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(20);

        java.util.Arrays.fill(diagnosticStatus, "UNTESTED");
        java.util.Arrays.fill(peakVelocities, 0.0);

        // =====================================================================
        // 1. HARDWARE MAPPING & INITIALIZATION (FINALTELEOPCODE & SERVO_TEST)
        // =====================================================================

        // --- 0. Front Left Drivetrain Motor (Displayed as Front Right on Telemetry) ---
        try {
            motors[0] = hardwareMap.get(DcMotorEx.class, "fLeft");
            motors[0].setDirection(DcMotor.Direction.REVERSE);
            motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorConnected[0] = true;
        } catch (Exception e) {
            motors[0] = null;
            motorConnected[0] = false;
        }

        // --- 1. Back Left Drivetrain Motor (Displayed as Back Right on Telemetry) ---
        try {
            motors[1] = hardwareMap.get(DcMotorEx.class, "bLeft");
            motors[1].setDirection(DcMotor.Direction.REVERSE);
            motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorConnected[1] = true;
        } catch (Exception e) {
            motors[1] = null;
            motorConnected[1] = false;
        }

        // --- 2. Front Right Drivetrain Motor (Displayed as Front Left on Telemetry) ---
        try {
            motors[2] = hardwareMap.get(DcMotorEx.class, "fRight");
            motors[2].setDirection(DcMotor.Direction.FORWARD);
            motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorConnected[2] = true;
        } catch (Exception e) {
            motors[2] = null;
            motorConnected[2] = false;
        }

        // --- 3. Back Right Drivetrain Motor (Displayed as Back Left on Telemetry) ---
        try {
            motors[3] = hardwareMap.get(DcMotorEx.class, "bRight");
            motors[3].setDirection(DcMotor.Direction.FORWARD);
            motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorConnected[3] = true;
        } catch (Exception e) {
            motors[3] = null;
            motorConnected[3] = false;
        }

        // --- 4. Shooter Motor ---
        try {
            motors[4] = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        } catch (Exception e) {
            try {
                motors[4] = hardwareMap.get(DcMotorEx.class, "shootermotor");
            } catch (Exception ex) {
                motors[4] = null;
            }
        }
        if (motors[4] != null) {
            motors[4].setDirection(DcMotorEx.Direction.REVERSE);
            motors[4].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motors[4].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[4].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorConnected[4] = true;
        } else {
            motorConnected[4] = false;
        }

        // --- 5. Intake Motor ---
        try {
            motors[5] = hardwareMap.get(DcMotorEx.class, "intake");
        } catch (Exception e) {
            try {
                motors[5] = hardwareMap.get(DcMotorEx.class, "intake1");
            } catch (Exception ex) {
                motors[5] = null;
            }
        }
        if (motors[5] != null) {
            motors[5].setDirection(DcMotor.Direction.FORWARD);
            motors[5].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorConnected[5] = true;
        } else {
            motorConnected[5] = false;
        }

        // --- 6. Storage Motor 1 ---
        try {
            motors[6] = hardwareMap.get(DcMotorEx.class, "storage1");
        } catch (Exception e) {
            try {
                motors[6] = hardwareMap.get(DcMotorEx.class, "intake1");
            } catch (Exception ex) {
                motors[6] = null;
            }
        }
        if (motors[6] != null) {
            motors[6].setDirection(DcMotor.Direction.REVERSE);
            motors[6].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorConnected[6] = true;
        } else {
            motorConnected[6] = false;
        }

        // --- 7. Storage Motor 2 ---
        try {
            motors[7] = hardwareMap.get(DcMotorEx.class, "storage2");
        } catch (Exception e) {
            try {
                motors[7] = hardwareMap.get(DcMotorEx.class, "intake2");
            } catch (Exception ex) {
                motors[7] = null;
            }
        }
        if (motors[7] != null) {
            motors[7].setDirection(DcMotor.Direction.FORWARD);
            motors[7].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorConnected[7] = true;
        } else {
            motorConnected[7] = false;
        }

        // --- 8. Test Servo Motor ("servo") ---
        try {
            ballservo = hardwareMap.get(Servo.class, "servo");
        } catch (Exception e) {
            try {
                ballservo = hardwareMap.get(Servo.class, "ballservo");
            } catch (Exception ex) {
                try {
                    ballservo = hardwareMap.get(Servo.class, "servo1");
                } catch (Exception ex2) {
                    ballservo = null;
                }
            }
        }
        if (ballservo != null) {
            try {
                if (ballservo instanceof PwmControl) {
                    ((PwmControl) ballservo).setPwmEnable();
                }
                ballservo.setPosition(SERVO_HOME_POSITION);
                servoConnected = true;
            } catch (Exception e) {
                servoConnected = false;
            }
        } else {
            servoConnected = false;
        }

        // Record initial motor encoder positions
        for (int i = 0; i < 8; i++) {
            if (motors[i] != null) {
                try {
                    startEncoderPositions[i] = motors[i].getCurrentPosition();
                } catch (Exception ignored) {
                }
            }
        }

        int totalConnected = countConnectedDevices();
        telemetry.addData("Status", "Diagnostic OpMode Initialized");
        telemetry.addData("Connected Devices", totalConnected + " / " + TOTAL_DEVICES + " Connected");
        telemetry.update();

        waitForStart();
        resetRuntime();

        // In Autonomous mode: Automatically execute full diagnostic sequence upon pressing START
        if (opModeIsActive()) {
            runFullAutomatedDiagnosticSequence();
        }

        // =====================================================================
        // 2. MAIN DIAGNOSTIC LOOP (Displays results & allows manual re-tests)
        // =====================================================================
        while (opModeIsActive()) {

            // Emergency stop via Left Bumper
            if (gamepad1.left_bumper) {
                stopAllOutputs();
                isAutomatedSequenceRunning = false;
            }

            if (isAutomatedSequenceRunning) {
                // Automated sequence mode handled separately
                continue;
            }

            // -----------------------------------------------------------------
            // A. DEVICE SELECTION (D-Pad UP / DOWN)
            // -----------------------------------------------------------------
            boolean currentDpadUp = gamepad1.dpad_up;
            if (currentDpadUp && !prevDpadUp) {
                selectedDeviceIndex = (selectedDeviceIndex - 1 + TOTAL_DEVICES) % TOTAL_DEVICES;
            }
            prevDpadUp = currentDpadUp;

            boolean currentDpadDown = gamepad1.dpad_down;
            if (currentDpadDown && !prevDpadDown) {
                selectedDeviceIndex = (selectedDeviceIndex + 1) % TOTAL_DEVICES;
            }
            prevDpadDown = currentDpadDown;

            // -----------------------------------------------------------------
            // B. RESET ENCODER (Button Y - For Motors Only)
            // -----------------------------------------------------------------
            boolean currentY = gamepad1.y;
            if (currentY && !prevY) {
                if (selectedDeviceIndex < 8) {
                    DcMotorEx activeMotor = motors[selectedDeviceIndex];
                    if (activeMotor != null) {
                        DcMotor.RunMode prevMode = activeMotor.getMode();
                        activeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        activeMotor.setMode(prevMode);
                    }
                }
            }
            prevY = currentY;

            // -----------------------------------------------------------------
            // C. SINGLE DEVICE PULSE TEST (Button X)
            // -----------------------------------------------------------------
            boolean currentX = gamepad1.x;
            if (currentX && !prevX) {
                if (selectedDeviceIndex < 8) {
                    runSingleMotorPulseTest(selectedDeviceIndex);
                } else {
                    runServoPulseTest();
                }
            }
            prevX = currentX;

            // -----------------------------------------------------------------
            // D. ALL-DEVICE AUTOMATED DIAGNOSTIC ROUTINE (Right Bumper)
            // -----------------------------------------------------------------
            boolean currentRB = gamepad1.right_bumper;
            if (currentRB && !prevRB) {
                runFullAutomatedDiagnosticSequence();
            }
            prevRB = currentRB;

            // -----------------------------------------------------------------
            // E. MANUAL DEVICE CONTROLS (Hold A for Forward/Outward, Hold B for Reverse/Home)
            // -----------------------------------------------------------------
            stopAllMotorsOnly(); // Default state for motors

            if (selectedDeviceIndex < 8) {
                DcMotorEx activeMotor = motors[selectedDeviceIndex];
                if (activeMotor != null) {
                    if (gamepad1.a) {
                        if (selectedDeviceIndex == 4) { // Shooter motor uses velocity
                            activeMotor.setVelocity(DEFAULT_SHOOTER_VELOCITY);
                        } else {
                            activeMotor.setPower(TEST_POWER);
                        }
                    } else if (gamepad1.b) {
                        if (selectedDeviceIndex == 4) { // Shooter motor velocity
                            activeMotor.setVelocity(-DEFAULT_SHOOTER_VELOCITY);
                        } else {
                            activeMotor.setPower(-TEST_POWER);
                        }
                    }
                }
            } else if (ballservo != null && servoConnected) {
                // Manual Servo control
                if (gamepad1.a) {
                    ballservo.setPosition(SERVO_OUTWARD_POSITION); // Move 90° outward (0.90)
                } else if (gamepad1.b) {
                    ballservo.setPosition(SERVO_HOME_POSITION); // Return home (0.00)
                }
            }

            // Update peak velocities for live feedback
            for (int i = 0; i < 8; i++) {
                if (motors[i] != null) {
                    try {
                        double currentVel = Math.abs(motors[i].getVelocity());
                        if (currentVel > peakVelocities[i]) {
                            peakVelocities[i] = currentVel;
                        }
                    } catch (Exception ignored) {
                    }
                }
            }

            // -----------------------------------------------------------------
            // F. DIAGNOSTIC TELEMETRY DISPLAY
            // -----------------------------------------------------------------
            telemetry.addData("=== AUTONOMOUS DIAGNOSTIC TEST ===", "");
            telemetry.addData("Status", "ACTIVE - Uptime: %.1fs", getRuntime());
            telemetry.addData("Controls", "D-Pad Up/Down: Select Device | Hold A: Fwd/90° | Hold B: Rev/Home");
            telemetry.addData("Controls", "X: Quick Test Selected | RB: Test ALL Devices | Y: Reset Motor Encoder");
            telemetry.addLine();

            telemetry.addData("--- ACTIVE DEVICE ---", "[%d/%d] %s",
                    selectedDeviceIndex + 1, TOTAL_DEVICES, deviceNames[selectedDeviceIndex]);
            telemetry.addData("HW Config Name", hardwareConfigNames[selectedDeviceIndex]);
            telemetry.addData("Location", hubLocations[selectedDeviceIndex]);

            if (selectedDeviceIndex < 8) {
                // Motor telemetry
                DcMotorEx activeMotor = motors[selectedDeviceIndex];
                if (activeMotor != null && motorConnected[selectedDeviceIndex]) {
                    double velTPS = activeMotor.getVelocity();
                    double rpm = (velTPS / TICKS_PER_REV) * 60.0;
                    int pos = activeMotor.getCurrentPosition();
                    double power = activeMotor.getPower();

                    telemetry.addData("Connection Status", "CONNECTED [OK]");
                    telemetry.addData("Current Power Setting", "%.2f", power);
                    telemetry.addData("Live Velocity (TPS)", "%.1f ticks/sec", velTPS);
                    telemetry.addData("Calculated RPM", "%.1f RPM", rpm);
                    telemetry.addData("Encoder Position", "%d ticks (Initial: %d)", pos, startEncoderPositions[selectedDeviceIndex]);
                    telemetry.addData("Calculated Rotations", "%.2f revs", pos / TICKS_PER_REV);
                    telemetry.addData("Peak Velocity Recorded", "%.1f ticks/sec", peakVelocities[selectedDeviceIndex]);
                    telemetry.addData("Last Test Status", diagnosticStatus[selectedDeviceIndex]);
                } else {
                    telemetry.addData("Connection Status", "NOT CONNECTED / NOT FOUND IN HARDWARE MAP");
                }
            } else {
                // Servo telemetry
                if (ballservo != null && servoConnected) {
                    double currentPos = ballservo.getPosition();
                    telemetry.addData("Connection Status", "CONNECTED [OK]");
                    telemetry.addData("Current Servo Position", "%.2f", currentPos);
                    telemetry.addData("Home Position", "%.2f", SERVO_HOME_POSITION);
                    telemetry.addData("Outward 90° Position", "%.2f", SERVO_OUTWARD_POSITION);
                    telemetry.addData("Last Test Status", diagnosticStatus[selectedDeviceIndex]);
                } else {
                    telemetry.addData("Connection Status", "NOT CONNECTED / NOT FOUND IN HARDWARE MAP");
                }
            }

            telemetry.addLine();
            telemetry.addLine("=================================");
            telemetry.addLine(" ALL HARDWARE REAL-TIME STATUS SUMMARY ");
            telemetry.addLine("=================================");

            for (int i = 0; i < TOTAL_DEVICES; i++) {
                String prefix = (i == selectedDeviceIndex) ? "-> " : "   ";
                if (i < 8) {
                    if (motors[i] != null && motorConnected[i]) {
                        double vel = motors[i].getVelocity();
                        int pos = motors[i].getCurrentPosition();
                        telemetry.addData(prefix + "[" + (i + 1) + "] " + hardwareConfigNames[i],
                                "Vel: %.0f tps | Pos: %d | Status: %s", vel, pos, diagnosticStatus[i]);
                    } else {
                        telemetry.addData(prefix + "[" + (i + 1) + "] " + hardwareConfigNames[i], "DISCONNECTED");
                    }
                } else {
                    if (ballservo != null && servoConnected) {
                        double pos = ballservo.getPosition();
                        telemetry.addData(prefix + "[" + (i + 1) + "] " + hardwareConfigNames[i],
                                "Pos: %.2f | Status: %s", pos, diagnosticStatus[i]);
                    } else {
                        telemetry.addData(prefix + "[" + (i + 1) + "] " + hardwareConfigNames[i], "DISCONNECTED");
                    }
                }
            }

            telemetry.update();
        }

        stopAllOutputs();
    }

    /**
     * Executes a 3-second automated pulse diagnostic test on a single selected motor.
     */
    private void runSingleMotorPulseTest(int index) {
        DcMotorEx motor = motors[index];
        if (motor == null || !motorConnected[index]) {
            diagnosticStatus[index] = "FAILED (Disconnected)";
            return;
        }

        stopAllMotorsOnly();
        diagnosticStatus[index] = "TESTING IN PROGRESS...";
        telemetry.addData("Diagnostic Test", "Testing %s...", deviceNames[index]);
        telemetry.update();

        int startPos = motor.getCurrentPosition();
        double maxVelocity = 0.0;

        // Phase 1: Forward run for 1.2 seconds
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.2 && !gamepad1.left_bumper) {
            if (index == 4) { // Shooter motor
                motor.setVelocity(DEFAULT_SHOOTER_VELOCITY);
            } else {
                motor.setPower(TEST_POWER);
            }

            double currentVel = Math.abs(motor.getVelocity());
            if (currentVel > maxVelocity) {
                maxVelocity = currentVel;
            }

            telemetry.addData("Diagnostic Test", "Phase 1/2 Forward Test: %s", deviceNames[index]);
            telemetry.addData("Velocity", "%.1f ticks/sec", currentVel);
            telemetry.update();
        }

        // Phase 2: Reverse run for 1.2 seconds
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.2 && !gamepad1.left_bumper) {
            if (index == 4) { // Shooter motor
                motor.setVelocity(-DEFAULT_SHOOTER_VELOCITY);
            } else {
                motor.setPower(-TEST_POWER);
            }

            double currentVel = Math.abs(motor.getVelocity());
            if (currentVel > maxVelocity) {
                maxVelocity = currentVel;
            }

            telemetry.addData("Diagnostic Test", "Phase 2/2 Reverse Test: %s", deviceNames[index]);
            telemetry.addData("Velocity", "%.1f ticks/sec", currentVel);
            telemetry.update();
        }

        motor.setPower(0);
        motor.setVelocity(0);

        int endPos = motor.getCurrentPosition();
        int deltaTicks = Math.abs(endPos - startPos);
        peakVelocities[index] = maxVelocity;

        if (gamepad1.left_bumper) {
            diagnosticStatus[index] = "CANCELLED (Emergency Stop)";
            return;
        }

        // Pass criterion: Measured peak velocity > 50 ticks/sec or encoder moved significantly
        if (maxVelocity > 50.0 || deltaTicks > 20) {
            diagnosticStatus[index] = String.format("PASSED (Peak: %.0f tps, Delta: %d ticks)", maxVelocity, deltaTicks);
        } else {
            diagnosticStatus[index] = "WARNING (Low/No Movement Detected)";
        }
    }

    /**
     * Executes an automated diagnostic pulse test on the test Servo motor ("servo").
     */
    private void runServoPulseTest() {
        if (ballservo == null || !servoConnected) {
            diagnosticStatus[8] = "FAILED (Disconnected)";
            return;
        }

        diagnosticStatus[8] = "TESTING IN PROGRESS...";
        telemetry.addData("Diagnostic Test", "Testing Servo: %s...", deviceNames[8]);
        telemetry.update();

        // Phase 1: Move outward 90° (position 0.90) and hold for 0.8 seconds
        ballservo.setPosition(SERVO_OUTWARD_POSITION);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 0.8 && !gamepad1.left_bumper) {
            telemetry.addData("Diagnostic Test", "Phase 1/2 Servo Outward 90° (0.90)");
            telemetry.update();
        }

        // Phase 2: Return to home (position 0.00) and hold for 0.8 seconds
        ballservo.setPosition(SERVO_HOME_POSITION);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 0.8 && !gamepad1.left_bumper) {
            telemetry.addData("Diagnostic Test", "Phase 2/2 Servo Return Home (0.00)");
            telemetry.update();
        }

        if (gamepad1.left_bumper) {
            diagnosticStatus[8] = "CANCELLED (Emergency Stop)";
        } else {
            diagnosticStatus[8] = String.format("PASSED (Positions Tested: %.2f <-> %.2f)",
                    SERVO_HOME_POSITION, SERVO_OUTWARD_POSITION);
        }
    }

    /**
     * Runs automated sequential diagnostic test on all 9 devices (8 motors + 1 servo) consecutively.
     */
    private void runFullAutomatedDiagnosticSequence() {
        isAutomatedSequenceRunning = true;
        stopAllOutputs();

        // Test all 8 motors
        for (int i = 0; i < 8 && opModeIsActive(); i++) {
            if (gamepad1.left_bumper) {
                break; // Emergency stop
            }
            selectedDeviceIndex = i;
            runSingleMotorPulseTest(i);
            sleep(200);
        }

        // Test servo (device 9, index 8)
        if (opModeIsActive() && !gamepad1.left_bumper) {
            selectedDeviceIndex = 8;
            runServoPulseTest();
        }

        stopAllOutputs();
        isAutomatedSequenceRunning = false;
    }

    /**
     * Stops all 8 motors only.
     */
    private void stopAllMotorsOnly() {
        for (int i = 0; i < 8; i++) {
            if (motors[i] != null && motorConnected[i]) {
                try {
                    motors[i].setPower(0.0);
                    if (i == 4) {
                        motors[i].setVelocity(0.0);
                    }
                } catch (Exception ignored) {
                }
            }
        }
    }

    /**
     * Safely stops all 8 motors and returns servo to home position.
     */
    private void stopAllOutputs() {
        stopAllMotorsOnly();
        if (ballservo != null && servoConnected) {
            try {
                ballservo.setPosition(SERVO_HOME_POSITION);
            } catch (Exception ignored) {
            }
        }
    }

    /**
     * Counts total connected devices (motors + servo).
     */
    private int countConnectedDevices() {
        int count = 0;
        for (boolean connected : motorConnected) {
            if (connected) count++;
        }
        if (servoConnected) count++;
        return count;
    }
}
