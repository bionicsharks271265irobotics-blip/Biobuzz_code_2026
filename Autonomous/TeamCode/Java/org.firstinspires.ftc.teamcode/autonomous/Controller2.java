/*
 * =============================================================================
 * TELEOP OPMODE: ControllerII
 * =============================================================================
 * PURPOSE:
 * Full TeleOp control program utilizing two gamepads for driver and operator controls.
 *
 * CONTROLS OVERVIEW:
 * - Gamepad 1 (Driver): Mecanum drivetrain movement (drive, strafe, turn) with 
 *   speed modes (Turbo on RB, Slow on LB, Normal by default).
 * - Gamepad 2 (Operator): Shooter flywheel control (triggers for variable power, 
 *   preset buttons Y/A/X/B, D-Pad speed tuning, RB direction toggle) and dual 
 *   intake motor control (LT for intake in, LB for reverse/outtake).
 *
 * HARDWARE CONFIGURATION:
 * - Drive Motors: "fLeft", "bLeft", "fRight", "bRight"
 * - Shooter Motor: "shootermotor"
 * - Intake Motors: "intake1", "intake2"
 * =============================================================================
 */
package org.firstinspires.ftc.teamcode;

// Import basic FTC tools to control the robot motors and gamepads
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * =============================================================================
 *                      CONTROLLER BUTTON INSTRUCTIONS
 * =============================================================================
 * 
 * -----------------------------------------------------------------------------
 * GAMEPAD 1: DRIVER (Drivetrain & Movement)
 * -----------------------------------------------------------------------------
 *  - Left Joystick (Push Up / Down)    : Drive Forward and Backward
 *  - Left Joystick (Push Left / Right)  : Strafe Sideways (Left and Right)
 *  - Right Joystick (Push Left / Right) : Rotate / Turn Robot Left and Right
 *  - Right Bumper (Hold RB)             : Turbo Fast Mode (100% Speed)
 *  - Left Bumper (Hold LB)              : Slow Precision Mode (35% Speed)
 *  - Default Speed (No Bumpers)         : Normal Driving Mode (85% Speed)
 *
 * -----------------------------------------------------------------------------
 * GAMEPAD 2: OPERATOR (Shooter Mechanism & Intake)
 * -----------------------------------------------------------------------------
 *  - Right Trigger (Press / Squeeze)    : Variable Shooter Power (0% to 100%)
 *  - Right Bumper (Press RB)            : Switch / Reverse Shooter Spin Direction
 *  - Left Trigger (Press / Squeeze)     : Variable Intake IN Power (0% to 100%)
 *  - Left Bumper (Hold LB)              : Reverse / Outtake (-100% Power)
 *  - Y Button (Press)                   : High Speed Shot (~2000 ticks/sec / 100% Power)
 *  - A Button (Press)                   : Medium Speed Shot (~1500 ticks/sec / 75% Power)
 *  - X Button (Press)                   : Low Speed Shot (~1000 ticks/sec / 50% Power)
 *  - B Button (Press)                   : STOP Shooter Motor Immediately
 *  - D-Pad UP (Press)                   : Fine-tune Speed UP (+50 ticks/sec or +5% Power)
 *  - D-Pad DOWN (Press)                 : Fine-tune Speed DOWN (-50 ticks/sec or -5% Power)
 * 
 * -----------------------------------------------------------------------------
 * HOW TO PAIR LOGITECH F310 CONTROLLERS ON DRIVER STATION PHONE:
 * -----------------------------------------------------------------------------
 *  - Switch on back of F310 MUST be set to 'X' (XInput mode)
 *  - MODE light next to left stick MUST be OFF
 *  - Gamepad 1 (Driver)   : Press START + A  (Connects to Slot 1 / Left Icon)
 *  - Gamepad 2 (Operator) : Press START + B  (Connects to Slot 2 / Right Icon)
 * 
 * -----------------------------------------------------------------------------
 * MOTOR CONFIGURATION NAMES (Set in Robot Controller App):
 * -----------------------------------------------------------------------------
 *  - "fLeft"        : Front Left Wheel Motor
 *  - "bLeft"        : Back Left Wheel Motor
 *  - "fRight"       : Front Right Wheel Motor
 *  - "bRight"       : Back Right Wheel Motor
 *  - "shootermotor" : Shooter Flywheel Motor
 *  - "intake1"      : First Intake Motor
 *  - "intake2"      : Second Intake Motor
 * =============================================================================
 */

// Show this program under TeleOp on the Driver Station phone screen
@TeleOp(name = "ControllerII", group = "TeleOp")
public class ControllerII extends LinearOpMode {

    // =========================================================================
    // NUMBERS AND SETTINGS
    // =========================================================================

    // How many encoder ticks make 1 full turn of the motor wheel (537.7 ticks)
    public static final double TICKS_PER_REV = 537.7;

    // How much to speed up or slow down the shooter when using D-Pad Up / Down
    public static final double VELOCITY_STEP = 50.0;

    // =========================================================================
    // MOTORS
    // =========================================================================

    // The 4 wheels on the robot
    private DcMotor fLeft;  // Front Left wheel
    private DcMotor bLeft;  // Back Left wheel
    private DcMotor fRight; // Front Right wheel
    private DcMotor bRight; // Back Right wheel

    // The shooter motor (DcMotorEx gives us speed control)
    private DcMotorEx shooterMotor;

    // The two intake motors
    private DcMotor intake1;
    private DcMotor intake2;
    // the poo of the woo too is the poo of the too woo but sadly the woo of to cow is too poo to be the too. - cow
    // =========================================================================
    // VARIABLES TO STORE ROBOT STATE
    // =========================================================================

    // Shooter speed setting in percent power (0.0 to 1.0)
    private double targetShooterPower = 0.0;

    // Shooter speed setting in motor ticks per second
    private double targetShooterVelocity = 0.0;

    // True = use exact motor speed; False = use basic percent power
    private boolean useVelocityControl = false;

    // True = shooter turns in reversed direction; False = normal direction
    private boolean isShooterReversed = false;

    // Remember if buttons were pressed in the last loop, so pressing a button only counts once
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevX = false;
    private boolean prevA = false;
    private boolean prevY = false;
    private boolean prevRightBumper2 = false;

    /**
     * Main program loop that runs when you press INIT and START on the phone.
     */
    @Override
    public void runOpMode() {

        // Update the phone screen 50 times per second
        telemetry.setMsTransmissionInterval(20);

        // =====================================================================
        // 1. SETUP MOTORS (When you press INIT)
        // =====================================================================

        // Find the 4 drive motors from the robot config
        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bRight = hardwareMap.get(DcMotor.class, "bRight");

        // Reverse left side motors so pushing forward moves all wheels forward
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        // Make the wheels stop quickly when you let go of the joysticks
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Find the shooter motor from the robot config
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");

        // Set initial shooter motor direction to REVERSE
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Let the shooter wheel coast smoothly to a stop when turned off
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Reset the shooter motor position sensor to 0
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Find the two intake motors from the robot config
        try {
            intake1 = hardwareMap.get(DcMotor.class, "intake1");
            intake1.setDirection(DcMotor.Direction.FORWARD);
            intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            intake1 = null;
        }

        try {
            intake2 = hardwareMap.get(DcMotor.class, "intake2");
            intake2.setDirection(DcMotor.Direction.REVERSE); // Reverse second motor so both spin together
            intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            intake2 = null;
        }

        // Show controller instructions on the driver station phone screen when INIT is pressed
        telemetry.addData("Status", "Initialized - Ready for Start!");
        telemetry.addData("--- GAMEPAD 1 (DRIVER) ---", "Left Stick: Drive/Strafe | Right Stick: Turn | RB: Fast | LB: Slow");
        telemetry.addData("--- GAMEPAD 2 (OPERATOR) ---", "RT: Shooter | LT: Intake IN | LB: Intake OUT | RB: Rev Shooter | Y/A/X: Speed");
        telemetry.update();

        // Wait here until the driver presses the START button on the phone
        waitForStart();

        // Reset timer
        resetRuntime();

        // =====================================================================
        // 2. MAIN LOOP (Runs again and again while match is active)
        // =====================================================================
        while (opModeIsActive()) {

            // -----------------------------------------------------------------
            // A. CONTROLLER 1: DRIVE AND STRAFE
            // -----------------------------------------------------------------

            // Read joystick directions:
            // - Left Stick Y  : Push UP/DOWN to move Forward/Backward (inverted with -)
            // - Left Stick X  : Push LEFT/RIGHT to Strafe Sideways (* 1.1 helps strafe straight)
            // - Right Stick X : Push LEFT/RIGHT to Turn/Rotate Robot Left and Right
            double y = -gamepad1.left_stick_y;       // Forward and Backward
            double x = gamepad1.left_stick_x * 1.1;   // Left and Right strafe
            double rx = gamepad1.right_stick_x;       // Turn Left and Right

            // Choose robot speed using bumper buttons:
            // - Default Speed (No Bumpers) = 85% Speed
            // - Right Bumper (RB)          = Fast Turbo Mode (100% Speed)
            // - Left Bumper (LB)           = Slow Precision Mode (35% Speed)
            double speedMultiplier = 0.85;
            if (gamepad1.right_bumper) {
                speedMultiplier = 1.0;
            } else if (gamepad1.left_bumper) {
                speedMultiplier = 0.35;
            }

            // Calculate power for each wheel to drive, strafe, and turn at the same time
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = ((y + x + rx) / denominator) * speedMultiplier;
            double backLeftPower   = ((y - x + rx) / denominator) * speedMultiplier;
            double frontRightPower = ((y - x - rx) / denominator) * speedMultiplier;
            double backRightPower  = ((y + x - rx) / denominator) * speedMultiplier;
// ducky's green shirt on 10/15/2026 is very cool
            // Send power to the wheel motors
            fLeft.setPower(frontLeftPower);
            bLeft.setPower(backLeftPower);
            fRight.setPower(frontRightPower);
            bRight.setPower(backRightPower);


            // -----------------------------------------------------------------
            // B. CONTROLLER 2: SHOOTER CONTROL & DIRECTION SWITCH
            // -----------------------------------------------------------------

            // RIGHT BUMPER (RB): Press to switch/reverse the direction the shooter gear spins
            boolean currentRightBumper2 = gamepad2.right_bumper;
            if (currentRightBumper2 && !prevRightBumper2) {
                isShooterReversed = !isShooterReversed;
                shooterMotor.setDirection(isShooterReversed ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
            }
            prevRightBumper2 = currentRightBumper2;

            // Read Right Trigger (0.0 means not pressed, 1.0 means pressed all the way)
            double rightTrigger = gamepad2.right_trigger;

            // RIGHT TRIGGER: Squeeze to manually control shooter power (0% to 100%)
            if (rightTrigger > 0.05) {
                useVelocityControl = false;
                targetShooterPower = rightTrigger;
            } else {
                // BUTTON PRESETS:
                // - Y Button : High Speed Preset  (~2000 ticks/sec or 100% Power)
                // - A Button : Medium Speed Preset (~1500 ticks/sec or 75% Power)
                // - X Button : Low Speed Preset    (~1000 ticks/sec or 50% Power)
                // - B Button : STOP Shooter Motor Immediately
                if (gamepad2.y && !prevY) {
                    useVelocityControl = true;
                    targetShooterVelocity = 2000.0;
                    targetShooterPower = 1.0;
                } else if (gamepad2.a && !prevA) {
                    useVelocityControl = true;
                    targetShooterVelocity = 1500.0;
                    targetShooterPower = 0.75;
                } else if (gamepad2.x && !prevX) {
                    useVelocityControl = true;
                    targetShooterVelocity = 1000.0;
                    targetShooterPower = 0.50;
                } else if (gamepad2.b) {
                    useVelocityControl = false;
                    targetShooterPower = 0.0;
                    targetShooterVelocity = 0.0;
                }
    // neal is so fast
                // D-PAD CONTROLS:
                // - D-Pad UP   : Speed UP (+50 ticks/sec or +5% Power)
                // - D-Pad DOWN : Speed DOWN (-50 ticks/sec or -5% Power)
                if (gamepad2.dpad_up && !prevDpadUp) {
                    if (useVelocityControl) {
                        targetShooterVelocity += VELOCITY_STEP;
                    } else {
                        targetShooterPower = Math.min(1.0, targetShooterPower + 0.05);
                    }
                }
                if (gamepad2.dpad_down && !prevDpadDown) {
                    if (useVelocityControl) {
                        targetShooterVelocity = Math.max(0, targetShooterVelocity - VELOCITY_STEP);
                    } else {
                        targetShooterPower = Math.max(0.0, targetShooterPower - 0.05);
                    }
                }
            }

            // Remember button presses so holding a button doesn't spam the code
            prevY = gamepad2.y;
            prevA = gamepad2.a;
            prevX = gamepad2.x;
            prevDpadUp = gamepad2.dpad_up;
            prevDpadDown = gamepad2.dpad_down;

            // Turn on shooter motor at the target speed or power
            if (useVelocityControl && targetShooterVelocity > 0) {
                shooterMotor.setVelocity(targetShooterVelocity);
            } else {
                shooterMotor.setPower(targetShooterPower);
            }


            // -----------------------------------------------------------------
            // C. CONTROLLER 2: INTAKE MOTORS CONTROL
            // -----------------------------------------------------------------

            // Read Left Trigger and Left Bumper for Intake control
            double leftTrigger2 = gamepad2.left_trigger;
            double intakePower = 0.0;

            if (gamepad2.left_bumper) {
                // Left Bumper: Reverse / Outtake (-100% Power)
                intakePower = -1.0;
            } else if (leftTrigger2 > 0.05) {
                // Left Trigger: Intake IN (Variable power 0% to 100%)
                intakePower = leftTrigger2;
            }

            // Send power to both intake motors
            if (intake1 != null) {
                intake1.setPower(intakePower);
            }
            if (intake2 != null) {
                intake2.setPower(intakePower);
            }


            // -----------------------------------------------------------------
            // D. DISPLAY INFO ON THE DRIVER PHONE SCREEN
            // -----------------------------------------------------------------

            // Get shooter speed from motor sensor
            double currentVelocityTicks = shooterMotor.getVelocity();

            // Calculate shooter RPM (turns per minute)
            double currentRPM = (currentVelocityTicks / TICKS_PER_REV) * 60.0;

            // Show drive info for Controller 1
            telemetry.addData("=== DRIVETRAIN (Gamepad 1) ===", "");
            telemetry.addData("Drive Mode", gamepad1.right_bumper ? "TURBO (100%)" : (gamepad1.left_bumper ? "SLOW (35%)" : "NORMAL (85%)"));
            telemetry.addData("Motor Powers", "FL:%.2f | FR:%.2f | BL:%.2f | BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            // Show shooter info for Controller 2
            telemetry.addData("\n=== SHOOTER (Gamepad 2) ===", "");
            telemetry.addData("Shooter Direction", isShooterReversed ? "FORWARD (Reversed)" : "REVERSE (Normal)");
            if (useVelocityControl) {
                telemetry.addData("Control Mode", "Exact Speed Control");
                telemetry.addData("Target Velocity", "%.1f ticks/sec", targetShooterVelocity);
            } else {
                telemetry.addData("Control Mode", "Percent Power Control");
                telemetry.addData("Target Power", "%.2f (%.0f%%)", targetShooterPower, targetShooterPower * 100);
            }
            telemetry.addData("Actual Speed", "%.1f ticks/sec (%.1f RPM)", currentVelocityTicks, currentRPM);
            telemetry.addData("Shooter Sensor Ticks", "%d ticks", shooterMotor.getCurrentPosition());

            // Show intake info for Controller 2
            telemetry.addData("\n=== INTAKE (Gamepad 2 LT / LB) ===", "");
            if (intake1 != null || intake2 != null) {
                telemetry.addData("Intake Power", "%.2f (%s)", intakePower,
                        intakePower > 0 ? "INTAKE IN" : (intakePower < 0 ? "REVERSE / OUT" : "STOPPED"));
            } else {
                telemetry.addData("CONFIG NOTICE", "Intake motors ('intake1', 'intake2') not configured");
            }

            // Send info to phone screen
            telemetry.update();
        }
// ayyansh is a bad boy
        // =====================================================================
        // 3. STOP ROBOT (When match finishes or user presses STOP)
        // =====================================================================
        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
        shooterMotor.setPower(0);
    }
}
