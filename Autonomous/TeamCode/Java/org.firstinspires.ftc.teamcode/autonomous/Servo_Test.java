/*
 * =============================================================================
 * TELEOP OPMODE: SERVO_TEST
 * =============================================================================
 * PURPOSE:
 * Advanced TeleOp OpMode testing full mecanum driving, dual shooter flywheels, 
 * dual intake motors, and an automated servo mechanism with timed 90-degree cycling 
 * and angle fine-tuning.
 *
 * CONTROLS OVERVIEW:
 * - Gamepad 1 (Driver): Mecanum driving, strafe, turn, fast/slow speed modes, 
 *   and quick shooter/outtake triggers.
 * - Gamepad 2 (Operator):
 *   * Shooter: Triggers for variable power, preset speed buttons (Y/A/X/B), D-Pad speed tuning, RB direction toggle.
 *   * Intake: LT for Intake IN, L3 for Outtake.
 *   * Servo: LB triggers automated 90° clockwise turn & auto-return sequence; D-Pad Left/Right fine-tunes servo angle target.
 *
 * HARDWARE CONFIGURATION:
 * - Drivetrain: "fLeft", "bLeft", "fRight", "bRight"
 * - Shooter Motors: "shootermotor"/"shooter1", "shootermotor2"/"shooter2"
 * - Intake Motors: "intake1", "intake2"
 * - Servo: "servo"
 * =============================================================================
 */
package org.firstinspires.ftc.teamcode;

// Import basic FTC tools to control the robot motors and gamepads
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * =============================================================================
 *                      CONTROLLER BUTTON INSTRUCTIONS
 * =============================================================================
 *
 * -----------------------------------------------------------------------------
 * GAMEPAD 1: DRIVER (Drivetrain, Movement & Shooter / Intake Reverse)
 * -----------------------------------------------------------------------------
 *  - Left Joystick (Push Up / Down)    : Drive Forward and Backward
 *  - Left Joystick (Push Left / Right)  : Strafe Sideways (Left and Right)
 *  - Right Joystick (Push Left / Right) : Rotate / Turn Robot Left and Right
 *  - Right Bumper (Hold RB)             : Turbo Fast Mode (100% Speed)
 *  - Left Bumper (Hold LB)              : Slow Precision Mode (35% Speed)
 *  - Right Trigger (Press / Squeeze)    : Variable Shooter Power (0% to 100%)
 *  - Left Trigger (Press / Squeeze)     : Reverse / Outtake Intake (-100% Power)
 *  - Y Button (Press)                   : High Speed Shot (~2000 ticks/sec / 100% Power)
 *  - A Button (Press)                   : Medium Speed Shot (~1500 ticks/sec / 75% Power)
 *  - X Button (Press)                   : Low Speed Shot (~1000 ticks/sec / 50% Power)
 *  - B Button (Press)                   : STOP Shooter Motor Immediately
 *  - D-Pad UP / DOWN (Press)            : Fine-tune Speed UP / DOWN
 *  - Right Stick Button (Press R3)      : Switch / Reverse Shooter Spin Direction
 *
 * -----------------------------------------------------------------------------
 * GAMEPAD 2: OPERATOR (Shooter Mechanism, Intake & Servo)
 * -----------------------------------------------------------------------------
 *  - Right Trigger (Press / Squeeze)    : Variable Shooter Power (0% to 100%)
 *  - Right Bumper (Press RB)            : Switch / Reverse Shooter Spin Direction
 *  - Left Trigger (Press / Squeeze)     : Variable Intake IN Power (0% to 100%)
 *  - Left Stick Button (Press L3)       : Reverse / Outtake Intake (-100% Power)
 *  - Left Bumper (Press LB)             : Toggle Servo 90° Outward (Clockwise) / Original
 *  - Y Button (Press)                   : High Speed Shot (~2000 ticks/sec / 100% Power)
 *  - A Button (Press)                   : Medium Speed Shot (~1500 ticks/sec / 75% Power)
 *  - X Button (Press)                   : Low Speed Shot (~1000 ticks/sec / 50% Power)
 *  - B Button (Press)                   : STOP Shooter Motor Immediately
 *  - D-Pad UP (Press)                   : Fine-tune Speed UP (+50 ticks/sec or +5% Power)
 *  - D-Pad DOWN (Press)                 : Fine-tune Speed DOWN (-50 ticks/sec or -5% Power)
 *  - D-Pad LEFT / RIGHT (Press)         : Fine-tune Servo 90° Position (+/- 0.05)
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
 * MOTOR & SERVO CONFIGURATION NAMES (Set in Robot Controller App):
 * -----------------------------------------------------------------------------
 *  - "fLeft"        : Front Left Wheel Motor
 *  - "bLeft"        : Back Left Wheel Motor
 *  - "fRight"       : Front Right Wheel Motor
 *  - "bRight"       : Back Right Wheel Motor
 *  - "shootermotor" / "shooter1"  : Primary Shooter Flywheel Motor
 *  - "shootermotor2" / "shooter2" : Optional Secondary Shooter Flywheel Motor
 *  - "intake1"      : First Intake Motor
 *  - "intake2"      : Second Intake Motor
 *  - "servo"        : Test Servo Motor
 * =============================================================================
 */

// Show this program under TeleOp on the Driver Station phone screen
@TeleOp(name = "SERVO_TEST", group = "TeleOp")
public class SERVO_TEST extends LinearOpMode {

    // =========================================================================
    // NUMBERS AND SETTINGS
    // =========================================================================

    // How many encoder ticks make 1 full turn of the motor wheel (537.7 ticks)
    public static final double TICKS_PER_REV = 537.7;

    // How much to speed up or slow down the shooter when using D-Pad Up / Down
    public static final double VELOCITY_STEP = 50.0;

    // Servo position & timing settings
    public static final double SERVO_ORIGINAL_POSITION = 0.0;  // 9 o'clock position (0.0)
    public static final double SERVO_HOLD_TIME_SEC = 0.50;     // Time (seconds) to hold 90° before returning

    // Tunable outward position (defaults to 0.90)
    private double servoOutwardPosition = 0.90;

    // Stores current target position sent to servo
    private double currentServoPosition = SERVO_ORIGINAL_POSITION;

    // Timer variables for automatic 90° clockwise and return sequence
    private boolean isServoCycling = false;
    private double servoCycleStartTime = 0.0;

    // =========================================================================
    // MOTORS & SERVOS
    // =========================================================================

    // The 4 wheels on the robot
    private DcMotor fLeft;  // Front Left wheel
    private DcMotor bLeft;  // Back Left wheel
    private DcMotor fRight; // Front Right wheel
    private DcMotor bRight; // Back Right wheel

    // The shooter motors (DcMotorEx gives us speed control)
    private DcMotorEx shooterMotor;  // Main shooter motor ("shootermotor" or "shooter1")
    private DcMotorEx shooterMotor2; // Optional second flywheel motor ("shootermotor2" or "shooter2")

    // The two intake motors
    private DcMotor intake1;
    private DcMotor intake2;

    // The servo motor
    private Servo ballservo;

    // =========================================================================
    // VARIABLES TO STORE ROBOT STATE
    // =========================================================================

    // Shooter speed setting in percent power (0.0 to 1.0)
    private double targetShooterPower = 0.0;

    // Shooter speed setting in motor ticks per second
    private double targetShooterVelocity = 0.0;

    // True = use exact motor speed; False = use basic percent power
    private boolean useVelocityControl = false;
    //TACO BELL IS THE BEST🌮🔔
    // True = shooter turns in reversed direction; False = normal direction
    private boolean isShooterReversed = false;

    // Remember if buttons were pressed in the last loop, so pressing a button only counts once
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
//neal is the goat
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

        // Run drive motors directly without encoder speed capping
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Find primary shooter motor from the robot config
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
        } catch (Exception e) {
            try {
                shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter1");
            } catch (Exception ex) {
                shooterMotor = null;
            }
        }

        if (shooterMotor != null) {
            shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Find optional secondary shooter motor
        try {
            shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shootermotor2");
        } catch (Exception e) {
            try {
                shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            } catch (Exception ex) {
                shooterMotor2 = null;
            }
        }
//roni+sana4ever
        if (shooterMotor2 != null) {
            shooterMotor2.setDirection(DcMotorEx.Direction.FORWARD);
            shooterMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Find the servo from the robot config and set it to original position
        try {
            ballservo = hardwareMap.get(Servo.class, "servo");

            if (ballservo instanceof PwmControl) {
                ((PwmControl) ballservo).setPwmEnable();
            }

            currentServoPosition = SERVO_ORIGINAL_POSITION;
            ballservo.setPosition(currentServoPosition);
        } catch (Exception e) {
            ballservo = null;
        }

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
            intake2.setDirection(DcMotor.Direction.REVERSE);
            intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            intake2 = null;
        }

        // Show controller instructions on the driver station phone screen when INIT is pressed
        telemetry.addData("Status", "Initialized - Ready for Start!");
        telemetry.addData("--- GAMEPAD 1 (DRIVER) ---", "Left Stick: Drive/Strafe | Right Stick: Turn | RB: Fast | LB: Slow | RT: Shooter | LT: Outtake");
        telemetry.addData("--- GAMEPAD 2 (OPERATOR) ---", "RT: Shooter | LT: Intake IN | L3: Outtake | LB: Servo 90° | RB: Rev Shooter | Y/A/X: Speed");
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
            // - Default Speed (No Bumpers) = 100% Full Speed
            // - Left Bumper (LB)           = Slow Precision Mode (35% Speed)
            double speedMultiplier = 1.0;
            if (gamepad1.left_bumper) {
                speedMultiplier = 0.35;
            }

            // Calculate power for each wheel to drive, strafe, and turn at the same time
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = ((y + x + rx) / denominator) * speedMultiplier;
            double backLeftPower = ((y - x + rx) / denominator) * speedMultiplier;
            double frontRightPower = ((y - x - rx) / denominator) * speedMultiplier;
            double backRightPower = ((y + x - rx) / denominator) * speedMultiplier;

            // Send power to the wheel motors
            fLeft.setPower(frontLeftPower);
            bLeft.setPower(backLeftPower);
            fRight.setPower(frontRightPower);
            bRight.setPower(backRightPower);


            // -----------------------------------------------------------------
            // B. CONTROLLERS 1 & 2: SHOOTER CONTROL & DIRECTION SWITCH
            // -----------------------------------------------------------------

            // Switch/reverse shooter direction (Right Bumper on Gamepad 2 OR Right Stick Button on Gamepad 1)
            boolean currentReverseSwitch = gamepad2.right_bumper || gamepad1.right_stick_button;
            if (currentReverseSwitch && !prevRightBumper2) {
                isShooterReversed = !isShooterReversed;
                if (shooterMotor != null) {
                    shooterMotor.setDirection(isShooterReversed ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
                }
                if (shooterMotor2 != null) {
                    shooterMotor2.setDirection(isShooterReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
                }
            }
            prevRightBumper2 = currentReverseSwitch;

            // Read Right Trigger from either Controller 1 or Controller 2 (0.0 to 1.0)
            double rightTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);

            // Read button inputs from either Controller 1 or Controller 2
            boolean currentY = gamepad1.y || gamepad2.y;
            boolean currentA = gamepad1.a || gamepad2.a;
            boolean currentX = gamepad1.x || gamepad2.x;
            boolean currentB = gamepad1.b || gamepad2.b;
            boolean currentDpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down || gamepad2.dpad_down;

            // RIGHT TRIGGER: Squeeze to manually control shooter power (0% to 100%)
            if (rightTrigger > 0.05) {
                useVelocityControl = false;
                targetShooterPower = rightTrigger;
            } else {
                // BUTTON PRESETS (Y = High, A = Mid, X = Low, B = Stop):
                if (currentY && !prevY) {
                    useVelocityControl = true;
                    targetShooterVelocity = 2000.0;
                    targetShooterPower = 1.0;
                } else if (currentA && !prevA) {
                    useVelocityControl = true;
                    targetShooterVelocity = 1500.0;
                    targetShooterPower = 0.75;
                } else if (currentX && !prevX) {
                    useVelocityControl = true;
                    targetShooterVelocity = 1000.0;
                    targetShooterPower = 0.50;
                } else if (currentB) {
                    useVelocityControl = false;
                    targetShooterPower = 0.0;
                    targetShooterVelocity = 0.0;
                }

                // D-PAD CONTROLS:
                // - D-Pad UP   : Speed UP (+50 ticks/sec or +5% Power)
                // - D-Pad DOWN : Speed DOWN (-50 ticks/sec or -5% Power)
                if (currentDpadUp && !prevDpadUp) {
                    if (useVelocityControl) {
                        targetShooterVelocity += VELOCITY_STEP;
                    } else {
                        targetShooterPower = Math.min(1.0, targetShooterPower + 0.05);
                    }
                }
                if (currentDpadDown && !prevDpadDown) {
                    if (useVelocityControl) {
                        targetShooterVelocity = Math.max(0, targetShooterVelocity - VELOCITY_STEP);
                    } else {
                        targetShooterPower = Math.max(0.0, targetShooterPower - 0.05);
                    }
                }
            }

            // Remember button presses so holding a button doesn't spam the code
            prevY = currentY;
            prevA = currentA;
            prevX = currentX;
            prevDpadUp = currentDpadUp;
            prevDpadDown = currentDpadDown;

            // Turn on shooter motor(s) at the target speed or power
            if (shooterMotor != null) {
                if (useVelocityControl && targetShooterVelocity > 0) {
                    shooterMotor.setVelocity(targetShooterVelocity);
                } else {
                    shooterMotor.setPower(targetShooterPower);
                }
            }

            if (shooterMotor2 != null) {
                if (useVelocityControl && targetShooterVelocity > 0) {
                    shooterMotor2.setVelocity(targetShooterVelocity);
                } else {
                    shooterMotor2.setPower(targetShooterPower);
                }
            }


            // -----------------------------------------------------------------
            // C. CONTROLLERS 1 & 2: INTAKE MOTORS CONTROL
            // -----------------------------------------------------------------

            // Read Left Trigger and Left Stick Button for Intake control
            double leftTrigger2 = gamepad2.left_trigger;
            double intakePower = 0.0;

            if (gamepad1.left_trigger > 0.05 || gamepad2.left_stick_button) {
                // Gamepad 1 LT or Gamepad 2 L3: Reverse / Outtake (-100% Power)
                intakePower = -1.0;
            } else if (leftTrigger2 > 0.05) {
                // Gamepad 2 Left Trigger: Intake IN (Variable power 0% to 100%)
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
            // D. CONTROLLER 2: SERVO 90° CLOCKWISE & RETURN COUNTER-CLOCKWISE
            // -----------------------------------------------------------------

            if (ballservo != null) {
                // LEFT BUMPER (LB): Press to trigger 90° clockwise turn, then auto-return counter-clockwise
                boolean currentLeftBumper2 = gamepad2.left_bumper;
                if (currentLeftBumper2 && !isServoCycling) {
                    isServoCycling = true;
                    servoCycleStartTime = getRuntime();
                    currentServoPosition = servoOutwardPosition;
                    ballservo.setPosition(currentServoPosition); // Turn 90° clockwise
                }

                // Handle timer to automatically turn counter-clockwise back to home position
                if (isServoCycling) {
                    if (getRuntime() - servoCycleStartTime >= SERVO_HOLD_TIME_SEC) {
                        currentServoPosition = SERVO_ORIGINAL_POSITION;
                        ballservo.setPosition(currentServoPosition); // Turn back counter-clockwise
                        isServoCycling = false;
                    }
                }

                // D-PAD LEFT / RIGHT: Fine-tune the 90° clockwise position by +/- 0.05
                boolean currentDpadLeft = gamepad2.dpad_left;
                boolean currentDpadRight = gamepad2.dpad_right;

                if (currentDpadRight && !prevDpadRight) {
                    servoOutwardPosition = Math.min(1.00, servoOutwardPosition + 0.05);
                }
                if (currentDpadLeft && !prevDpadLeft) {
                    servoOutwardPosition = Math.max(0.00, servoOutwardPosition - 0.05);
                }
                prevDpadLeft = currentDpadLeft;
                prevDpadRight = currentDpadRight;
            }


            // -----------------------------------------------------------------
            // E. DISPLAY INFO ON THE DRIVER PHONE SCREEN
            // -----------------------------------------------------------------

            // Get shooter speed from motor sensor
            double currentVelocityTicks = (shooterMotor != null) ? shooterMotor.getVelocity() : ((shooterMotor2 != null) ? shooterMotor2.getVelocity() : 0.0);

            // Calculate shooter RPM (turns per minute)
            double currentRPM = (currentVelocityTicks / TICKS_PER_REV) * 60.0;

            // Show drive info for Controller 1
            telemetry.addData("=== DRIVETRAIN (Gamepad 1) ===", "");
            telemetry.addData("Drive Mode", gamepad1.right_bumper ? "TURBO (100%)" : (gamepad1.left_bumper ? "SLOW (35%)" : "NORMAL (85%)"));
            telemetry.addData("Motor Powers", "FL:%.2f | FR:%.2f | BL:%.2f | BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            // Show shooter info for Controller 1 & 2
            telemetry.addData("\n=== SHOOTER (Gamepad 1 & 2) ===", "");
            if (shooterMotor != null || shooterMotor2 != null) {
                telemetry.addData("Shooter Status", (useVelocityControl && targetShooterVelocity > 0) || targetShooterPower > 0 ? "RUNNING" : "STOPPED");
                telemetry.addData("Shooter Direction", isShooterReversed ? "REVERSED" : "NORMAL");
                if (useVelocityControl) {
                    telemetry.addData("Control Mode", "Closed-Loop Speed Control (Ticks/Sec)");
                    telemetry.addData("Target Velocity", "%.1f ticks/sec", targetShooterVelocity);
                } else {
                    telemetry.addData("Control Mode", "Percent Power Control");
                    telemetry.addData("Target Power", "%.2f (%.0f%%)", targetShooterPower, targetShooterPower * 100);
                }
                telemetry.addData("Actual Speed", "%.1f ticks/sec (%.1f RPM)", currentVelocityTicks, currentRPM);
                if (shooterMotor != null) {
                    telemetry.addData("Shooter 1 Position", "%d ticks", shooterMotor.getCurrentPosition());
                }
                if (shooterMotor2 != null) {
                    telemetry.addData("Shooter 2 Position", "%d ticks", shooterMotor2.getCurrentPosition());
                }
            } else {
                telemetry.addData("CONFIG NOTICE", "No shooter motor configured ('shootermotor'/'shooter1'/'shootermotor2'/'shooter2')");
            }

            // Show intake info for Controller 1 & 2
            telemetry.addData("\n=== INTAKE (Gamepad 2 LT / L3 | Gamepad 1 LT) ===", "");
            if (intake1 != null || intake2 != null) {
                telemetry.addData("Intake Power", "%.2f (%s)", intakePower,
                        intakePower > 0 ? "INTAKE IN" : (intakePower < 0 ? "REVERSE / OUT" : "STOPPED"));
            } else {
                telemetry.addData("CONFIG NOTICE", "Intake motors ('intake1', 'intake2') not configured");
            }

            // Show servo info for Controller 2
            telemetry.addData("\n=== SERVO (Gamepad 2 LB: 90° Cycle | D-Pad L/R: Tune Angle) ===", "");
            if (ballservo != null) {
                telemetry.addData("Servo Status", isServoCycling ? "TURNING (90° Clockwise)" : "READY (At 9 o'clock / 0.0)");
                telemetry.addData("90° Clockwise Angle Target", "%.2f (D-Pad L/R to adjust)", servoOutwardPosition);
                telemetry.addData("Current Position", "%.2f", currentServoPosition);
            } else {
                telemetry.addData("CONFIG ERROR", "Servo 'servo' not configured on Control Hub!");
            }

            // Send info to phone screen
            telemetry.update();

// no neal taco bell i snot the best it is the best
            // =====================================================================
            // 3. STOP ROBOT (When match finishes or user presses STOP)
            // =====================================================================
            fLeft.setPower(0);
            bLeft.setPower(0);
            fRight.setPower(0);
            bRight.setPower(0);
            if (shooterMotor != null) {
                shooterMotor.setPower(0);
            }
            if (shooterMotor2 != null) {
                shooterMotor2.setPower(0);
            }
            if (intake1 != null) {
                intake1.setPower(0);
            }
            if (intake2 != null) {
                intake2.setPower(0);
            }
        }
    }
}
