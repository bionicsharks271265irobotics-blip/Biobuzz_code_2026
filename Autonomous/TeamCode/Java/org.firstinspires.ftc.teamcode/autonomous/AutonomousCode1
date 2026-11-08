/*
 * =============================================================================
 * AUTONOMOUS OPMODE: AutonomousCode1
 * =============================================================================
 * PURPOSE:
 * Autonomous routine that runs a test/feeder motor for 10 seconds, followed by
 * a gyro-stabilized sideways strafe to the right for a customizable distance.
 *
 * EXECUTION SEQUENCE:
 * 1. Step 1: Runs the test/feeder motor ("motor") at 100% power for 10 seconds.
 * 2. Step 2: Strafes right for a configured distance (default 24 inches) using
 *    IMU gyro heading correction to maintain orientation and eliminate drift.
 *
 * HARDWARE CONFIGURATION:
 * - Drivetrain Motors: "fLeft", "bLeft", "fRight", "bRight" (Mecanum)
 * - Auxiliary Motor: "motor" (Feeder / Intake test motor)
 * - IMU: Control Hub Internal Gyroscope ("imu")
 * =============================================================================
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Autonomous OpMode: Runs the motor for 10 seconds, then strafes EXACTLY sideways 
 * to the right for a customizable distance using IMU gyro heading correction.
 * 
 * <p><strong>Autonomous Execution Sequence:</strong></p>
 * <ol>
 *   <li><strong>Step 1 (Motor Phase):</strong> Runs the test/feeder motor ({@code "motor"}) 
 *       for 10.0 seconds with telemetry feedback.</li>
 *   <li><strong>Step 2 (Gyro-Stabilized Strafe Phase):</strong> Drives the robot EXACTLY sideways 
 *       to the right for a configurable distance in inches (default: 24.0 inches) using 
 *       closed-loop IMU gyro heading correction to eliminate rotation and drift.</li>
 * </ol>
 * 
 * <p><strong>Hardware Mapping:</strong></p>
 * <ul>
 *   <li>{@code "fLeft"}: Front Left Drive Motor (DcMotor, REVERSE direction, BRAKE zero power behavior)</li>
 *   <li>{@code "bLeft"}: Back Left Drive Motor (DcMotor, REVERSE direction, BRAKE zero power behavior)</li>
 *   <li>{@code "fRight"}: Front Right Drive Motor (DcMotor, FORWARD direction, BRAKE zero power behavior)</li>
 *   <li>{@code "bRight"}: Back Right Drive Motor (DcMotor, FORWARD direction, BRAKE zero power behavior)</li>
 *   <li>{@code "motor"}: Test / Feeder motor (DcMotorEx, REVERSE direction)</li>
 *   <li>{@code "imu"}: Control Hub Internal IMU (Gyroscope for heading stabilization)</li>
 * </ul>
 */
@Autonomous(name = "Autonomous Code 1", group = "Autonomous")
public class AutonomousCode1 extends LinearOpMode {

    // =========================================================================
    // CHANGEABLE AUTONOMOUS CONFIGURATION PARAMETERS
    // =========================================================================
    /** Total duration to run the motor during Step 1 (in seconds). */
    public static double MOTOR_DURATION_SECONDS = 10.0;

    /**
     * Changeable distance to strafe to the right (in inches).
     * <p>Modify this parameter to adjust how far the robot strafes right during Step 2.</p>
     */
    public static double STRAFE_DISTANCE_INCHES = 24.0;

    /** Motor power magnitude used for the strafing movement (0.0 to 1.0). */
    public static double STRAFE_POWER = 0.5;

    /** Proportional gain constant (Kp) for active gyro heading correction during strafe. */
    public static double HEADING_P_GAIN = 0.03;

    // =========================================================================
    // HARDWARE & ENCODER KINEMATIC CONSTANTS (GoBILDA 5202/5203 Series 19.2:1 Ratio)
    // =========================================================================
    /** Encoder ticks per full motor shaft revolution. */
    public static final double TICKS_PER_REV = 537.7;

    /** Diameter of standard Mecanum wheels in inches (96mm = 3.77953 in). */
    public static final double WHEEL_DIAMETER_INCHES = 3.77953;

    /** Empirical slip multiplier to compensate for Mecanum sideways wheel friction/slippage. */
    public static final double STRAFE_SLIP_FACTOR = 1.1;

    /** Calculated encoder counts per linear inch of strafe motion. */
    public static final double COUNTS_PER_INCH = (TICKS_PER_REV * STRAFE_SLIP_FACTOR) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // =========================================================================
    // HARDWARE DECLARATIONS
    // =========================================================================
    // 4-Wheel Mecanum Drivetrain Motors
    private DcMotor fLeft;
    private DcMotor bLeft;
    private DcMotor fRight;
    private DcMotor bRight;

    // Test / Feeder Motor
    private DcMotorEx motor;

    // Control Hub Built-In IMU Gyroscope
    private IMU imu;

    @Override
    public void runOpMode() {
        // High-frequency telemetry updates (20ms interval)
        telemetry.setMsTransmissionInterval(20);

        // =====================================================================
        // 1. HARDWARE INITIALIZATION
        // =====================================================================
        // Initialize Drivetrain Motors
        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bRight = hardwareMap.get(DcMotor.class, "bRight");

        // Set Drivetrain Motor Directions (left motors reversed, right motors forward)
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        // Enable BRAKE behavior to hold position firmly when stopped
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and configure drivetrain motor encoders
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Control Hub Internal IMU Gyroscope
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            // Default Orientation: Logo UP, USB FORWARD. Adjust if Control Hub is mounted differently.
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
        } catch (Exception e) {
            imu = null; // Marked null if IMU hardware lookup fails
        }

        // Initialize test / feeder motor (from Motor_ControlTest.java)
        try {
            motor = hardwareMap.get(DcMotorEx.class, "motor");
            motor.setDirection(DcMotor.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            motor = null; // Marked null if not connected in current hardware configuration
        }

        // Display ready status and configured parameters
        telemetry.addData("Status", "Initialized - Ready for Start");
        telemetry.addData("Strafe Distance Setting", "%.1f inches", STRAFE_DISTANCE_INCHES);
        if (imu != null) {
            telemetry.addData("IMU Status", "Initialized (Heading: %.2f°)", getHeading());
        } else {
            telemetry.addData("IMU Status", "Not Detected (Using Open-Loop Strafe)");
        }
        telemetry.update();

        // =====================================================================
        // 2. WAIT FOR START COMMAND
        // =====================================================================
        waitForStart();
        resetRuntime();

        // =====================================================================
        // 3. STEP 1: RUN MOTOR FOR 10 SECONDS
        // =====================================================================
        double motorStartTime = getRuntime();
        while (opModeIsActive() && (getRuntime() - motorStartTime < MOTOR_DURATION_SECONDS)) {
            // Drive test / feeder motor if present
            if (motor != null) {
                motor.setPower(1.0);
            }

            double timeRemaining = MOTOR_DURATION_SECONDS - (getRuntime() - motorStartTime);

            telemetry.addData("Status", "Step 1/2: Motor active");
            telemetry.addData("Time Remaining", "%.1f s", timeRemaining);
            if (motor != null) {
                double ticksPerSecond = motor.getVelocity();
                double ticksPerMinute = ticksPerSecond * 60.0;
                telemetry.addData("Motor Velocity (Ticks/Min)", "%.1f", ticksPerMinute);
                telemetry.addData("Motor Position Ticks", "%d", motor.getCurrentPosition());
            }
            telemetry.update();
        }

        // Stop motor immediately after 10-second duration expires
        if (motor != null) {
            motor.setPower(0);
        }

        telemetry.addData("Status", "Step 1 Complete: Motor Stopped. Preparing to strafe...");
        telemetry.update();

        // Brief stabilization pause
        sleep(250);

        // =====================================================================
        // 4. STEP 2: STRAFE RIGHT EXACTLY SIDEWAYS (GYRO STABILIZED)
        // =====================================================================
        if (opModeIsActive()) {
            strafeRightStraight(STRAFE_DISTANCE_INCHES, STRAFE_POWER);
        }

        // Autonomous routine complete
        telemetry.addData("Status", "Autonomous Routine Finished");
        telemetry.update();
    }

    /**
     * Strafes the robot EXACTLY sideways to the right by the specified distance in inches.
     * Uses IMU (Gyroscope) closed-loop heading correction to actively counteract drift and rotation.
     *
     * @param distanceInInches Distance to strafe to the right in inches.
     * @param power            Base motor power magnitude (0.0 to 1.0).
     */
    public void strafeRightStraight(double distanceInInches, double power) {
        int moveTicks = (int) (distanceInInches * COUNTS_PER_INCH);

        // Reset encoders before starting strafe calculation
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Lock in target heading at the beginning of the strafe
        double targetHeading = getHeading();

        while (opModeIsActive()) {
            // Read absolute displacement ticks on all 4 wheels
            int currentFL = Math.abs(fLeft.getCurrentPosition());
            int currentBL = Math.abs(bLeft.getCurrentPosition());
            int currentFR = Math.abs(fRight.getCurrentPosition());
            int currentBR = Math.abs(bRight.getCurrentPosition());

            // Average absolute encoder position
            int avgTicks = (currentFL + currentBL + currentFR + currentBR) / 4;
            if (avgTicks >= moveTicks) {
                break; // Target distance reached!
            }

            // Read current IMU heading and calculate correction
            double currentHeading = getHeading();
            double headingError = targetHeading - currentHeading;

            // Normalize error between -180 and +180 degrees
            while (headingError > 180)  headingError -= 360;
            while (headingError < -180) headingError += 360;

            // Calculate active turn correction
            double turn = headingError * HEADING_P_GAIN;

            // Mecanum Strafe Right Vector (+FL, -BL, -FR, +BR) + Gyro Heading Turn Correction
            double flPwr = power + turn;
            double blPwr = -power + turn;
            double frPwr = -power - turn;
            double brPwr = power - turn;

            // Normalize motor powers if any individual power exceeds 1.0
            double maxPwr = Math.max(Math.max(Math.abs(flPwr), Math.abs(blPwr)),
                                     Math.max(Math.abs(frPwr), Math.abs(brPwr)));
            if (maxPwr > 1.0) {
                flPwr /= maxPwr;
                blPwr /= maxPwr;
                frPwr /= maxPwr;
                brPwr /= maxPwr;
            }

            // Apply drive powers
            fLeft.setPower(flPwr);
            bLeft.setPower(blPwr);
            fRight.setPower(frPwr);
            bRight.setPower(brPwr);

            // Display live straight-strafe telemetry
            telemetry.addData("Status", "Step 2/2: Strafing Exactly Sideways (Gyro Stabilized)");
            telemetry.addData("Distance Progress", "%d / %d ticks (%.1f in)", avgTicks, moveTicks, distanceInInches);
            telemetry.addData("Heading (Target : Current)", "%.2f° : %.2f°", targetHeading, currentHeading);
            telemetry.addData("Heading Error", "%.2f°", headingError);
            telemetry.addData("Correction Turn Power", "%.3f", turn);
            telemetry.update();
        }

        // Stop all drive motors
        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
    }

    /**
     * Reads the robot heading directly from the Control Hub IMU (in degrees).
     *
     * @return Current robot yaw heading in degrees (-180.0 to +180.0).
     */
    public double getHeading() {
        if (imu != null) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);
        }
        return 0.0;
    }
}
