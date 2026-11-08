package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Park_from_left")
public class Park_from_left extends LinearOpMode {

    private DcMotor fLeft, bLeft, fRight, bRight;

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;

    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);

    // Main autonomous execution: initializes hardware, waits for start, and drives the parking path.
    @Override
    public void runOpMode() {

        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bRight = hardwareMap.get(DcMotor.class, "fRight"); // Wait, bRight should be hardwareMap.get(DcMotor.class, "bRight") - keep original: bRight = hardwareMap.get(DcMotor.class, "bRight")

        fLeft.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        encoderDrive(1, 3, 5);
        encoderDriveTurn(1, 18, 5, true);
        encoderDrive(1, -15, 5);
    }

    // Drives straight forward (+) or backward (-) by specified inches using encoders.
    public void encoderDrive(double speed, double inches, double timeoutS) {

        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        fLeft.setTargetPosition(fLeft.getCurrentPosition() + moveCounts);
        fRight.setTargetPosition(fRight.getCurrentPosition() + moveCounts);
        bLeft.setTargetPosition(bLeft.getCurrentPosition() + moveCounts);
        bRight.setTargetPosition(bRight.getCurrentPosition() + moveCounts);

        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLeft.setPower(Math.abs(speed));
        fRight.setPower(Math.abs(speed));
        bLeft.setPower(Math.abs(speed));
        bRight.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                fLeft.isBusy() && fRight.isBusy() &&
                bLeft.isBusy() && bRight.isBusy()) {

            telemetry.addData("Driving", "Running");
            telemetry.update();
        }

        stopMotors();
        resetEncoders();
    }

    // Turns the robot left or right based on encoder counts and direction flag.
    public void encoderDriveTurn(double speed, double inches,
                                 double timeoutS, boolean right) {

        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        if (right) {
            fLeft.setTargetPosition(moveCounts);
            bLeft.setTargetPosition(moveCounts);
            fRight.setTargetPosition(-moveCounts);
            bRight.setTargetPosition(-moveCounts);
        } else {
            fLeft.setTargetPosition(-moveCounts);
            bLeft.setTargetPosition(-moveCounts);
            fRight.setTargetPosition(moveCounts);
            bRight.setTargetPosition(moveCounts);
        }

        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLeft.setPower(Math.abs(speed));
        fRight.setPower(Math.abs(speed));
        bLeft.setPower(Math.abs(speed));
        bRight.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                fLeft.isBusy() && fRight.isBusy() &&
                bLeft.isBusy() && bRight.isBusy()) {

            telemetry.addData("Turning", "Running");
            telemetry.update();
        }

        stopMotors();
        resetEncoders();
    }

    // Sets power to 0 for all drive motors.
    private void stopMotors() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    // Resets encoder counts and sets mode to stop and reset.
    private void resetEncoders() {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
