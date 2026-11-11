package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "shoot_park")
public class shoot_park extends LinearOpMode {

    private DcMotor fLeft, bLeft, fRight, bRight,  shooterMotor , storage1, storage2;


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {

        fLeft  = hardwareMap.get(DcMotor.class, "fLeft");
        bLeft  = hardwareMap.get(DcMotor.class, "bLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bRight = hardwareMap.get(DcMotor.class, "bRight");
        shooterMotor =  hardwareMap.get(DcMotor.class, "shooter");
        storage1 =  hardwareMap.get(DcMotor.class, "storage1");
        storage2 =  hardwareMap.get(DcMotor.class, "storage2");



        fLeft.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        storage1.setDirection(DcMotor.Direction.REVERSE);
        storage2.setDirection(DcMotor.Direction.REVERSE);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        storage1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        storage2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        storage1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        storage2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();





        sleep(5000);
        // shoot pollen 4x
        encoderShooter(8, 50, 5);
        // move backward
        encoderDrive(1,  -24, 6);
        // turn left
        encoderDriveTurn(1,28,4,true);
        // move forward
        encoderDrive(1, 48, 6);



    }

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

    public void encoderShooter(double speed, double inches, double timeouts) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);
        shooterMotor.setTargetPosition(shooterMotor.getCurrentPosition() + moveCounts);
        storage1.setTargetPosition(storage1.getCurrentPosition() + moveCounts);
        storage2.setTargetPosition(storage2.getCurrentPosition() + moveCounts);

        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        storage1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        storage2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        shooterMotor.setPower(Math.abs(speed));
        storage1.setPower(Math.abs(speed));
        storage2.setPower(Math.abs(speed));


        while (opModeIsActive() && shooterMotor.isBusy() && storage1.isBusy() && storage2.isBusy()) {
            telemetry.addData("spinning", "Running");
            telemetry.update();

        }
        stopMotors();
        resetEncoders();
    }

    public void encoderDriveTurn(double speed, double inches, double timeoutS, boolean right) {

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

    private void stopMotors() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
        shooterMotor.setPower(0);
        storage1.setPower(0);
        storage2.setPower(0);
    }

    private void resetEncoders() {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        storage1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        storage2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }
}
