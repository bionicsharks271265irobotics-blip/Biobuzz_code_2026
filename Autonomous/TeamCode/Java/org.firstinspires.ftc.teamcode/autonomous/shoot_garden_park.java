package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "shoot_garden_park")
public class shoot_garden_park extends LinearOpMode {

    private DcMotor fLeft, bLeft, fRight, bRight, intake;
    private DcMotorEx shooterMotor, storage1, storage2;


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
        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        storage1 =  hardwareMap.get(DcMotorEx.class, "storage1");
        storage2 =  hardwareMap.get(DcMotorEx.class, "storage2");





        fLeft.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        storage1.setDirection(DcMotorEx.Direction.REVERSE);
        storage2.setDirection(DcMotorEx.Direction.REVERSE);


        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        storage1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        storage2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        storage1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        storage2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);




        waitForStart();




        //Shoot 4 balls code from start pos

        encoderShooter(3000, 200, -100, 5);
        // drive forward
        encoderDrive(1,9,5);
        // turn right
        encoderDriveTurn(1,18,5,false);
        // drive forward
        encoderDrive(1, -44,5);
        // turn left
        encoderDriveTurn(1,18,5,true);
        //drive forward with intake
        encoderMotor(0.5, -19, 50, 5);
        //drive backward
        encoderDrive(1, 10, 5);
        //turn right
        encoderDriveTurn(1,24   ,5,false);
        // drive backward
        encoderDrive(1,44,5);
        // turn left
        encoderDriveTurn(1,25,5,true);
        // drive forward
        encoderDrive(1, -19, 5);
        //shoot for 5 seconds
        sleep(5000);
        encoderShooter(3000, 200, -100, 5);
        // drive backward
        encoderDrive(1,10,5);
        //turn right
        encoderDriveTurn(1,24,5,false);
        //drive forward
        encoderDrive(1,-30,5);
        //turn right
        encoderDriveTurn(1, 26, 5, false);
        // drive forward
        encoderDrive(1, -85, 5);
        // turn left
        encoderDriveTurn(1, 29, 5, true);
        //drive forward
        encoderDrive(1, -17, 5);
    }
    public void encoderShooter(double velocity, double velocity2 ,double inches, double timeouts){
        int moveCounts = (int) (inches * COUNTS_PER_INCH);
        int leftTarget = (int)(inches * COUNTS_PER_INCH);
        int rightTarget = (int)(inches * COUNTS_PER_INCH);


        shooterMotor.setTargetPosition(leftTarget);
        storage1.setTargetPosition(rightTarget);
        storage2.setTargetPosition(rightTarget);


        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        storage1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        storage2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shooterMotor.setVelocity(velocity);
        storage1.setVelocity(velocity2);
        storage2.setVelocity(velocity2);




        while (opModeIsActive() && shooterMotor.isBusy() && storage1.isBusy() && storage2.isBusy()) {
            telemetry.addData("spinning", "Running");
            telemetry.update();

        }
        stopMotors();
        resetEncoders();
    }




    public void encoderMotor(double speed, double inches, double other_inches, double timeouts){
        int moveCounts = (int) (inches * COUNTS_PER_INCH);
        int moveMotor = (int) (other_inches * COUNTS_PER_INCH);

        intake.setTargetPosition(intake.getCurrentPosition() + moveMotor);
        fLeft.setTargetPosition(fLeft.getCurrentPosition() + moveCounts);
        fRight.setTargetPosition(fRight.getCurrentPosition() + moveCounts);
        bLeft.setTargetPosition(bLeft.getCurrentPosition() + moveCounts);
        bRight.setTargetPosition(bRight.getCurrentPosition() + moveCounts);

        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        intake.setPower(Math.abs(speed));
        fLeft.setPower(Math.abs(speed));
        fRight.setPower(Math.abs(speed));
        bLeft.setPower(Math.abs(speed));
        bRight.setPower(Math.abs(speed));

        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && intake.isBusy() && fLeft.isBusy() && fRight.isBusy() && bLeft.isBusy() && bRight.isBusy()) {
            telemetry.addData("intake", "Running");
            telemetry.update();
        }
        stopMotors();
        resetEncoders();



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
        intake.setPower(0);

        shooterMotor.setVelocity(0);
        storage1.setVelocity(0);
        storage2.setVelocity(0);


    }

    private void resetEncoders() {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        storage1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        storage2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }
}

