package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor_New", group = "Linear Opmode")
public class Motor_New extends LinearOpMode {


    private DcMotor Motor = null;

    @Override
    public void runOpMode() {

        Motor = hardwareMap.get(DcMotor.class, "leftMotor");


        Motor.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {


            double power = -gamepad1.left_stick_y;
            Motor.setPower(power);
            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }
}
