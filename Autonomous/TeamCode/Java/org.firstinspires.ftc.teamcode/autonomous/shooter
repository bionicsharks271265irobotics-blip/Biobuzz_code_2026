package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter")
public class Shooter extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        Servo indexer = hardwareMap.get(Servo.class, "indexer");

        waitForStart();

        while (opModeIsActive()) {

            shooter.setPower(gamepad1.a ? 0.85 : 0.0);
            indexer.setPosition(gamepad1.b ? 0.6 : 0.2);

            telemetry.addData("Shooter", shooter.getPower());
            telemetry.update();
        }
    }
}
