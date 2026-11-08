package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
public class Intakemotortest extends LinearOpMode {
    private DcMotor Intake;


    @Override
    public void runOpMode() {


        Intake = hardwareMap.get(DcMotor.class,"Intake");



        Intake.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        if (opModeIsActive()) {
            Intake.setPower(-0.5);



            sleep(2000);


            Intake.setPower(0);

        }
    }
}
