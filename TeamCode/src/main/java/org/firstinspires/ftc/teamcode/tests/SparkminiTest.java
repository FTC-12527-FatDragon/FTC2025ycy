package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "SparkMini Test")
public class SparkminiTest extends LinearOpMode {

    public static double lastPower = 0;
    DcMotorEx climber;

    @Override
    public void runOpMode() throws InterruptedException {

        climber = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                climber.setPower(1);
            }
            else if(gamepad1.b) {
                climber.setPower(-1);
            }
            else {
                climber.setPower(0);
            }
        }

        while (isStopRequested()) {
            climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            climber.setPower(1);
            telemetry.addData("Hello", "world");
            telemetry.update();
        }

    }

}
