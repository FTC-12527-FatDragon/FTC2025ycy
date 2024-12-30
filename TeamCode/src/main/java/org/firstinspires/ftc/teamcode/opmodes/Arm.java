package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "armTeleOP")
public class Arm extends LinearOpMode {
    private Servo armServo;
    private ArmServoState armState = ArmServoState.ORIGIN;

    @Override
    public void runOpMode() throws InterruptedException {
        armServo = hardwareMap.get(Servo.class,"servo3");
        //slideServo = hardwareMap.get(Servo.class,"servo5");

        waitForStart();
        while(opModeIsActive()){
            double forward = -gamepad1.left_stick_y;
            double fun = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if(gamepad1.a) {
                armState = ArmServoState.SCORE;
            }
            else if(gamepad1.b) {
                armState = ArmServoState.INTAKE;
            }
            else if(gamepad1.dpad_right) {
                armState = ArmServoState.HANG;
            }

            armServo.setPosition(armState.armPosition);
            /*else if(gamepad1.x) {
                slideServo.setPosition(0.5);
            }
            else if(gamepad1.y) {
                slideServo.setPosition(0.65);
            }*/
            telemetry.addData("forward",gamepad1.left_stick_y);
            telemetry.addData("fun:",gamepad1.left_stick_x);
            telemetry.addData("turn:",gamepad1.right_stick_x);
            telemetry.addData("armServo:",armServo.getPosition());
            telemetry.update();
        }
    }

    private enum ArmServoState {
        ORIGIN(0.15),
        INTAKE(0.85),
        SCORE(0.15),
        HANG(0.5);

        public double armPosition;

        ArmServoState(double position) {
            this.armPosition = position;
        }


    }
}
