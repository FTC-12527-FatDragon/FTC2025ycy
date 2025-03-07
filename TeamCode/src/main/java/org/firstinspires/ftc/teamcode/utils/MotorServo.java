package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.subsystems.MotorPIDSlideSubsystem;

@Config
public class MotorServo implements Servo{
    public static double kP = 0.01, kI = 0.0, kD = 0.0002;
    private final PIDController motorPID;
    private final DcMotorEx motor;


    public MotorServo(HardwareMap hardwareMap, String name, PIDController motorPID){
        this.motorPID = motorPID;
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public MotorServo(HardwareMap hardwareMap, String name){
        this(hardwareMap, name, new PIDController(kP, kI, kD));
    }

    public ServoController getController(){
        return (ServoController) motor.getController();
    }

    public int getPortNumber(){
        return motor.getPortNumber();
    }

    public void setDirection(Direction direction){
        if(direction==Direction.FORWARD){
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public Direction getDirection(){
        switch (motor.getDirection()){
            case FORWARD:
                return Direction.FORWARD;
//                break;
            case REVERSE:
                return Direction.REVERSE;
//                break;
            default:
                RobotLog.ee("ServoMotor", "Invalid MotorEx Direction "+motor.getDirection());
        }
        return Direction.FORWARD;
    }

    public void setPower(double x) {
        motor.setPower(x);
    }

    public void setPosition(double position){
        motorPID.setSetPoint(position);
    }

    public double getPosition(){
        return motor.getCurrentPosition();
    }

    public void scaleRange(double lower, double upper){

    }

    public Manufacturer getManufacturer(){
        return motor.getManufacturer();
    }

    public String getDeviceName(){
        return motor.getDeviceName();
    }

    public String getConnectionInfo(){
        return motor.getConnectionInfo();
    }

    public int getVersion(){
        return motor.getVersion();
    }

    public void resetDeviceConfigurationForOpMode(){
        motor.resetDeviceConfigurationForOpMode();
    }

    public void close(){
        motor.close();
    }

    public void update(){
        motor.setPower(motorPID.calculate(motor.getCurrentPosition()));
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}