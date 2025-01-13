package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config @TeleOp(name="servo test++")
public class ServoExTest extends LinearOpMode {
    public static volatile double _0RotDeg = 0;
    public static volatile double _1RotDeg = 180;
    public static volatile double SERVO_POS_DEG = 0;
    public static volatile boolean useDegMode = false;

    private final Telemetry telemetry_M =
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private SimpleServo servoEx;

    private String lastServoName = null;
    private double lastSERVO_POS_DEG = SERVO_POS_DEG;
    private double lastservo_pos = ServoTest.servo_pos;
    private double lastServoRawPos = 0;

    private boolean warnShown = false;

    private void updateServoPos(){
        if(useDegMode){
            servoEx.turnToAngle(SERVO_POS_DEG);
        }else{
            servoEx.setPosition(ServoTest.servo_pos);
        }
        servoEx.setInverted(ServoTest.reverse);
        if(servoEx.getPosition() != lastServoRawPos){
            telemetry_M.addData(ServoTest.servo_name1 + " Send Pos", servoEx.getPosition());
            telemetry_M.addData(ServoTest.servo_name1 + " Rot Deg", servoEx.getAngle());
            telemetry_M.addData(ServoTest.servo_name1 + " Range Deg", servoEx.getAngleRange());
            telemetry_M.update();
            lastServoRawPos = servoEx.getPosition();
        }
    }

    @Override
    public void runOpMode() {
        telemetry_M.clearAll();
        telemetry_M.addLine("本OpMode会让你设置Servo底层0~1值与旋转角度");
        telemetry_M.addLine("基础值请到ServoTest里面Config，其他如度数等值在ServoExTest里设置");
        telemetry_M.addLine("");
        telemetry_M.addLine("SERVO TEST++");
        telemetry_M.update();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if(!ServoTest.servo_name1.equals(lastServoName)){
                try{
                    servoEx = new SimpleServo(hardwareMap, ServoTest.servo_name1, _0RotDeg, _1RotDeg);
                }catch (IllegalArgumentException e){
                    telemetry_M.addLine("无法找到Servo: "+ServoTest.servo_name1);
                    telemetry_M.addLine(e.toString());
                    telemetry_M.update();
                    continue;
                }
                Servo servo = hardwareMap.get(Servo.class, ServoTest.servo_name1);
//                telemetry_M.addLine("已更新控制Servo");
                telemetry_M.clearAll();
                telemetry_M.addLine(ServoTest.servo_name1+" 信息: ");
                telemetry_M.addLine("\tServo端口: "+servo.getPortNumber());
                telemetry_M.addLine("\tServo Position: "+servo.getPosition());
                telemetry_M.addLine("\tServo Direction: "+servo.getDirection());
                telemetry_M.addLine("\tServo PWM: "+servo.getController().getPwmStatus());
                telemetry_M.addLine("\tServo Device: "+servo.getController().getDeviceName());
                telemetry_M.addLine("\tServo Connection: "+servo.getController().getConnectionInfo());
                telemetry_M.update();
                lastServoName = ServoTest.servo_name1;
            }
            if(lastSERVO_POS_DEG != SERVO_POS_DEG){
                servoEx.setRange(_0RotDeg, _1RotDeg);
                useDegMode = true;
                lastSERVO_POS_DEG = SERVO_POS_DEG;
            }else if(lastservo_pos != ServoTest.servo_pos){
                useDegMode = false;
                lastservo_pos = ServoTest.servo_pos;
            }

            updateServoPos();

            if(_0RotDeg > _1RotDeg) {
                if(warnShown == false){
                    telemetry_M.clearAll();
                    telemetry_M.addLine("警告：_0RotDeg必须小于_1RotDeg，否则会出现bug。");
                    telemetry_M.addLine("若要实现反转，请设置ServoTest.reverse并交换_0RotDeg和_1RotDeg的值");
                    telemetry_M.update();
                    warnShown = true;
                }
            }else if(warnShown){
                telemetry_M.clear();
                warnShown = false;
            }
        }
    }
}
