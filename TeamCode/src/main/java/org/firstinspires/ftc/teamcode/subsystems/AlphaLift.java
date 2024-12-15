package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class AlphaLift extends SubsystemBase {
  private final double kP = 0.005, kI = 0.0, kD = 0.0, kG = 0.0;
  private final PIDController pidController;
  private final DcMotorEx liftMotor;
  private double setpointTicks = 0.0;
  private MultipleTelemetry telemetry;

  @Getter @Setter private Goal goal = Goal.STOW;

  public AlphaLift(final HardwareMap hardwareMap, Telemetry telemetry) {
    liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    pidController = new PIDController(kP, kI, kD);

    this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
  }

  public void runSetpoint(double ticks) {
    setpointTicks = Range.clip(ticks, 0.0, 1550.0);
  }

  public void runLiftOpen(double percent) {
    goal = Goal.OPEN_LOOP;
    liftMotor.setPower(Range.clip(percent, -1, 1));
  }

  public Command resetCommand() {
    return new StartEndCommand(
        () -> {
          runLiftOpen(-0.6);
        },
        () -> {
          pidController.reset();
          pidController.calculate(0);
          runLiftOpen(0);
          liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          goal = Goal.STOW;
          telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
          telemetry.addData("Error", pidController.getPositionError());
          telemetry.update();
        },
        this);
  }

  public double getCurrentPosition() {
    return liftMotor.getCurrentPosition();
  }

  public boolean atGoal() {
    return MathUtils.isNear(goal.setpointTicks, liftMotor.getCurrentPosition(), 5);
  }

  public boolean atHome() {
    return MathUtils.isNear(Goal.STOW.setpointTicks, liftMotor.getCurrentPosition(), 5);
  }

  public boolean atPreHang() {
    return MathUtils.isNear(Goal.PRE_HANG.setpointTicks, liftMotor.getCurrentPosition(), 5);
  }

  @Override
  public void periodic() {
    //
    //        telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
    //        telemetry.addData("Error", pidController.getPositionError());
    //        telemetry.update();
    if (goal == Goal.OPEN_LOOP) return;

    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    setpointTicks = goal.setpointTicks;
    double pidPower = pidController.calculate(liftMotor.getCurrentPosition(), setpointTicks);
    liftMotor.setPower(Range.clip(pidPower + kG, -1, 1));

    telemetry.addData("Lift Current Power", liftMotor.getPower());
    telemetry.addData("Lift Goal", goal);
    telemetry.addData("Lift At Home", atHome());
    telemetry.addData("controller", pidController.atSetPoint());
    telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
    telemetry.addData("Error", pidController.getPositionError());

    telemetry.addData("Current", liftMotor.getCurrent(CurrentUnit.AMPS));
    telemetry.update();
  }

  public enum Goal {
    BASKET(1550.0),
    STOW(10.0),
    PRE_HANG(700.0),
    HANG(1180.0),
    GRAB(0.0),
    OPEN_LOOP(0.0);

    private final double setpointTicks;

    Goal(double setpointTicks) {
      this.setpointTicks = setpointTicks;
    }
  }
}

// package org.firstinspires.ftc.teamcode.subsystems;
//
// import com.acmerobotics.dashboard.FtcDashboard;
// import com.acmerobotics.dashboard.config.Config;
// import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
// import com.arcrobotics.ftclib.command.SubsystemBase;
// import com.arcrobotics.ftclib.controller.PIDFController;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.HardwareMap;
//
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.firstinspires.ftc.teamcode.utils.PIDMotionProfileController;
//
// import java.util.Set;
//
// @Config
// public class AlphaLift extends SubsystemBase {
//    public static boolean UsePID = true;
//    public static double TicksPerRev = 567;//((((1+(46/17))) * (1+(46/17))) * 28);
//    public static double SpoolInnerRadiusMM = 36.25000 / 2;
//    public static double SetpointMM = -1;
//    public static PIDMotionProfileController liftPID = new PIDMotionProfileController(0.0075, 0,
// 0, 0.015, 0, 0, 0.05);
//    private DcMotor liftMotor;
//    private final HardwareMap hardwareMap;
//    private final Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
//    public AlphaLift(HardwareMap hm){
//        // TODO: Check PID works or not & which motor has encoder
//        hardwareMap = hm;
//        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
//        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        if (UsePID) liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftPID.setTolerance(20, 10);
//    }
//
//    public void setOpenLoop(double vol) {
//        liftMotor.setPower(vol);
//    }
//
//    @Override
//    public void periodic() {
//        liftPID.TicksPerRev = TicksPerRev;
//        if (SetpointMM != -1) setSetPoint(SetpointMM);
//        if (UsePID) {
//            telemetry.addData("setpoint", liftPID.getSetPoint());
//            telemetry.addData("liftMotorPos", liftMotor.getCurrentPosition());
//
//            double power = liftPID.calculate(liftMotor.getCurrentPosition());
//            telemetry.addData("liftMotorPower", power);
//            liftMotor.setPower(power);
//        }
//        telemetry.update();
//    }
//
//    public void setSetPointByMotorRot(double rev) {
//        liftPID.setSetPoint(rev * TicksPerRev);
//    }
//
//    public double getPositionByMotorRot() {
//        return liftMotor.getCurrentPosition() / TicksPerRev;
//    }
//
//    public void setSetPoint(double posmm) {
//        setSetPointByMotorRot(posmm / (2 * Math.PI * SpoolInnerRadiusMM));
//    }
//
//    public double getPositionMM() {
//        return getPositionByMotorRot() * (2 * Math.PI * SpoolInnerRadiusMM);
//    }
//
//    public boolean atSetPoint() {
//        return liftPID.atSetPoint();
//    }
// }
