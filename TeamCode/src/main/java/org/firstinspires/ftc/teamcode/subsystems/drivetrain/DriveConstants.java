package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.lib.Units;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;
import org.firstinspires.ftc.teamcode.utils.Translation2dHelperClass;

import lombok.Getter;
import lombok.Setter;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {
  public static String android_id = FtcRobotControllerActivity.getAndroidId();

  @Setter @Getter public static volatile Pose2dHelperClass robotTeleOpStartPose = new Pose2dHelperClass();


  public static final RobotType currentRobot = RobotType.EPSILON;

  public enum RobotType {
    ALPHA,
    BETA,
    GAMMA,
    DELTA,
    EPSILON
  }

  @Deprecated
  public static final Pose2d xPose =
      currentRobot == RobotType.ALPHA
          ? new Pose2d(Units.mmToInches(-25), Units.mmToInches(155), Units.degreesToRadians(0))
          : new Pose2d(Units.mmToInches(0), Units.mmToInches(-100), Units.degreesToRadians(0));

  @Deprecated
  public static final Pose2d yPose =
      currentRobot == RobotType.ALPHA
          ? new Pose2d(Units.mmToInches(-25), Units.mmToInches(-155), Units.degreesToRadians(90))
          : new Pose2d(Units.mmToInches(40), Units.mmToInches(15), Units.degreesToRadians(90));

  /*
   * These are motor constants that should be listed online for your motors.
   */
  public static final double TICKS_PER_REV =
      537.7; // From https://learnroadrunner.com/drive-constants.html#ticks-per-rev-max-rpm ->
  // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
  public static final double MAX_RPM = 312; // From
  // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/

  // DO NOT TOUCH THESE VALUES ref.
  // https://learnroadrunner.com/drive-constants.html#run-using-encoder-motor-velo-pid
  public static final boolean RUN_USING_ENCODER = false;
  public static PIDFCoefficients MOTOR_VELO_PID =
      new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

  /*
   * These are physical constants that can be determined from your robot (including the track
   * width; it will be tune empirically later although a rough estimate is important). Users are
   * free to chose whichever linear distance unit they would like so long as it is consistently
   * used. The default values were selected with inches in mind. Road runner uses radians for
   * angular distances although most angular parameters are wrapped in Math.toRadians() for
   * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
   */
  public static double WHEEL_RADIUS = currentRobot == RobotType.ALPHA?2:2.05; // INCH!!
  public static double GEAR_RATIO =
      1; // output (wheel) speed / input (motor) speed, >1 -> 加速 <1 -> 减速
  public static double TRACK_WIDTH; // INCH!!

  /*
   * These are the feedforward parameters used to model the drive motor behavior. If you are using
   * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
   * motor encoders or have elected not to use them for velocity control, these values should be
   * empirically tuned.
   */
  public static double kV; // 1.0 / rpmToVelocity(MAX_RPM);
  public static double kA;
  public static double kStatic;

  /*
   * These values are used to generate the trajectories for you robot. To ensure proper operation,
   * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
   * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
   * small and gradually increase them later after everything is working. All distance units are
   * inches.
   */
  public static double MAX_VEL;
  public static double MAX_ACCEL = 60;
  public static double MAX_ANG_VEL;
  public static double MAX_ANG_ACCEL = Math.toRadians(180);

  /*
   * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
   */
  public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
      currentRobot == RobotType.GAMMA
          ? RevHubOrientationOnRobot.LogoFacingDirection.DOWN
          : RevHubOrientationOnRobot.LogoFacingDirection.UP;
  public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
      currentRobot == RobotType.GAMMA
          ? RevHubOrientationOnRobot.UsbFacingDirection.LEFT
          : RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

  public static GoBildaPinpointDriver.EncoderDirection GoBildaXLocalizerDirection;
  public static GoBildaPinpointDriver.EncoderDirection GoBildaYLocalizerDirection;
  public static GoBildaPinpointDriver.GoBildaOdometryPods GoBildaLocalizerEncoderResolution;
  public static Translation2dHelperClass GoBildaLocalizerPerpendicularOffset;
  public static boolean isReflected = false;

  static {
    switch (currentRobot) {
      case ALPHA:
        isReflected = false;
        GoBildaXLocalizerDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        GoBildaYLocalizerDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        GoBildaLocalizerEncoderResolution =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        GoBildaLocalizerPerpendicularOffset = new Translation2dHelperClass(97, 108);
        TRACK_WIDTH = 10.26;
        kV = 0.015;
        kA = 0.0022;
        kStatic = 0.05;
        MAX_VEL = 62.89457585940184;
        MAX_ANG_VEL = Math.toRadians(134.16);
        break;
      case EPSILON:
        GoBildaXLocalizerDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        GoBildaYLocalizerDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        GoBildaLocalizerEncoderResolution =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        GoBildaLocalizerPerpendicularOffset = new Translation2dHelperClass(-97, -108);
        TRACK_WIDTH = 13.97;
        kV = 0.011;
        kA = 0.0026;
        kStatic = 0.043;
        MAX_VEL = 85.65532110468519;
        MAX_ANG_VEL = 2.43857479095459;
        isReflected = false;
        break;
      case BETA:
        GoBildaXLocalizerDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        GoBildaYLocalizerDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        GoBildaLocalizerEncoderResolution =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        GoBildaLocalizerPerpendicularOffset = new Translation2dHelperClass(-92.03742, 104.03742);
        break;
      case GAMMA:
        GoBildaXLocalizerDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        GoBildaYLocalizerDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        GoBildaLocalizerEncoderResolution =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        GoBildaLocalizerPerpendicularOffset = new Translation2dHelperClass(0, 92.5);
        TRACK_WIDTH = 16.19;
        kV = 0.012;
        kA = 0.003;
        kStatic = 0.065;
        MAX_VEL = 51.064485597344834;
        MAX_ANG_VEL = Math.toRadians(143.51011005498376);
        isReflected = true;
        break;
      default:
        RobotLog.ee(
            "DriveConstants",
            "GoBildaPinpointDriver not configured for %s.",
            currentRobot.toString());
    }
  }

  public static double encoderTicksToInches(double ticks) {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }

  public static double rpmToVelocity(double rpm) {
    return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
  }

  public static double getMotorVelocityF(double ticksPerSecond) {
    // see
    // https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
    return 32767 / ticksPerSecond;
  }
}
