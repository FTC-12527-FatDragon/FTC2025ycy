package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoUtils {
  public static Command setServoPosCommand(Servo servo, double pos, long delay) {
    return new ConditionalCommand(
        new InstantCommand(
                () -> {
                  servo.setPosition(pos);
                })
            .andThen(new WaitCommand(delay)),
        new InstantCommand(() -> {}),
        () -> servo.getPosition() != pos);
  }
}
