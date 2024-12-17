package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator extends SubsystemBase {
    final ServoEx elevator;
    public Elevator(HardwareMap hardwareMap){
        elevator = hardwareMap.get(ServoEx.class, "elevatorMotor");
    }

    public void elevate(){
        elevator.setPosition(1);
    }

    public void decline(){
        elevator.setPosition(0);
    }

    public void stop(){
        elevator.setPosition(0.5);
    }

    public Command elevateCommand(){
        return new StartEndCommand(
                this::elevate,
                this::stop
        );
    }

    public Command declineCommand(){
        return new StartEndCommand(
                this::decline,
                this::stop
        );
    }
}
