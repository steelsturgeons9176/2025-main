package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapDoorSubsystem extends SubsystemBase{
    
    private final Servo m_servo;
    private final Servo m_servo2;
    

    public TrapDoorSubsystem(){

        m_servo = new Servo(0);
        m_servo2 = new Servo(1);

        
        
    }





    public void setServoAngle(double Angle){
        m_servo.setAngle(-Angle + 100); //-
        m_servo2.setAngle(-Angle + 70); //+90
        
       // m_servo.setPosition(Angle);
    }


    public void periodic(){

    }

    
}
