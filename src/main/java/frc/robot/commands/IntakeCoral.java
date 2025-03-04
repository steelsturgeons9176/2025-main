package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;


public class IntakeCoral extends Command{
    private Intake m_intake;
    private Intake.wristPositions m_tarPosition;


    public IntakeCoral(Intake intake){
        m_intake = intake;
        m_tarPosition = Intake.wristPositions.SOURCE_ANGLE;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_intake.setCoralIntakeVoltage(4);
    }

    @Override
    public void execute(){
        m_intake.wristAngle(m_tarPosition);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        m_intake.setCoralIntakeVoltage(1);
        m_intake.wristAngle(Intake.wristPositions.HOLD_ANGLE);
    }
}
