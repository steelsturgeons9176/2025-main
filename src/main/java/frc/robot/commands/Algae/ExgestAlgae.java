package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class ExgestAlgae extends Command{
  private Intake m_intake;


  public ExgestAlgae(Intake intake){
      m_intake = intake;
      addRequirements(m_intake);
  }

  @Override
  public void initialize(){
      m_intake.setAlgaeVoltage(-4);
  }

  @Override
  public void execute(){

  }


  @Override
  public boolean isFinished(){
      return false;
  }

  @Override
  public void end(boolean isInterrupted){
      m_intake.setAlgaeVoltage(0);
  }
}
