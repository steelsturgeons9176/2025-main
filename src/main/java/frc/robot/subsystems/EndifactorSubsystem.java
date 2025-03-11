package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import javax.security.auth.login.Configuration;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndifactorConstant;
import frc.robot.Constants.KrakenMotorConstants;

public class EndifactorSubsystem extends SubsystemBase {
    
    private final TalonFX m_endifactor;
    private final TalonFXConfiguration m_endifactorConfig;
 //   private final boolean m_Canrange2;
    private final CANrange m_canRange;
    private final CANrangeConfiguration m_canRangeConfig;
   // private final ControlRequest m_ControlRequest;

    public EndifactorSubsystem(){
        
        m_canRangeConfig = new CANrangeConfiguration();
        m_canRange = new CANrange(KrakenMotorConstants.kCANRangecCanId);
        m_endifactorConfig = new TalonFXConfiguration();
        m_endifactor = new TalonFX(KrakenMotorConstants.KEndifactorCanId);
       // m_fallowNeo = new SparkMax(EndifactorConstant.KFallowNepId, null);
       // m_leadNeo = new SparkMax(EndifactorConstant.KLeadNeoId, null);
     //   m_ControlRequest = new ControlRequest("CANRange");

        m_canRangeConfig.FovParams.FOVRangeX = 6.75;
        m_canRangeConfig.FovParams.FOVRangeY = 6.75;
        m_canRangeConfig.ToFParams.UpdateFrequency = 50;
        m_canRangeConfig.ProximityParams.ProximityThreshold = 0;

        m_endifactorConfig.Voltage.PeakReverseVoltage = -12;
        m_endifactorConfig.Voltage.PeakForwardVoltage = 12;
        m_endifactorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_endifactorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_canRange.getConfigurator().apply(m_canRangeConfig,0.25 );
        
        m_endifactor.getConfigurator().apply(m_endifactorConfig, 0.25);
    }



    @AutoLogOutput(key = "Coral/Stored")
    public boolean hasCoral() {
        return m_canRange.getIsDetected().getValue();
    }

    public void setCoralVoltage(double voltage){
        m_endifactor.setVoltage(voltage);
    }

    public void periodic(){

    }

    
}
