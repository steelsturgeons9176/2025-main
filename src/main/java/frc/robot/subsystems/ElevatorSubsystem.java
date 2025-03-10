package frc.robot.subsystems;

import java.util.EnumMap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
    public static final double reduction = 20.0/5.5;
    public enum elevatorPositions{
        L1_HEIGHT,
        L2_HEIGHT,
        L3_HEIGHT,
        L4_HEIGHT,
        SOURCE_HEIGHT
    }

    private final TalonFX m_elevatorLead;
    private final TalonFX m_elevatorFollow;

    //private final PositionTorqueCurrentFOC torqueCurrentRequest = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0).withUpdateFreqHz(0);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    private double pos;

    private TalonFXConfiguration m_leadConfig;

    private double feedforward;

 //   private final RelativeEncoder ElevatorEncoder;


    //private final PIDController m_AbsPidController = new PIDController(1, 0.0, 0.0);



    private double currentGoal = 0.125f;

    DoubleSubscriber currentGoalSub;


    private Timer m_timer;

    private double kS_tuner = 0;

    private double kP = 10; //10
    private double kI = 0;
    private double kD = 0.0;



    private double kG = .2;
    private double kS = .3;
    private double kV = 0;
    private double kA = 0;

    private double p;
    private double i;
    private double d;

    private boolean isTuning = false;

    private ElevatorFeedforward ff =
    new ElevatorFeedforward(kS, kG ,kV, kA);


    private final TrapezoidProfile profile;

    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    EnumMap<elevatorPositions, Double> mapAbs = new EnumMap<>(elevatorPositions.class);

    double m_speed = 0.0;

    //DoublePublisher kG_gain = NetworkTableInstance.getDefault()
    //.getDoubleTopic("MyStates", ).publish();

    public ElevatorSubsystem () {

        m_leadConfig = new TalonFXConfiguration();
        m_timer = new Timer();
        m_timer.start();
        m_timer.reset();

        mapAbs.put(elevatorPositions.L1_HEIGHT, ElevatorConstants.L1_HEIGHT);
        mapAbs.put(elevatorPositions.L2_HEIGHT, ElevatorConstants.L2_HEIGHT);
        mapAbs.put(elevatorPositions.L3_HEIGHT, ElevatorConstants.L3_HEIGHT);
        mapAbs.put(elevatorPositions.L4_HEIGHT, ElevatorConstants.L4_HEIGHT);
        mapAbs.put(elevatorPositions.SOURCE_HEIGHT, ElevatorConstants.SOURCE_HEIGHT);
      //  mapAbs.put(armPositions.INTAKE, ArmConstants.INTAKE);
      //  mapAbs.put(armPositions.POOP, ArmConstants.POOP);



      m_elevatorLead = new TalonFX(ElevatorConstants.kLeadMotorID);
        m_elevatorFollow = new TalonFX(ElevatorConstants.kFollowMotorID);
        m_elevatorFollow.setControl(new Follower(m_elevatorLead.getDeviceID(), true));


        m_leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_leadConfig.Slot0 = new Slot0Configs().withKP(kP).withKI(0).withKD(0);
        m_leadConfig.Feedback.SensorToMechanismRatio = reduction;
        m_leadConfig.Voltage.PeakForwardVoltage = 12;
        m_leadConfig.Voltage.PeakReverseVoltage = -8;
        //m_leadConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
      //  m_leadConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
        m_leadConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        m_leadConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

        m_leadConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_leadConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        //m_leadConfig.

        m_elevatorLead.getConfigurator().apply(m_leadConfig, 0.25);
        m_leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_elevatorFollow.getConfigurator().apply(m_leadConfig,0.25);
        position = m_elevatorLead.getPosition();
        velocity = m_elevatorLead.getVelocity();



        BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity);

        ParentDevice.optimizeBusUtilizationForAll(m_elevatorLead, m_elevatorFollow);




        p = kP;
        i = kI;
        d = kD;
        //m_pidController.enableContinuousInput(0, 1);
        profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(4.0, 2.0));

        SmartDashboard.putNumber("Arm/goal", currentGoal);
        SmartDashboard.putNumber("Arm/KP", kP);
        SmartDashboard.putNumber("Arm/KI", kI);
        SmartDashboard.putNumber("Arm/KD", kD);

        SmartDashboard.putNumber("Arm/KS", kS);
        SmartDashboard.putNumber("Arm/KG", kG);
        SmartDashboard.putNumber("Arm/KV", kV);
        SmartDashboard.putNumber("Arm/KA", kA);

        SmartDashboard.putNumber("Arm/KSTuner", kS_tuner);
    }

    @Override
    public void periodic() {
        pos = m_elevatorLead.getPosition().getValueAsDouble();
        //SmartDashboard.putNumber("Arm Relative Enc", m_armRight.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm/ArmABS Absolute", pos);
        //SmartDashboard.putNumber("Arm/RightSide Encoder" , m_armFollow.getAlternateEncoder().getPosition());
        //SmartDashboard.putNumber("Arm oCurrent", m_armRight.getOutputCurrent());
        //SmartDashboard.putNumber("Arm Motor Speed", m_speed);
        SmartDashboard.putNumber("Arm/LeftMotor", m_elevatorLead.getSupplyCurrent().getValueAsDouble());
        //SmartDashboard.putNumber("Right Motor", m_armLeft.getOutputCurrent());

        if(isTuning)
        {
            tuneNumbers();
        }

        m_speed = velocity.getValueAsDouble();
        SmartDashboard.putNumber("Arm/setpointState", setpointState.position);
        if(profile.isFinished(m_timer.get() + .06))
        {
            setpointState = new TrapezoidProfile.State(currentGoal, 0);
            updateMotionProfile();
             setpointState =
          profile.calculate(
              m_timer.get(),
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      currentGoal,
                      Constants.ElevatorConstants.kElevatorMinOutput,
                      Constants.ElevatorConstants.kElevatorMaxOutput),
                   0.0));
        }
        else{
        setpointState =
          profile.calculate(
              m_timer.get(),
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      currentGoal,
                      Constants.ElevatorConstants.kElevatorMinOutput,
                      Constants.ElevatorConstants.kElevatorMaxOutput),
                      0.0));
        }
        // Once given pro we can chang positionVoltagerequest to torqueCurrentRequest
        feedforward = ff.calculate(setpointState.position, setpointState.velocity);
        m_elevatorLead.setControl(positionVoltageRequest.withPosition(setpointState.position).withFeedForward(feedforward));
    }

    private void updateMotionProfile() {
        m_timer.reset();
      }
      private double getP()
      {
          return p;
      }
      private double getI()
      {
          return i;
      }
      private double getD()
      {
          return d;
      }

    public boolean raiseArmAbs(elevatorPositions position){


        double ref = mapAbs.get(position);
        currentGoal = ref;
        updateMotionProfile();
        return false;
    }

    public void updatePID()
    {
        m_leadConfig.Slot0.kP = kP;
        m_leadConfig.Slot0.kI = kI;
        m_leadConfig.Slot0.kD = kD;
        p = kP;
        i = kI;
        d = kD;
    }

    public boolean atPosition(){
        double currentEncoderPosition = position.getValueAsDouble();
        return (Math.abs(currentEncoderPosition - currentGoal) < Constants.ArmConstants.kAllowedErrAbs);
    }

    public void noArmPower()
    {
        m_elevatorLead.set(0);
    }

    public void tuneNumbers()
    {
        currentGoal = SmartDashboard.getNumber("Arm/goal", currentGoal);
        kP = SmartDashboard.getNumber("Arm/KP", kP);
        kI = SmartDashboard.getNumber("Arm/KI", kI);
        kD = SmartDashboard.getNumber("Arm/KD", kD);

        kS = SmartDashboard.getNumber("Arm/KS", kS);
        kG = SmartDashboard.getNumber("Arm/KG", kG);
        kV = SmartDashboard.getNumber("Arm/KV", kV);
        kA = SmartDashboard.getNumber("Arm/KA", kA);

        kS_tuner = SmartDashboard.getNumber("Arm/KSTuner", kS_tuner);

        if( getP() != kP || getI() != kI || getD() != kD)
        {
            updatePID();
        }

        if(ff.getKa() != kA || ff.getKg() != kG || ff.getKs() != kS || ff.getKv() != kV)
        {
            ff = new ElevatorFeedforward(kS, kG, kV, kA);
        }
    }

}