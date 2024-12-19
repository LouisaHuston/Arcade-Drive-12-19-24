package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


// test take 2

public class Robot extends TimedRobot {
    // Motor CAN IDs
    private static final int LEFT_FRONT_ID = 1;
    private static final int LEFT_REAR_ID = 2;
    private static final int RIGHT_FRONT_ID = 4;
    private static final int RIGHT_REAR_ID = 3;

    // SPARK MAX Motor Controllers
    private final SparkMax leftFrontMotor = new SparkMax(LEFT_FRONT_ID, MotorType.kBrushed);
    private final SparkMax leftRearMotor = new SparkMax(LEFT_REAR_ID, MotorType.kBrushed);
    private final SparkMax rightFrontMotor = new SparkMax(RIGHT_FRONT_ID, MotorType.kBrushed);
    private final SparkMax rightRearMotor = new SparkMax(RIGHT_REAR_ID, MotorType.kBrushed);


    // Differential Drive
    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    // Motor map for dynamic setup
    private final Map<String, SparkMax> motors = new HashMap<>();

    // Joystick
    private final Joystick joystick = new Joystick(1);

    public Robot(){

      // Populate the map with motors
      motors.put("LeftFront", leftFrontMotor);
      motors.put("LeftRear", leftRearMotor);
      motors.put("RightFront", rightFrontMotor);        
      motors.put("RightRear", rightRearMotor);
    }
    @Override
    public void robotInit() {
        // configure all motors dynamically
        for(Map.Entry<String, SparkMax> entry : motors.entrySet()){
          String name = entry.getKey();
          SparkMax motor = entry.getValue();

          if(name.contains("Front")){
            configureMotor(motor, name.contains("Right")); // Invert right lead motors
          } 
          else if (name.contains("Rear")){
            String leadName = name.replace ("Rear", "Front");
            SparkMax leadMotor = motors.get(leadName);
            if (name.contains("Left")){
              configureFollowerMotor(motor, leadMotor, LEFT_FRONT_ID);
            }
            else{
              configureFollowerMotor(motor, leadMotor, RIGHT_FRONT_ID);
            }
          }
        }
 
        
        // Differential Drive Safety
        differentialDrive.setSafetyEnabled(true);
    }

    // Method to configure lead motors
    private void configureMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted);
      config.idleMode(IdleMode.kBrake);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Method to configure follower motors
    private void configureFollowerMotor(SparkMax follower, SparkMax leader, int leaderCANID){
      SparkMaxConfig config = new SparkMaxConfig();
      config.follow(leaderCANID);
      config.inverted(false); // Match direction of the leader
      follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     
    }

    @Override
    public void teleopPeriodic() {
        // Get raw joystick inputs
        double rawSpeed = -joystick.getY(); // Forward/backward
        double rawRotation = joystick.getX(); // Left/right turn

        // Apply deadband
        double speed = applyDeadband(rawSpeed, 0.1);
        double rotation = applyDeadband(rawRotation, 0.1);

        // Scale inputs for smoother driving
        speed = scaleInput(speed);
        rotation = scaleInput(rotation);

        // Arcade drive control
        differentialDrive.arcadeDrive(speed, rotation);
    }

    /**
     * Applies a deadband to joystick input.
     * 
     * @param value - The raw joystick input.
     * @param deadband - The deadband threshold.
     * @return The adjusted input value.
     */

    private double applyDeadband(double value, double deadband){
      if(Math.abs(value) < deadband){
        return 0.0;
      }
      return value;
    }

    /**
     * Scales the joystick input using a squared response curve.
     * 
     * @param value - The input value after applying deadband.
     * @return The scaled input value.
     */

     private double scaleInput(double value){
      // Scale using a squared curve while preserving the sign
      return Math.signum(value) * Math.pow(value, 2);
     }

    @Override
    public void disabledInit() {
        // Stop motors when disabled
        differentialDrive.stopMotor();
    }
}