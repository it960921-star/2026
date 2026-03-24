package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

public class MAXSwerveModule {
  
  private final SparkFlex driveMotor;
  private final SparkMax turningMotor;

  private final RelativeEncoder drivingEncoder;
  private final RelativeEncoder turningEncoder;
  private final CANcoder absoluteEncoder;

  private final SparkClosedLoopController drivingController;
  private final SparkClosedLoopController turningController;

  private final boolean absoluteEncoderReverse;

  private double angle;
  private final double angleOffset;

  public MAXSwerveModule(int drivingCANId, int turningCANId,int absoluteEncoderId, boolean driveMotorReverse, boolean turningMotorReverse,boolean absoluteEncoderReversed, double angleOffset ) {
    this.angleOffset = angleOffset;
    this.absoluteEncoderReverse = absoluteEncoderReversed;
      // 初始化 Offset
    
    driveMotor = new SparkFlex(drivingCANId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningCANId, MotorType.kBrushless);

    absoluteEncoder = new CANcoder(absoluteEncoderId);

    drivingEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    drivingController = driveMotor.getClosedLoopController();
    turningController = turningMotor.getClosedLoopController();

    driveMotor.configure(Configs.MAXSwerveModule.drivingConfig.inverted(driveMotorReverse), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    turningMotor.configure(Configs.MAXSwerveModule.turningConfig.inverted(turningMotorReverse), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    resetEncoders();
  }

    public double getDrivePosition(){
    return drivingEncoder.getPosition();
  }
  public double getAbsoluteEncoderRad() {
    // 取得原始數值 (CANcoder 預設為 0~1)
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    // 轉換為弧度
    angle = angle * 2.0 * Math.PI;
    // 處理反向
    angle *= absoluteEncoderReverse ? -1.0 : 1.0;
    // 減去校準位移 (Offset)
    angle = angle - angleOffset;

    // 將角度限制在 -PI 到 PI 之間
    return MathUtil.angleModulus(angle);
}

  
  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return drivingEncoder.getVelocity();
  }

  public double getTurningVelocity(){
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoerRad(){
    angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
    angle *= absoluteEncoderReverse ? -1.0d : 1.0d;
    SmartDashboard.putNumber("absolutEncoderAngle", angle);
    return angle;
  }

  public void resetEncoders() {
    drivingEncoder.setPosition(0);
    // 必須使用計算過 Offset 的值
    turningEncoder.setPosition(getAbsoluteEncoderRad());
}

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDrivePosition(),
      Rotation2d.fromRadians(getTurningPosition()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(),
        Rotation2d.fromRadians(getTurningPosition()) // 使用馬達內部的相對編碼器位置
    );
}

  
  // 1. 在類別頂部定義變數
  private SwerveModuleState lastDesiredState = new SwerveModuleState();

  // 2. 修改原本的 setDesiredState
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = new SwerveModuleState();
    state.angle = Rotation2d.fromRadians(getTurningPosition());
    desiredState.optimize(state.angle);
    
    // 存起來給 AdvantageScope 用
    this.lastDesiredState = desiredState;

    drivingController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    turningController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
  }
  public void logModuleData(String moduleName) {
    // 轉向 PID 曲線 (單位：Radians)
    Logger.recordOutput("Swerve/PID/" + moduleName + "/SteerSetpoint", lastDesiredState.angle.getRadians());
    Logger.recordOutput("Swerve/PID/" + moduleName + "/SteerMeasured", getTurningPosition());

    // 驅動 PID 曲線 (單位：m/s)
    Logger.recordOutput("Swerve/PID/" + moduleName + "/DriveSetpoint", lastDesiredState.speedMetersPerSecond);
    Logger.recordOutput("Swerve/PID/" + moduleName + "/DriveMeasured", getDriveVelocity());
}

  // 3. 新增一個 Getter 方法
    public SwerveModuleState getDesiredState() {
    return lastDesiredState;
  }

  public void stop() {
    driveMotor.stopMotor();
    turningMotor.stopMotor();
  }
  // 在 MAXSwerveModule.java 內
  // 在 MAXSwerveModule.java 裡

  

}
