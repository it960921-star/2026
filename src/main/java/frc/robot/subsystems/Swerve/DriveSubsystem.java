package frc.robot.subsystems.Swerve;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Newton;

import java.util.function.Supplier;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private double m_targetDistance = 1.0; // 儲存 A 鍵切換的距離
  private final PIDController xPID = new PIDController(0.8, 0, 0.001); // 前後偏移 PID
  private final PIDController yPID = new PIDController(0.08, 0, 0.01); // 左右偏移 PID
  private final PIDController rotPID = new PIDController(0.08, 0, 0.01);
      // 設定 PID 範圍
      {
        xPID.setTolerance(0.05); // 5cm 誤差容忍
        yPID.setTolerance(0.05);
        rotPID.enableContinuousInput(-180,180);
        rotPID.setTolerance(1.5);
        
    }
    
  private final MAXSwerveModule[] modules = new MAXSwerveModule[4];
  private final AHRS gyro;
  private final SwerveDriveOdometry m_odometry;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  
  // 在 DriveSubsystem 類別內
  // 在 DriveSubsystem 類別內
private static final String kFrontLimelight = "limelight";
private static final String kBackLimelight = "limelight-back";
private final HttpCamera limelightBackCamera;
private final HttpCamera limelightFrontCamera;

// 加上這一個，用於 Elastic 顯示地圖
private final Field2d field = new Field2d();

// 設定視覺信任度標準差 (X, Y, Rotation)
// 增加旋轉的標準差 (999999) 是因為我們通常更信任 navX 的陀螺儀
  private final edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> 
    visionStdDevs = VecBuilder.fill(0.7, 0.7, 999999);

  PathConstraints constraints = new PathConstraints(
        4.0, 2.6,
        Units.degreesToRadians(520), Units.degreesToRadians(600));
  public DriveSubsystem() {
       gyro = new AHRS(NavXComType.kMXP_SPI);

        modules[0] = new MAXSwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.FrontLeftdriveMotorReverse,
        DriveConstants.FrontLeftTurningMotorReverse,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontLeftAngleOffset);
        modules[1] = new MAXSwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.FrontRightdriveMotorReverse,
        DriveConstants.FrontRightTurningMotorReverse,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontRightAngleOffset);
        
        modules[2] = new MAXSwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.BackLeftdriveMotorReverse,
        DriveConstants.BackLeftdTurningMotorReverse,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kBackLeftAngleOffset);
        
        
        modules[3] = new MAXSwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.BackRightdriveMotorReverse,
        DriveConstants.BackRightTurningMotorReverse,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        DriveConstants.kBackRightAngleOffset);
       

    resetEncoders();
    zeroHeading();
        
    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      });

        RobotConfig config = null;
        try{
          config = RobotConfig.fromGUISettings();
        }catch(Exception e) {
          e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getdriveRobotRelative, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPHolonomicDriveController(
                    new PIDConstants(2.8, 0.001, 0.006),  // 6,0,0
                    new PIDConstants(3.0, 0.005, 0.0)  // 5,0,0
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
     getRotation2d(),
     new SwerveModulePosition[] {
         modules[0].getPosition(),
         modules[1].getPosition(),
         modules[2].getPosition(),
         modules[3].getPosition()},
    getPose());

    SmartDashboard.putData("Field", field);

    limelightBackCamera = new HttpCamera("limelight-back", "http://10.76.45.200:5800/stream.mjpg");
    limelightFrontCamera = new HttpCamera("limelight", "http://10.76.45.201:5800/stream.mjpg");
    
    CameraServer.addCamera(limelightBackCamera);
    CameraServer.addCamera(limelightFrontCamera);

    // 預先設定好 Dashboard 需要的路徑，不需要在 periodic 重複寫入
    SmartDashboard.putString("CameraPublisher/limelight-back/streams", "mjpeg:http://10.76.45.200:5800/stream.mjpg");
    SmartDashboard.putString("CameraPublisher/limelight/streams", "mjpeg:http://10.76.45.201:5800/stream.mjpg");

    }

  @Override
public void periodic() {
  
   m_poseEstimator.update(gyro.getRotation2d(), getModulePositions());
    m_odometry.update(gyro.getRotation2d(), getModulePositions());

    // 2. 融合視覺數據
    updateVisionMeasurement(kFrontLimelight);
    updateVisionMeasurement(kBackLimelight);

    // 3. 計算距離 (只計算一次)
    double distance = 0;
    if (LimelightHelpers.getTV(kFrontLimelight)) {
        distance = calculateDistance(LimelightHelpers.getTY(kFrontLimelight), kFrontLimelight);
    }

    // 4. 更新必要的數據到 Dashboard
    updateToAdvantageScope();
    SmartDashboard.putNumber("Vision/Distance", distance);
    
    // 盡量減少 String 的寫入，或者只在狀態改變時才寫入
    // SmartDashboard.putString("RobotState", "Teleop"); 
    
}

private void updateVisionMeasurement(String llName) {
    if (LimelightHelpers.getTV(llName)) {
        // MegaTag2
        LimelightHelpers.SetRobotOrientation(llName, 
            m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
        
        // 增加一個簡單的延遲過濾，避免每 0.02s 都進行昂貴的運算
        if (Math.abs(gyro.getRate()) < 720 && mt2.tagCount > 0) {
            m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
    }
}
private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    };
}
  

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  

  public void getTurning() {
    SmartDashboard.putNumber("FL", modules[0].getTurningPosition());
    SmartDashboard.putNumber("FR", modules[1].getTurningPosition());
    SmartDashboard.putNumber("BL", modules[2].getTurningPosition());
    SmartDashboard.putNumber("BR", modules[3].getTurningPosition());
  }

  public void getVelocity() {
    SmartDashboard.putNumber("FL V", modules[0].getDriveVelocity());
    SmartDashboard.putNumber("FR V", modules[1].getDriveVelocity());
    SmartDashboard.putNumber("BL V", modules[2].getDriveVelocity());
    SmartDashboard.putNumber("BR V", modules[3].getDriveVelocity());

  }

  public ChassisSpeeds getdriveRobotRelative() {
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
      modules[0].getState(),
      modules[1].getState(),
      modules[2].getState(),
      modules[3].getState());

  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
      setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
       
  }
  
  private double getTargetYawForTag(String activeLL) {
    long id = (long) LimelightHelpers.getFiducialID("limelight");
    // 2026 賽季地圖參考：
    // 假設 1-5 號 Tag 在紅方牆壁 (面向 180度)
    // 假設 11-15 號 Tag 在藍方牆壁 (面向 0度)
    if (id >= 1 && id <= 5) return 180.0;
    if (id >= 11 && id <= 15) return 0.0;
    
    // 若沒看到特定 ID，則保持當前最接近的 90 度倍數角度
    return Math.round(getHeading() / 90.0) * 90.0;
}

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(0),//gyro.getAngle()),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        pose);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        modules[0].setDesiredState(desiredStates[0]);
        modules[1].setDesiredState(desiredStates[1]);
        modules[2].setDesiredState(desiredStates[2]);
        modules[3].setDesiredState(desiredStates[3]);
  }
  


  

  public void resetEncoders() {
    modules[0].resetEncoders();
    modules[1].resetEncoders();
    modules[2].resetEncoders();
    modules[3].resetEncoders();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }



  public void stopModules() {
    modules[0].stop();
    modules[1].stop();
    modules[2].stop();
    modules[3].stop();
  }

  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void get() {
    m_poseEstimator.getEstimatedPosition();
  }

  public void setUpVision() {
    m_poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
        });

    boolean useMegaTag2 = true;
    boolean doRejectUpdate = false;
      if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(gyro.getRate()) > 720) 
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }
  

 // 預設 1 公尺

public void toggleTargetDistance() {
    m_targetDistance = (m_targetDistance == 1.0) ? 1.5 : 1.0;
    System.out.println("目前目標距離設定為: " + m_targetDistance + "m");
}
public double getTargetDistance() {
    return m_targetDistance;
}
// 3. 定位指令 (給 LB/RB 用)
// 確保方法名稱叫 alignToTag，且只接收一個 double 角度參數
public Command alignToTag(double txOffset, double distance) {
    return run(() -> {
        String activeLL = "limelight";
        if (LimelightHelpers.getTV(kFrontLimelight)) {
            activeLL = kFrontLimelight;
        } else if (LimelightHelpers.getTV(kBackLimelight)) {
            activeLL = kBackLimelight;
        }

        if (!activeLL.isEmpty()) {
            double tx = LimelightHelpers.getTX(activeLL);
            double ty = LimelightHelpers.getTY(activeLL);
            double currentDistance = calculateDistance(ty, activeLL); 

            // --- 速度控制優化 ---
            // 計算距離誤差
            double distanceError = currentDistance - distance; 
            
            // 1. 前後速度 (X)：靠近 1m 時速度放慢
            // 我們將 P 值設適中，並利用 clamp 限制最高速
            // 當誤差小於 0.3m 時，PID 會自然線性減速
            double xSpeed = xPID.calculate(currentDistance, distance);
            double limitedXSpeed = MathUtil.clamp(xSpeed, -0.4, 0.4); // 限制最大出力為 40%

            // 2. 左右速度 (Y)：同樣給予適度限制
            double ySpeed = yPID.calculate(tx, txOffset);
            double limitedYSpeed = MathUtil.clamp(ySpeed, -0.3, 0.3);
            
            // 3. 旋轉速度 (Rot)：對準目標角度
            double targetYaw = getTargetYawForTag(activeLL); 
            double rotSpeed = rotPID.calculate(getHeading(), targetYaw);
            double limitedRotSpeed = MathUtil.clamp(rotSpeed, -0.3, 0.3);

            // 執行移動
            this.drive(limitedXSpeed, limitedYSpeed, limitedRotSpeed, true);
            
            // Log 偵錯數據到 Dashboard
            SmartDashboard.putNumber("Vision/DistanceError", distanceError);
            SmartDashboard.putNumber("Vision/OutputX", limitedXSpeed);
        } else {
            this.stopModules();
        }
    });
}

public Command alignTo45Degree(double targetTx) {
    return run(() -> {
        String activeLL = LimelightHelpers.getTV(kFrontLimelight) ? kFrontLimelight : 
                         (LimelightHelpers.getTV(kBackLimelight) ? kBackLimelight : "");

        if (!activeLL.isEmpty()) {
            double tx = LimelightHelpers.getTX(activeLL);
            double ty = LimelightHelpers.getTY(activeLL);
            double currentDistance = calculateDistance(ty, activeLL);

            // --- 動態幾何計算 ---
            // 使用 m_targetDistance 作為斜邊 (L)
            // 機器人正前方距離 (鄰邊) = L * cos(45°)
            double targetForwardDistance = m_targetDistance * Math.cos(Math.toRadians(45));

            // 前後 PID (X)
            double xSpeed = xPID.calculate(currentDistance, targetForwardDistance);
            double limitedXSpeed = MathUtil.clamp(xSpeed, -0.4, 0.4);

            // 左右 PID (Y) - 這裡 targetTx 是 45 或 -45
            double ySpeed = yPID.calculate(tx, targetTx);
            double limitedYSpeed = MathUtil.clamp(ySpeed, -0.4, 0.4);

            // 旋轉 PID (Rot)
            double targetYaw = getTargetYawForTag(activeLL);
            double rotSpeed = rotPID.calculate(getHeading(), targetYaw);
            double limitedRotSpeed = MathUtil.clamp(rotSpeed, -0.3, 0.3);

            this.drive(limitedXSpeed, limitedYSpeed, limitedRotSpeed, true);
            
            // 在 Dashboard 顯示目前目標，方便駕駛員確認
            SmartDashboard.putNumber("Vision/TargetDiagonalDist", m_targetDistance);
        } else {
            this.stopModules();
        }
    });
}

// 修正後的正面定位：改為接收 specificDistance，確保 Y 鍵可以固定 1m
public Command trackTargetWithJoystick(Supplier<Double> xSpeed, Supplier<Double> ySpeed) {
    return run(() -> {
        double rotationOutput = 0;
        if (LimelightHelpers.getTV("limelight")) {
            // 自動計算轉向，使 tx 為 0
            rotationOutput = rotPID.calculate(LimelightHelpers.getTX("limelight"), 0);
        }
        // 使用傳入的搖桿數值進行平移，使用自動計算的轉向
        this.drive(xSpeed.get(), ySpeed.get(), rotationOutput, true);
    });
}
            
   public double calculateDistance(double ty,String llName) {
    // 請務必使用皮尺測量以下數值（單位建議統一使用公尺）
    double kLimelightHeight;
    double kMountAngle;
    double kTargetHeight = 0.25; // 2026 Tag 高度

    if (llName.equals(kFrontLimelight)) {
        kLimelightHeight = 0.23; // 前相機高度
        kMountAngle = 0.0;      // 前相機仰角
    } else {
        kLimelightHeight = 0.23; // 假設後相機裝比較高
        kMountAngle = 0;     // 假設後相機有仰角
    }

    double angleToGoalRadians = Math.toRadians(kMountAngle + ty);
    return (kTargetHeight - kLimelightHeight) / Math.tan(angleToGoalRadians);
}
            
            // 輔助方法：計算與 AprilTag 的距離 (需根據實際安裝高度調整)


// 2. 支援 B 鍵的鎖定轉向方法
public Command trackTargetWithDrive(Supplier<Double> xSpd, Supplier<Double> ySpd) {
    return run(() -> {
        double rotationSpeed = 0;
        
        // 1. 修正名稱為 "limelight"
        if (LimelightHelpers.getTV("limelight")) {
            double tx = LimelightHelpers.getTX("limelight");
            
            // 2. 使用 rotPID (建議 P 值設為 0.015 ~ 0.02 開始調教)
            // MathUtil.clamp 限制最大轉速為 0.4 (40%)，防止偏移過猛
            rotationSpeed = MathUtil.clamp(rotPID.calculate(tx, 0), -0.4, 0.4);
        }

        // 3. 執行移動
        this.drive(
            xSpd.get() * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
            ySpd.get() * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
            rotationSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
            true
        );
    });
}
             
    private void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds;
    if (fieldRelative) {
        // 場地相對坐標 (Field Oriented)
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d());
    } else {
        // 機器人相對坐標 (Robot Oriented)
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }
    
    // 將底盤速度轉換為四個模組的狀態
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // 確保速度不超過物理上限
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
    setModuleStates(swerveModuleStates);
}  
                  
        
          public void updateToAdvantageScope() {
    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    
    // 1. 記錄機器人位置 (建議加上分類路徑)
    Logger.recordOutput("Swerve/RobotPose", currentPose);

    // 2. 記錄實際輪子狀態 (藍色箭頭)
    SwerveModuleState[] actualStates = new SwerveModuleState[] {
        modules[0].getState(), modules[1].getState(),
        modules[2].getState(), modules[3].getState()
    };
    Logger.recordOutput("Swerve/ActualStates", actualStates);

    // 3. 記錄目標輪子狀態 (紅色或空心箭頭)
    SwerveModuleState[] desiredStates = new SwerveModuleState[] {
        modules[0].getDesiredState(), modules[1].getDesiredState(),
        modules[2].getDesiredState(), modules[3].getDesiredState()
    };
    Logger.recordOutput("Swerve/DesiredStates", desiredStates);
    for (int i = 0; i < modules.length; i++) {
    String name = "Module" + i;
    modules[i].logModuleData(name);
  field.setRobotPose(currentPose);
    modules[0].logModuleData("FrontLeft");
    modules[1].logModuleData("FrontRight");
    modules[2].logModuleData("BackLeft");
    modules[3].logModuleData("BackRight");
    
    
  }}


  public void resetPose(Pose2d startPose) {
    // TODO Auto-generated method stub
    
  }

  public void setX() {
    // 讓四個輪子分別轉向 45, -45, -45, 45 度，形成 X 字型
    // 速度設為 0，確保機器人靜止不動
    modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
}

 
  
  /**
 * 自動旋轉底盤以對準 Limelight 看到的目標 (AprilTag)
 */
}