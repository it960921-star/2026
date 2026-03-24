package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Swerve.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
    // 1. 宣告子系統
    private final DriveSubsystem swerveSubsystem = new DriveSubsystem();
    
    // 2. 宣告控制器 (使用 Port 0，對應 Constants 設定)
    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    
    // 3. 自動路徑選擇器
    private SendableChooser<Command> autoChooser = new SendableChooser<>();
        
        public RobotContainer() {
            // A. 設定底盤的「預設指令」
            // 這會讓機器人隨時都在讀取搖桿數值並移動
            boolean isCompetition = true;
            autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> isCompetition
                        ? stream.filter(auto -> auto.getName().startsWith("eric"))
                        : stream);
            SmartDashboard.putData("Auto Chooser", autoChooser);
            
            
        

        // B. 設定按鈕綁定
        // 在 RobotContainer 建構子內
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> -driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis),
            () -> true
        ));
        configureButtonBindings();
    }
        

        // C. 設定自動路徑選項 (如果有 PathPlanner 路徑可以加在這裡)
        public Command getAutonomousCommand() {
            return autoChooser.getSelected();
        }

    

     // 預設為 1.0 公尺

private double targetDistance = 1.0; // 預設 1 公尺

private void configureButtonBindings() {
    

    // [Y 鍵]：自動移動到距離 Tag 1.0m 處，並校正車頭與左右位置
    driverController.y().whileTrue(
    swerveSubsystem.alignToTag(0.0, 1.0) 
    );

    // [B 鍵]：車頭自動朝向 Tag，但前後左右位置由左搖桿決定
    driverController.b().whileTrue(
        swerveSubsystem.trackTargetWithJoystick(
            () -> -driverController.getLeftY(), 
            () -> -driverController.getLeftX()
        )
    );

    // [A 鍵]：切換斜向距離 (例如 1.0m <-> 1.5m)
    // 這裡會影響 LB 和 RB 的執行結果
    driverController.a().onTrue(
        Commands.runOnce(() -> swerveSubsystem.toggleTargetDistance(), swerveSubsystem)
    );

    // [LB 鍵]：左斜 45 度，距離採用 A 鍵設定的值
    driverController.leftBumper().whileTrue(
        swerveSubsystem.alignTo45Degree(45.0)
    );

    // [RB 鍵]：右斜 45 度，距離採用 A 鍵設定的值
    driverController.rightBumper().whileTrue(
        swerveSubsystem.alignTo45Degree(-45.0)
    );

    // 原有的其他按鍵
    driverController.start().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    driverController.x().whileTrue(new InstantCommand(() -> swerveSubsystem.setX(), swerveSubsystem));
    
    
    
    


   
}
}