//aim idk how to make it aim with the gyro so have fun <3
Aim = TrajectoryGenerator.generateTrajectory(?

//shoot
shoot = TrajectoryGenerator.generateTrajectory(new shooterRunWheelsCMD(shooter speed, 1))

//put out intake
BringOutIntake = TrajectoryGenerator.generateTrajectory(new ArmSetCMD(theArmSystem, 0.4))

//run intake
RunIntake = TrajectoryGenerator.generateTrajectory(new intakeRunWheelsCMD(intake speed, 1))

//stop intake
stopIntake = TrajectoryGenerator.generateTrajectory(if color sensor sees ring then 
                                                    new intakeRunWheelsCMD(intake speed, 0))                

//bring up intake
BringUpIntake = TrajectoryGenerator.generateTrajectory(new ArmSetCMD(theArmSystem, 0), new intakeRunWheelsCMD(intake speed, -1))










//Red Left

//This Auton path shoots, drives to the center, picks up, drives back, shoots, drives out again and shoots. 

List.of (Aim, shoot, new Translation2d(-0.61, 8.23, new Rotation2d(0)), BringOutIntake, RunIntake, 
          stopIntake, BringUpIntake, new Translation2d(0, 0, new Rotation2d(0)), Aim, shoot,
          new Translation2d(0.61, 8.23, new Rotation2d(0)), BringOutIntake, RunIntake, 
          stopIntake, BringUpIntake, new Translation2d(0, 0, new Rotation2d(0)), Aim, shoot)

//17 point auto







//Red Mid

//This Auton path shoots, drives to the center, picks up, drives back, shoots, drives out again and shoots. 

List.of (Aim, shoot, BringOutIntake, RunIntake,  new Translation2d(0, 1.524, new Rotation2d(0)),
          stopIntake, BringUpIntake, Aim, shoot, new Translation2d(1.22, 1.22, new Rotation2d(0)), 
          BringOutIntake, RunIntake, new Translation2d(1.22, 1.524, new Rotation2d(0)), stopIntake, 
          BringUpIntake, Aim, shoot)

//17 point auto







//Red Right

//This Auton path shoots, drives to the center, picks up, drives back, shoots, drives out again and shoots. 

List.of (Aim, shoot, put out intake, run intake, new Translation2d(0, 1.8288, new Rotation2d(0)), 
          stop intake, bring up intake, aim, shoot, put out intake, run intake,
           new Translation2d(0.3048, 8.2296, new Rotation2d(0)), stop intake, bring up intake, 
           new Translation2d(0, 3.048, new Rotation2d(0)), aim, shoot)






//Blue Left

//This Auton path shoots, drives to the center, picks up, drives back, shoots, drives out again and shoots. 

List.of (Aim, shoot, put out intake, run intake, new Translation2d(0, 1.8288, new Rotation2d(0)), 
          stop intake, bring up intake, aim, shoot, put out intake, run intake,
           new Translation2d(-0.3048, 8.2296, new Rotation2d(0)), stop intake, bring up intake, 
           new Translation2d(0, 3.048, new Rotation2d(0)), aim, shoot)
//17 point auto









//Blue Mid

//This Auton path shoots, drives to the center, picks up, drives back, shoots, drives out again and shoots. 

List.of (Aim, shoot, BringOutIntake, RunIntake,  new Translation2d(0, 1.524, new Rotation2d(0)),
          stopIntake, BringUpIntake, Aim, shoot, new Translation2d(-1.22, 1.22, new Rotation2d(0)), 
          BringOutIntake, RunIntake, new Translation2d(-1.22, 1.524, new Rotation2d(0)), stopIntake, 
          BringUpIntake, Aim, shoot)
//17 point auto











//Blue Right

//This Auton path shoots, drives to the center, picks up, drives back, shoots, drives out again and shoots. 

List.of (Aim, shoot, new Translation2d(0.61, 8.23, new Rotation2d(0)), BringOutIntake, RunIntake, 
          stopIntake, BringUpIntake, new Translation2d(0, 0, new Rotation2d(0)), Aim, shoot,
          new Translation2d(-0.61, 8.23, new Rotation2d(0)), BringOutIntake, RunIntake, 
          stopIntake, BringUpIntake, new Translation2d(0, 0, new Rotation2d(0)), Aim, shoot)


//17 point auto
