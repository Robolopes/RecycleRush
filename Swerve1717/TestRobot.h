/*
---------------------------------------------------------------------------------------

                   :hhdmNy                        
                   dMMMMMMy       `:s+.           
        `-`       :MMMMMMMMs`` `-smMMMMds`        
      .oNMNho:.`./dMMMMMMMMMNmddMMMMMMMMy         
     :NMMMMMMMNmMMMMNNmmdmmNNNMMMMMMMMMm`         
      +NMMMMMMMMNho//++osso++/+sdNMMMMM+          
       :mMMMMNh+:ohNMMMMMMMMMMmh+:omMMMNs---::::. 
       `hMMMd/:hNMMmmMMMMMMMMdNMMNs-oNMMMMMMMMMMh`
`.--:+ohMMMh.oNMMMs`oMMMMMMMM.-dMMMm::mMMMMMMMMMMo                Team 1717
sNNMMMMMMMm.+MMMMN` oMMMMMMMN. +MMMMN-/MMMMMMMNms:     Dos Pueblos Engineering Academy
dMMMMMMMMMs NMMMMN. `+yhhhhy:  oMMMMMy mMMMMds:`                    2009
sdmNMMMMMMo`MMMMMMh.   `--`   :NMMMMMh dMMMN`     
 `.-+ymMMMd hMMMMMMd/       `oNMMMMMM+.NMMMNo-    
      .NMMM+.dMMMMMMN/      hMMMMMMMy`hMMMMMMNh/` 
     .sNMMMNo-yNMMMMMo      mMMMMMm+-hMMMMMMMMMNd`
   `omMMMMMMMd/:smMMMo      mMMNdo:omMMMMMMMMMMN+ 
  `dMMMMMMMMMMMms+/+s/      ss+/+yNMMMMh+oossyy+  
   .yMMMMNmhymMMMMNmhso+//+oyhmNMMMMMM+`          
     /o/-``` `-sMMMMMMMMMMMMMMMMMMMMMMy           
               .MMMMMMMMNhyssodMMMMMMMN.          
               .MMMMMMMm-     `:hNMMMNd:          
               `NMMMMMy`         .so:`            
             `-/+/                            

---------------------------------------------------------------------------------------
*/

#ifndef TEST_ROBOT_H_
#define TEST_ROBOT_H_

#include "WPILib.h"
#include <iostream.h>
#include <taskLib.h>
#include <kernelLib.h>
#include "DriveTrain/DriveTrainManager.h"


/**
 * This is the main class for the robot
 */
//#define SUPER_DEBUG



class TestRobot : public IterativeRobot
{
	public:
		
		DriveTrainManager* 	mDrivetrainManager;

		Counter*	mCounter;
		
		TestRobot();
		void RobotInit(void);
		void DisabledInit(void);
		
//                                                                                   
//           Hood: Servo                                                           
//                  
//                ------
//               /  /        <>                                                           
//              |<>|              <>                                                  
//           ____________                                                          
//          |   Turret   |                                                         
//          |Rotate:     |                                                         
//          |____________|                                                         
//                                    
//      Gabe and Wojcik are machining bevel gears, ROFL
//           CODEZCODEZCODEZCODEZCODEZ      
		void AutonomousInit(void);
		void TeleopInit(void);
		
		void DisabledPeriodic(void);
		
		/**
		 * The function that is called over and over again during autonomous.
		 */
		void AutonomousPeriodic(void);
		
		/**
		 * The function that is called over and over again during teleop.
		 */
		void TeleopPeriodic(void);
		
		
		DriveSide*			mLeft;
		DriveSide*			mRight;
};

#endif
