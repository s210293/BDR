
#include <sys/time.h>
#include <cstdlib>

#include "umission.h"
#include "utime.h"
#include "ulibpose2pose.h"



UMission::UMission(UBridge * regbot, UCamera * camera)
{
  cam = camera;
  bridge = regbot;
  threadActive = 100;
  // initialize line list to empty
  for (int i = 0; i < missionLineMax; i++)
  { // add to line list 
    lines[i] = lineBuffer[i];    
    //  terminate c-strings strings - good practice, but not needed
    lines[i][0] = '\0';
  }
  // start mission thread
  th1 = new thread(runObj, this);
//   play.say("What a nice day for a stroll\n", 100);
//   sleep(5);
}


UMission::~UMission()
{
  printf("Mission class destructor\n");
}


void UMission::run()
{
  while (not active and not th1stop)
    usleep(100000);
//    printf("UMission::run:  active=%d, th1stop=%d\n", active, th1stop);
  if (not th1stop)
    runMission();
  printf("UMission::run: mission thread ended\n");
}
  
void UMission::printStatus()
{
  printf("# ------- Mission ----------\n");
  printf("# active = %d, finished = %d\n", active, finished);
  printf("# mission part=%d, in state=%d\n", mission, missionState);
}
  
void UMission::missionInit()
{ // stop any not-finished mission
  bridge->send("robot stop\n");
  // clear old mission
  bridge->send("robot <clear\n");
  //
  // add new mission with 3 threads
  // one (100) starting at event 30 and stopping at event 31
  // one (101) starting at event 31 and stopping at event 30
  // one (  1) used for idle and initialisation of hardware
  // the mission is started, but staying in place (velocity=0, so servo action)
  //
  bridge->send("robot <add thread=1\n");
  // Irsensor should be activated a good time before use 
  // otherwise first samples will produce "false" positive (too short/negative).
  bridge->send("robot <add irsensor=1,vel=0:dist<0.2\n");
  //
  // alternating threads (100 and 101, alternating on event 30 and 31 (last 2 events)
  bridge->send("robot <add thread=100,event=30 : event=31\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    //  are to be replaced with real mission
    // NB - hereafter no lines can be added to these threads, just modified
    bridge->send("robot <add vel=0 : time=0.1\n");
  //
  bridge->send("robot <add thread=101,event=31 : event=30\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    bridge->send("robot <add vel=0 : time=0.1\n");
  usleep(10000);
  //
  //
  // send subscribe to bridge
  bridge->pose->subscribe();
  bridge->edge->subscribe();
  bridge->motor->subscribe();
  bridge->event->subscribe();
  bridge->joy->subscribe();
  bridge->motor->subscribe();
  bridge->info->subscribe();
  bridge->irdist->subscribe();
  bridge->imu->subscribe();
  usleep(10000);
  // there maybe leftover events from last mission
  bridge->event->clearEvents();
}


void UMission::sendAndActivateSnippet(char ** missionLines, int missionLineCnt)
{
  // Calling sendAndActivateSnippet automatically toggles between thread 100 and 101. 
  // Modifies the currently inactive thread and then makes it active. 
  const int MSL = 100;
  char s[MSL];
  int threadToMod = 101;
  int startEvent = 31;
  // select Regbot thread to modify
  // and event to activate it
  if (threadActive == 101)
  {
    threadToMod = 100;
    startEvent = 30;
  }
  if (missionLineCnt > missionLineMax)
  {
    printf("# ----------- error - too many lines ------------\n");
    printf("# You tried to send %d lines, but there is buffer space for %d only!\n", missionLineCnt, missionLineMax);
    printf("# set 'missionLineMax' to a higher number in 'umission.h' about line 57\n");
    printf("# (not all lines will be send)\n");
    printf("# -----------------------------------------------\n");
    missionLineCnt = missionLineMax;
  }
  // send mission lines using '<mod ...' command
  for (int i = 0; i < missionLineCnt; i++)
  { // send lines one at a time
    if (strlen((char*)missionLines[i]) > 0)
    { // send a modify line command
      snprintf(s, MSL, "<mod %d %d %s\n", threadToMod, i+1, missionLines[i]);
      bridge->send(s); 
    }
    else
      // an empty line will end code snippet too
      break;
  }
  // let it sink in (10ms)
  usleep(10000);
  // Activate new snippet thread and stop the other  
  snprintf(s, MSL, "<event=%d\n", startEvent);
  bridge->send(s);
  // save active thread number
  threadActive = threadToMod;
}



void UMission::runMission()
{ /// current mission number
  mission = fromMission;
  int missionOld = mission;
  bool regbotStarted = false;
  /// end flag for current mission
  bool ended = false;
  /// manuel override - using gamepad
  bool inManual = false;
  /// debug loop counter
  int loop = 0;
  // keeps track of mission state
  missionState = 0;
  int missionStateOld = missionState;
  // fixed string buffer
  const int MSL = 120;
  char s[MSL];
  /// initialize robot mission to do nothing (wait for mission lines)
  missionInit();
  /// start (the empty) mission, ready for mission snippets.
  bridge->send("start\n"); // ask REGBOT to start controlled run (ready to execute)
  bridge->send("oled 3 waiting for REGBOT\n");
//   play.say("Waiting for robot data.", 100);
  ///
  for (int i = 0; i < 3; i++)
  {
    if (not bridge->info->isHeartbeatOK())
    { // heartbeat should come at least once a second
      sleep(2);
    }
  }
  if (not bridge->info->isHeartbeatOK())
  { // heartbeat should come at least once a second
    play.say("Oops, no usable connection with robot.", 100);
//    system("espeak \"Oops, no usable connection with robot.\" -ven+f4 -s130 -a60 2>/dev/null &"); 
    bridge->send("oled 3 Oops: Lost REGBOT!");
    printf("# ---------- error ------------\n");
    printf("# No heartbeat from robot. Bridge or REGBOT is stuck\n");
//     printf("# You could try restart ROBOBOT bridge ('b' from mission console) \n");
    printf("# -----------------------------\n");
    //
    if (false)
      // for debug - allow this
      stop();
  }
  /// loop in sequence every mission until they report ended
  while (not finished and not th1stop)
  { // stay in this mission loop until finished
    loop++;
    // test for manuel override (joy is short for joystick or gamepad)
    if (bridge->joy->manual)
    { // just wait, do not continue mission
      usleep(20000);
      if (not inManual)
      {
//         system("espeak \"Mission paused.\" -ven+f4 -s130 -a40 2>/dev/null &"); 
        play.say("Paused.", 90);
      }
      inManual = true;
      bridge->send("oled 3 GAMEPAD control\n");
    }
    else
    { // in auto mode
      if (not regbotStarted)
      { // wait for start event is received from REGBOT
         // - in response to 'bot->send("start\n")' earlier
        if (bridge->event->isEventSet(33))
        { // start mission (button pressed)
//           printf("Mission::runMission: starting mission (part from %d to %d)\n", fromMission, toMission);
          regbotStarted = true;
        }
      }
      else
      { // mission in auto mode
        if (inManual)
        { // just entered auto mode, so tell.
          inManual = false;
//           system("espeak \"Mission resuming.\" -ven+f4 -s130 -a40 2>/dev/null &");
          play.say("Mission resuming", 90);
          bridge->send("oled 3 running AUTO\n");
        }
        switch(mission)
        {
          case 1: // running auto mission
            ended = mission1(missionState);
            break;
          //case 2:
            //ended = mission2(missionState);
            //break;
          //case 3:
            //ended = mission3(missionState);
            //break;
          //case 4:
            //ended = mission4(missionState);
            //break;
		  //case 5:
			//ended = mission5(missionState);
			//break;
          default:
            // no more missions - end everything
            finished = true;
            break;
        }
        if (ended)
        { // start next mission part in state 0
          mission++;
          ended = false;
          missionState = 0;
        }
        // show current state on robot display
        if (mission != missionOld or missionState != missionStateOld)
        { // update small O-led display on robot - when there is a change
          UTime t;
          t.now();
          snprintf(s, MSL, "oled 4 mission %d state %d\n", mission, missionState);
          bridge->send(s);
          if (logMission != NULL)
          {
            fprintf(logMission, "%ld.%03ld %d %d\n", 
                    t.getSec(), t.getMilisec(),
                    missionOld, missionStateOld
            );
            fprintf(logMission, "%ld.%03ld %d %d\n", 
                    t.getSec(), t.getMilisec(),
                    mission, missionState
            );
          }
          missionOld = mission;
          missionStateOld = missionState;
        }
      }
    }
    //
    // check for general events in all modes
    // gamepad buttons 0=green, 1=red, 2=blue, 3=yellow, 4=LB, 5=RB, 6=back, 7=start, 8=Logitech, 9=A1, 10 = A2
    // gamepad axes    0=left-LR, 1=left-UD, 2=LT, 3=right-LR, 4=right-UD, 5=RT, 6=+LR, 7=+-UD
    // see also "ujoy.h"
    if (bridge->joy->button[BUTTON_RED])
    { // red button -> save image
      if (not cam->saveImage)
      {
        printf("UMission::runMission:: button 1 (red) pressed -> save image\n");
        cam->saveImage = true;
      }
    }
    if (bridge->joy->button[BUTTON_YELLOW])
    { // yellow button -> make ArUco analysis
      if (not cam->doArUcoAnalysis)
      {
        printf("UMission::runMission:: button 3 (yellow) pressed -> do ArUco\n");
        cam->doArUcoAnalysis = true;
      }
    }
    // are we finished - event 0 disables motors (e.g. green button)
    if (bridge->event->isEventSet(0))
    { // robot say stop
      finished = true;
      printf("Mission:: insist we are finished\n");
    }
    else if (mission > toMission)
    { // stop robot
      // make an event 0
      bridge->send("stop\n");
      // stop mission loop
      finished = true;
    }
    // release CPU a bit (10ms)
    usleep(10000);
  }
  bridge->send("stop\n");
  snprintf(s, MSL, "Robot %s finished.\n", bridge->info->robotname);
//   system(s); 
  play.say(s, 100);
  printf("%s", s);
  bridge->send("oled 3 finished\n");
}


//Safe track
/*
bool UMission::mission1(int & state)
{
	bool finished = false;
	
	switch (state){
		case 0:
			printf("# press green to start.\n");
			play.say("Press green to start", 90);
			bridge->send("oled 5 press green to start");
			state++;
			break;
		
		
		case 1:
			if (bridge->joy->button[BUTTON_GREEN]){
				state = 10;
			}
			break;
		
		
		case 10:
		{
			// Mission 1 - From start to up the ramp
			printf("# Mission1.\n");
			play.say("Running mission 1.", 90);
		
			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.6,acc=3,edgel=0,white=1:dist=1.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.6,acc=3,edgel=0,white=1:dist=15,xl>1");
			snprintf(lines[line++], MAX_LEN, "vel=0.4,tr=0.2:turn=80.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.3,edger=0.0,white=1:dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3,acc=3: time=2");
			snprintf(lines[line++], MAX_LEN, "vel=0.3,edger=0.0,white=1:dist=0.9");
			snprintf(lines[line++], MAX_LEN, "vel=0.15,edger=0.0,white=1:time=15.0, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.2,acc=3.0:dist=0.15");
			snprintf(lines[line++], MAX_LEN, "vel=-0.35,acc=3.0:time=1.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=3.0: dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.1: turn=-60.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.5, acc=3: dist=1.65, xl>1");
			snprintf(lines[line++], MAX_LEN, "vel=0.5, acc=3: dist=0.1");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, acc=3: dist=1.0, xl>1");
			snprintf(lines[line++], MAX_LEN, "vel=0.2, tr=0.0: turn=-95.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edger=0, white=1: dist=0.10");
			snprintf(lines[line++], MAX_LEN, "vel=0.8, acc=3, edger=0, white=1: dist=2.8");
			
			snprintf(lines[line++], MAX_LEN, "event=1,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
		
			//Mission 1 end
			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(1);

			printf("# case=%d sent mission snippet 1\n", state);
			bridge->send("oled 5 code snippet 1");

			state = 11;
			featureCnt = 0;
		}
		case 11:
		{
			if (bridge->event->isEventSet(1)){
				state = 12;
			}
			break;
		}
		case 12:
		{
			//Mission 2 - Down the stairs
			printf("# Mission2.\n");
			play.say("Running mission 2.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edger=0, white=1: dist=0.10");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edger=0, white=1: dist=1, xl=1, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.25, acc=3: dist=0.25");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.25: turn=-85");
			snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=3.0, edger=0,white=1: time=5, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.25, acc=3.0: time=1.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0, edger=0,white=1: time=3, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.25, acc=3.0: time=1.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0, edger=0,white=1: time=3, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.25, acc=3.0: time=1.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0, edger=0,white=1: time=3, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.25, acc=3.0: time=1.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0, edger=0,white=1: time=3, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0: time=1.2");
			
			snprintf(lines[line++], MAX_LEN, "event=2,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 2 end
			
			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(2);

			printf("# case=%d sent mission snippet 2\n", state);
			bridge->send("oled 5 code snippet 2");

			state = 13;
			featureCnt = 0;
			break;
		}
		case 13:
		{
			if (bridge->event->isEventSet(2)){
				state = 14;
				
			}
			break;
		}
		
		case 14:
		{
			//Mission 3
			printf("# Mission3.\n");
			play.say("Running mission 3.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0, edger=0,white=1: time=3, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0: time=1.2");
			snprintf(lines[line++], MAX_LEN, "vel=0.6, acc=3, edgel=0, white=1: dist=10, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-85.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.5: dist=1.45");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=92.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.5: dist=1.87");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=90.0");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3: time=4");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.39");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.35");
			
			snprintf(lines[line++], MAX_LEN, "event=3,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1"); //This line is needed for now.
			//Mission 3 end
			
			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(3);

			printf("# case=%d sent mission snippet 3\n", state);
			bridge->send("oled 5 code snippet 3");

			state = 15;
			featureCnt = 0;
			break;
		}
		
		case 15:
		{
			if (bridge->event->isEventSet(3)){
				state = 16;
				
			}
			break;
		}
		
		case 16:
		{
			//Mission 4 - Racetrack
			printf("# Mission4.\n");
			play.say("Running mission 4.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, acc=2, edger=0, white=1: dist=2.45, lv<1"); //distance might need to change
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2<0.35");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2>0.35");
			snprintf(lines[line++], MAX_LEN, "vel=1.5, acc=5, edger=0, white=1: dist=1, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=1.5, acc=5, edger=0, white=1: dist=10, lv<1, ir1<0.3");
			snprintf(lines[line++], MAX_LEN, "vel=1.5, acc=5, edger=0, white=1: dist=0.1");
			snprintf(lines[line++], MAX_LEN, "vel=1.5, acc=5, edger=0, white=1: dist=10, lv<1, ir1<0.3");
			
			snprintf(lines[line++], MAX_LEN, "event=4,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 3 end
			

			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(4);

			printf("# case=%d sent mission snippet 3\n", state);
			bridge->send("oled 5 code snippet 3");

			state = 17;
			featureCnt = 0;
			break;
		}
		
		case 17:
		{
			if (bridge->event->isEventSet(4)){
				state = 18;
			
			}
			break;
		}
		
		case 18:
		{
			//Mission 4 - End of the racetrack
			printf("# Mission4.\n");
			play.say("Running mission 3.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.1");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=2");
			snprintf(lines[line++], MAX_LEN, "vel=-0.4, tr=0.1: turn =-90");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90");
			snprintf(lines[line++], MAX_LEN, "vel=0.6, acc=3, edgel=0, white=1: dist=10, lv<1, xl>15");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=127");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2<0.4");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2>0.4");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=0.8: dist=1");

			snprintf(lines[line++], MAX_LEN, "event=5,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 4 end
	

			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(5);

			printf("# case=%d sent mission snippet 5\n", state);
			bridge->send("oled 5 code snippet 4");

			state = 19;
			featureCnt = 0;
			break;
		}
		
		case 19:
		{
			if (bridge->event->isEventSet(5)){
				state = 20;
				
			}
			break;
		}
		
		case 20:
		{
			//Mission 5 - Causel to finish
			printf("# Mission5.\n");
			play.say("Running mission 5.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1:turn=-60");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.17");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=37");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.32");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=65");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.31");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=72");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.30");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=70");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.32");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-48");
			snprintf(lines[line++], MAX_LEN, "vel=0.4:dist=0.05");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2<0.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2>0.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=6");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.28");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-92");
			snprintf(lines[line++], MAX_LEN, "vel=0.6, acc=3, edgel=0, white=1: dist=10, lv<1, xl>15");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=145");
			snprintf(lines[line++], MAX_LEN, "vel=0.5: dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=0, white=1: dist=0.5, lv<1");

			snprintf(lines[line++], MAX_LEN, "event=6,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 5 end
			

			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(6);

			printf("# case=%d sent mission snippet 6\n", state);
			bridge->send("oled 5 code snippet 6");

			state = 21;
			featureCnt = 0;
			break;
		}
		
		case 21:
		{
			if (bridge->event->isEventSet(6)){
				state = 999;
				
			}
			break;
		}
		
		case 999:
			printf("Vitus er sej \n");
			bridge->send("oled 5 \"mission 1 ended.\"");
			finished = true;
			break;
		

		default:
			printf("Laura is cool \n");
			bridge->send("oled 5 \"mission 1 ended.\"");
			finished = true;
			break;
		
	}
	return finished;
}
*/

//All the missions together
bool UMission::mission1(int & state)
{
	bool finished = false;
	
	switch (state){
		case 0:
		printf("# press green to start.\n");
		play.say("Press green to start", 90);
		bridge->send("oled 5 press green to start");
		state++;
		break;
		
		case 1:
		if (bridge->joy->button[BUTTON_GREEN]){
			state = 10; //NEEDS TO BE 10
		}
		break;
		
		
		case 10:
		{
			//Mission 1 - From start to up the ramp
			printf("# Mission1.\n");
			play.say("Running mission 1.", 90);
			
			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.6,acc=3,edgel=0,white=1:dist=4.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.6,acc=3,edgel=0,white=1:dist=15,xl>1");
			snprintf(lines[line++], MAX_LEN, "vel=0.4,tr=0.2:turn=80.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.3,edger=0.0,white=1:dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3,acc=3: time=2");
			snprintf(lines[line++], MAX_LEN, "vel=0.3,edger=0.0,white=1:dist=0.9");
			snprintf(lines[line++], MAX_LEN, "vel=0.15,edger=0.0,white=1:time=15.0, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.2,acc=3.0:dist=0.15");
			snprintf(lines[line++], MAX_LEN, "vel=-0.35,acc=3.0:time=1.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=3.0: dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.1: turn=-60.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.5, acc=3: dist=1.65, xl>1");
			snprintf(lines[line++], MAX_LEN, "vel=0.5, acc=3: dist=0.1");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, acc=3: dist=2.0, xl>1");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0.0: turn=-95.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edger=0, white=1: dist=0.10");
			snprintf(lines[line++], MAX_LEN, "vel=0.8, acc=3, edger=0, white=1: dist=2.86");
			
			snprintf(lines[line++], MAX_LEN, "event=1,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			
			//Mission 1 end
			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(1);

			printf("# case=%d sent mission snippet 1\n", state);
			bridge->send("oled 5 code snippet 1");

			state = 11;
			featureCnt = 0;
		}
		case 11:
		{
			if (bridge->event->isEventSet(1)){
				state = 12;
			}
			break;
		}
		case 12:
		{
			//Mission 2 - Get the ball
			printf("# Mission2.\n");
			play.say("Running mission 2.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edger=0, white=1: dist=0.10");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edger=0, white=1: dist=1, xl=1, lv<1");  // finds crosslines LINK CAP A LES ESCALES
			snprintf(lines[line++], MAX_LEN, "vel=0.0:time=2");
			//snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");  
			snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.1:turn=-69"); //pick up the ball from crosslines
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=245, vservo=180");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=2");
			snprintf(lines[line++], MAX_LEN, "vel=0.4,tr=0.1:turn=-118");   //-118
			snprintf(lines[line++], MAX_LEN, "vel=0.3:dist=0.60");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=2");

			//sweeping
			snprintf(lines[line++], MAX_LEN, "tr=0, vel=-0.23, acc=1: turn=40");
			snprintf(lines[line++], MAX_LEN, "tr=0, vel=-0.23, acc=1: turn=-65"); 
			snprintf(lines[line++], MAX_LEN, "vel=0:time=0.05");
			snprintf(lines[line++], MAX_LEN, "vel=-0.27:dist=0.04");
			snprintf(lines[line++], MAX_LEN, "tr=0, vel=-0.23, acc=1: turn=65");
			snprintf(lines[line++], MAX_LEN, "tr=0, vel=-0.23, acc=1: turn=-65");
			snprintf(lines[line++], MAX_LEN, "vel=0:time=0.05");
			snprintf(lines[line++], MAX_LEN, "vel=-0.27:dist=0.04");
			
			snprintf(lines[line++], MAX_LEN, "event=2,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");

			
			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(2);

			printf("# case=%d sent mission snippet 2\n", state);
			bridge->send("oled 5 code snippet 2");

			state = 13;
			featureCnt = 0;
			break;
		}
		case 13:
		{
			if (bridge->event->isEventSet(2)){
				state = 14;
				
			}
			break;
		}

		case 14: 
		{
			//Mission 3 
			printf("# Mission2.\n");
			play.say("Running mission 14 dawg.", 90);

			int line = 0;
			
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=250, vservo=180");
			
			snprintf(lines[line++], MAX_LEN, "tr=0, vel=-0.23, acc=1: turn=70");
			snprintf(lines[line++], MAX_LEN, "tr=0, vel=-0.23, acc=1: turn=-70");		//sweepign
			snprintf(lines[line++], MAX_LEN, "vel=0:time=0.05");
			//snprintf(lines[line++], MAX_LEN, "vel=-0.3:dist=0.04");
			
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");	// get back to white line
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.12");						//HERE DISTANCE   (IF IT DOESN'T HIT THE WHITE LINE AFTER GETTING THE BALL IN change)
			snprintf(lines[line++], MAX_LEN, "vel=-0.3, tr=0.25: turn=-70"); //115
			snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.1");	

			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edger=0, white=1: dist=0.10"); //find edge
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edger=0, white=1: dist=1, xl=1, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.25, acc=3: dist=0.28"); //0.33
			snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.25: turn=-85"); //point towards the stairs

			//snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.1");   //MAYBE DELETE THIS?
			
			snprintf(lines[line++], MAX_LEN, "event=3,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			
			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(3);

			printf("# case=%d sent mission snippet 2\n", state);
			bridge->send("oled 5 code snippet 2");

			state = 15;
			featureCnt = 0;
			break;

		}

		case 15:
		{
			if (bridge->event->isEventSet(3)){
				state = 16;
				
			}
			break;
		}
		
		
		case 16: //down the stairs
		{
			//Mission 3 - Down the stairs
			printf("# Mission2.\n");
			play.say("Running mission 14.", 90);

			int line = 0;
			
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0, edgel=0,white=1: time=5, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3, acc=3.0: time=1.5"); //1a escala
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0, edgel=0,white=1: time=3, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3, acc=3.0: time=1.5"); // 2a escala
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0, edgel=0,white=1: time=3, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, acc=3.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3, acc=3.0: time=1.5"); // 3a escala
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0, edgel=0,white=1: time=3, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3, acc=3.0: time=1.5"); //4a escala
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0, edgel=0,white=1: time=3, lv<1"); //THIS
			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3: time=2");  //THIS

			//snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3: time=1.5");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3: time=4"); //5a escala

			//snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0, edgel=0,white=1: time=3, lv<1");
			//snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3.0: time=1.2");
			//snprintf(lines[line++], MAX_LEN, "vel=-0.4, tr=0.1: turn=8");
			//snprintf(lines[line++], MAX_LEN, "vel=-0.3: time=3");

			snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=3, edgel=0, white=1: dist=0.10");
			snprintf(lines[line++], MAX_LEN, "vel=0.6, acc=3, edgel=0, white=1: dist=10, lv<1");

			snprintf(lines[line++], MAX_LEN, "event=4,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			
			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(4);

			printf("# case=%d sent mission snippet 2\n", state);
			bridge->send("oled 5 code snippet 2");

			state = 17;
			featureCnt = 0;
			break;

		}

		case 17: /// DONT CHANGE ANYTHING HERE
		{
			if (bridge->event->isEventSet(4)){
				state = 18;
				
			}
			break;
		}

		case 18:
		{
			//Mission 3 - From the floor after the last stair (green part)
			printf("# Mission3.\n");
			play.say("Laura is the best.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.4");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90.0");
			snprintf(lines[line++], MAX_LEN, "vel=-0.4: time=3");
			
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.262");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=85.0");     //FIRST TREE  (89)
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=200, vservo=200"); //THIS
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=1.20");
			snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0.1: turn=4");    
			snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.21");				//ldeixa la bola
			snprintf(lines[line++], MAX_LEN, "vel=-0.2: dist=0.42");			
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90.0");	
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=-0.4: time=3");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.2");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=1.4");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=90.0");
			snprintf(lines[line++], MAX_LEN, "vel=-0.4: time=4");
			
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.665");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=83");     //SECOND TREE (MOVE THIS UP AGAIN)
			
			snprintf(lines[line++], MAX_LEN, "event=5,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1"); //This line is needed for now.
			//Mission 3 end
			
			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(5);

			printf("# case=%d sent mission snippet 3\n", state);
			bridge->send("oled 5 code snippet 3");

			state = 19;
			featureCnt = 0;
			break;
		}
		
		case 19:
		{
			if (bridge->event->isEventSet(5)){
				state = 20;
				
			}
			break;
		}
		
		case 20:
		{
			//Mission 4 - Tree stuff and little box stuff
			printf("# Mission4.\n");
			play.say("Running mission 20.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=200, vservo=200");
			
			snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=1.2");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.1: turn=12");
			snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.57"); //cap al aruc, (0.54)
			snprintf(lines[line++], MAX_LEN, "vel=-0.4: dist=0.34");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90");
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			
			snprintf(lines[line++], MAX_LEN, "vel=-0.4: time=5");	 //NEW
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=1.2");	//NEW  (1.35)
			
			//snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.31");					//0.313
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-88");
			snprintf(lines[line++], MAX_LEN, "vel=0: time=2"); 
			
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=260, vservo=200"); //Box mission starts here
			snprintf(lines[line++], MAX_LEN, "vel=0.73: dist=0.75");
			snprintf(lines[line++], MAX_LEN, "vel=0: time=2");
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=90.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.2");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=83.0"); //fine tune to drive inside box
			snprintf(lines[line++], MAX_LEN, "vel=0.35: dist=1.1"); //inside box
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90.0");
			
			snprintf(lines[line++], MAX_LEN, "event=6,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 3 end
			

			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(6);

			printf("# case=%d sent mission snippet 3\n", state);
			bridge->send("oled 5 code snippet 3");

			state = 21;
			featureCnt = 0;
			break;
		}
		
		case 21:
		{
			if (bridge->event->isEventSet(6)){
				state = 22;
				
			}
			break;
		}
		
		case 22:
		{
			//Mission 5 - finish the little box until the racetrack
			printf("# Mission4.\n");
			play.say("Running mission 3.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-85.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=1.05");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-84.0");
			snprintf(lines[line++], MAX_LEN, "vel=0: time=2");
			snprintf(lines[line++], MAX_LEN, "vel=0.7: dist=0.35");
			snprintf(lines[line++], MAX_LEN, "vel=0: time=2");
			snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.1: turn=-85.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.2: time=4"); //closes 1st gate
			snprintf(lines[line++], MAX_LEN, "vel=-0.3: dist=0.4"); 
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=85.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.6");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-87.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=1.12"); //distance to maybe fine tune
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-87.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.76");          //0.83  (78) IF THE ROBOT GOES TOO FAR, BEFORE WE TURN TO THE RACE TRACK)
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=85.0");
			snprintf(lines[line++], MAX_LEN, "vel=-0.3: time=5");
		
			//snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.1");
			//snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-10");   //LAST TURN
			//snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.52");   //DISTANCE FROM THE LAST TURN TO THE RACETRACK PART (Might need to change)?

			snprintf(lines[line++], MAX_LEN, "event=7,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 4 end
			

			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(7);

			printf("# case=%d sent mission snippet 5\n", state);
			bridge->send("oled 5 code snippet 4");

			state = 23;
			featureCnt = 0;
			break;
		}
		
		case 23:
		{
			if (bridge->event->isEventSet(7)){
				state = 24;
				
			}
			break;
		} 
		
		case 24:
		{
			//Mission 4 - Racetrack
			printf("# Mission4.\n");
			play.say("Running mission 4.", 90);

			int line = 0;

			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=200");
			////// NEW CODE to look towards the white line
	
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=90.0");
			snprintf(lines[line++], MAX_LEN, "vel=-0.4: time=4");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.45");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90.0");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.7");
			//// 
			snprintf(lines[line++], MAX_LEN, "vel=0.4, acc=2, edger=0, white=1: dist=2.02, lv<1"); //distance might need to change 2.45
			//snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5);
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2<0.35"); //0.35
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2>0.35"); // 0.35
			//snprintf(lines[line++], MAX_LEN, "vel=0: time=0.4");
			snprintf(lines[line++], MAX_LEN, "vel=1.6, acc=5, edger=0, white=1: dist=1, lv<1");
			snprintf(lines[line++], MAX_LEN, "vel=1.5, acc=5, edger=0, white=1: dist=10, lv<1, ir1<0.3");
			snprintf(lines[line++], MAX_LEN, "vel=1.5, acc=5, edger=0, white=1: dist=0.1");
			snprintf(lines[line++], MAX_LEN, "vel=1.5, acc=5, edger=0, white=1: dist=10, lv<1, ir1<0.3");
			
			snprintf(lines[line++], MAX_LEN, "event=8,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 3 end
			

			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(8);

			printf("# case=%d sent mission snippet 3\n", state);
			bridge->send("oled 5 code snippet 3");

			state = 25;
			featureCnt = 0;
			break;
		}
		
		case 25:
		{
			if (bridge->event->isEventSet(8)){
				state = 26;
				
			}
			break;
		}
		
		
		case 26:
		{
			//Mission 4 - End of the racetrack till looking at the causel
			printf("# Mission4.\n");
			play.say("Running mission 3.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.1");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=2");
			snprintf(lines[line++], MAX_LEN, "vel=-0.4, tr=0.1: turn =-90");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-90");
			snprintf(lines[line++], MAX_LEN, "vel=0.2, edgel=0,white=1: dist=0.10"); // this line was added NOW
			snprintf(lines[line++], MAX_LEN, "vel=0.6, acc=3, edgel=0, white=1: dist=10, lv<1, xl>15");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=120"); //127   (124)  (This is the turn looking into the causel - change if the robot can't get up into the causel)
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2<0.4");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2>0.4");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=1");
			snprintf(lines[line++], MAX_LEN, "vel=0.8: dist=1.03");  //vel=0.9
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=1"); //new line added

			snprintf(lines[line++], MAX_LEN, "event=9,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 4 end
			

			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(9);

			printf("# case=%d sent mission snippet 5\n", state);
			bridge->send("oled 5 code snippet 4");

			state = 27;
			featureCnt = 0;
			break;
		}
		
		case 27:
		{
			if (bridge->event->isEventSet(9)){
				state = 28;
				
			}
			break;
		}
		
		case 28:
		{
			//Mission 5 - Causel to finish (Roundabout)
			printf("# Mission5.\n");
			play.say("Running mission 5.", 90);

			int line = 0;
			snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-850, vservo=200");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1:turn=-56"); //63 (60)
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.18");  //(0.18)
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=38"); //37
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.30");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=65");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.32");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=69");
			snprintf(lines[line++], MAX_LEN, "vel=0.3: dist=0.33");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=70");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.32");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-48");
			snprintf(lines[line++], MAX_LEN, "vel=0.4:dist=0.05");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2<0.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=20.0, ir2>0.5");
			snprintf(lines[line++], MAX_LEN, "vel=0.0: time=3");
			snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.28");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=-92");
			snprintf(lines[line++], MAX_LEN, "vel=0.6, acc=3, edgel=0, white=1: dist=10, lv<1, xl>15");
			snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1: turn=145");
			snprintf(lines[line++], MAX_LEN, "vel=0.5: dist=0.3");
			snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=0, white=1: dist=0.5, lv<1");

			snprintf(lines[line++], MAX_LEN, "event=10,vel=0.0");
			snprintf(lines[line++], MAX_LEN, ": dist=1");
			//Mission 5 end
			

			sendAndActivateSnippet(lines, line);
			bridge->event->isEventSet(10);

			printf("# case=%d sent mission snippet 6\n", state);
			bridge->send("oled 5 code snippet 6");

			state = 29;
			featureCnt = 0;
			break;
		}
		
		case 29:
		{
			if (bridge->event->isEventSet(10)){
				state = 999;
				
			}
			break;
		}
		
		case 999:
		printf("Vitus er sej \n");
		bridge->send("oled 5 \"mission 1 ended.\"");
		finished = true;
		break;
		

		default:
		printf("Laura is cool \n");
		bridge->send("oled 5 \"mission 1 ended.\"");
		finished = true;
		break;
		
	}
	return finished;
}


void UMission::openLog()
{
  // make logfile
  const int MDL = 32;
  const int MNL = 128;
  char date[MDL];
  char name[MNL];
  UTime appTime;
  appTime.now();
  appTime.getForFilename(date);
  // construct filename ArUco
  snprintf(name, MNL, "log_mission_%s.txt", date);
  logMission = fopen(name, "w");
  if (logMission != NULL)
  {
    const int MSL = 50;
    char s[MSL];
    fprintf(logMission, "%% Mission log started at %s\n", appTime.getDateTimeAsString(s));
    fprintf(logMission, "%% Start mission %d end mission %d\n", fromMission, toMission);
    fprintf(logMission, "%% 1  Time [sec]\n");
    fprintf(logMission, "%% 2  mission number.\n");
    fprintf(logMission, "%% 3  mission state.\n");
  }
  else
    printf("#UCamera:: Failed to open image logfile\n");
}

void UMission::closeLog()
{
  if (logMission != NULL)
  {
    fclose(logMission);
    logMission = NULL;
  }
}
