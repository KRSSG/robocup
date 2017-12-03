#include <list>
#include "pExec.h"
#include "robot.h"
#include  "tactics/tactic.h"
#include <fstream>

using namespace std;
//using namespace HAL;

namespace Strategy
{
  PExec::PExec(krssg_ssl_msgs::BeliefState* state,ros::NodeHandle& n) :
    NaivePS(*state)
  {
     //state1=state;
    for (int botID = 0; botID < HomeTeam::SIZE; ++botID)
    {
      //tactic[botID] = new  Tactic();
      robot[botID]    = new Robot(botID,n);
    }
  } // PExec

  PExec::~PExec()
  {
    for (int botID = 0; botID < HomeTeam::SIZE; ++botID)
    {
      //delete tactic[botID];
      delete robot[botID];
    }
  } // ~PExec

//*********************this function assigns roles to bots **********************************************

  void PExec::assignRoles(krssg_ssl_msgs::BeliefState &bs)
  {
    if (playID == PlayBook::None)
    {
      return;
    }
    Play* currPlay = playList[playID];
    bool goodBot[6] = {true, true, true, true, true, true};
    int goodBotCount = 6;
    // Initialization
    list<int> freeBots;
    for (int botID = 0; botID < HomeTeam::SIZE; ++botID) // Iterating over all bots - making them all free for role allocation
    {
      if(goodBot[botID])
      freeBots.push_back(botID);
    }
    for (int roleIdx = 0; roleIdx < HomeTeam::SIZE; ++roleIdx) // Iterating over all roles
    {
      if (currTacticIdx[roleIdx] < currPlay->roleList[roleIdx].size()) // Tactic exists for the current role iteration
      {
        std::string    tID     = currPlay->roleList[roleIdx][currTacticIdx[roleIdx]].first;
        Tactic::Param tParam  = currPlay->roleList[roleIdx][currTacticIdx[roleIdx]].second;
        
        int           bestBot;

       if(tID.compare("TReceive")==0)
        {
          bestBot=currPlay->recvrID;
        }
        else if(tID.compare("TDribbleTurnPass")==0)
        {
          bestBot=3;
        }
        else if (tID.compare("TPassToPoint")==0)
        {
          bestBot=currPlay->pasrID;
        }
        else 
        {
          selTactic = TacticFactory::instance()->Create(tID, roleIdx);
          bestBot=(*selTactic).chooseBestBot(bs,freeBots,tParam);
          selTactic.reset ();
          cout<<"best bot for "<<tID<<" is "<<bestBot<<endl;
          //bestBot = robot[roleIdx]->curTactic.get()->chooseBestBot(bs,freeBots, tParam); 
        }
        roleBotMapping[roleIdx]=bestBot;
        freeBots.remove(bestBot);
        
       //  // Saving the current tactic to be executed by the bot selected
       //  currTactic[bestBot]   = currPlay->roleList[roleIdx][currTacticIdx];
       // // Util::Logger::toStdOut("Updating tactics\n");
       //  // Updating the tactic of the selected bot
        robot[bestBot]->tID    = tID;
        robot[bestBot]->tParamJSON =TacticFactory::instance()->Create(tID, 0)->paramToJSON(tParam);
        //robot[bestBot]->tStateSh = Tactic::INIT;
      }
      if(roleIdx == goodBotCount-1) {
        for (int botID = 0; botID < HomeTeam::SIZE; ++botID) // Iterating over all bots - making them all free for role allocation
        {
          if(!goodBot[botID])
          freeBots.push_back(botID);
        }
      }
    }
    //*returnIndx=currTacticIdx;
    //Logger::toStdOut("Assigned roles for tactic index %d\n", currTacticIdx);
  } // asisgnRoles

//############### Can transit #################

  // bool PExec::canTransit(void)
  // {
  //   //if play is none then can transit

  //   if (playID == PlayBook::None)
  //   {
  //      ////printf("can transit \n");
  //     return true;
  //   }

  //   //if current tactic Index is greater than number of max tactics per roles then can not transit 
  //   // if (currTacticIdx >= playList[playID]->maxTacticsPerRole)
  //   // {
  //   //   //printf("can not transit 1 \n");
  //   //   return false;
  //   // }

  //   Play* currPlay = playList[playID];

  //   int numActiveTactics = 0;
  //   //fstream file;
  //   //file.open("/home/gunjan/catkin_ws/src/play/logger.txt",fstream::out | fstream::app);
  //   for (int roleID = 0; roleID < HomeTeam::SIZE; ++roleID)
  //   {
  //     std::string tID       = currPlay->roleList[roleID][currTacticIdx].first;
  //     Tactic*    selTactic = robot[roleID]->curTactic.get();
  //     //file<<"here 0 \n";
  //     if (selTactic->isActiveTactic()==true)
  //     {
  //       ++numActiveTactics;
  //       //file<<"here 1"<<endl;
        
  //         if (selTactic->isCompleted(state)==false)
  //         {
  //           // If there is at least one incomplete active tactic, then cannot transit
  //           fstream f;
  //           f.open("/home/gunjan/catkin_ws/src/play/out.txt",fstream::out|fstream::app);
  //           f<<tID<<" - is not Completed :D \n";
  //           f.close();
  //           return false;
  //         }
  //         else
  //         {
  //           fstream f;
  //           f.open("/home/gunjan/catkin_ws/src/play/out.txt",fstream::out|fstream::app);
  //           f<<tID<<" - is Completed :D \n";
  //           f.close();
  //         }
  //     }
  //   }
  //   //file<<"here biches !!\n";
  //   if (numActiveTactics > 0)
  //   {
  //     //file<<"can transit \n";
  //     return true;  // There is atleast 1 active tactic and all of them have completed hence can transit
  //   }
  //   else
  //   {
  //     // There are no active tactics in this iteration and hence all the tactics must be completed in order to transit
  //     //file<<"here 4\n";
  //     for (int roleID = 0; roleID < HomeTeam::SIZE; ++roleID)
  //     {
  //       std::string tID       = currPlay->roleList[roleID][currTacticIdx].first;
  //       Tactic*    selTactic = robot[roleID]->curTactic.get();
  //       if (selTactic->isCompleted(state)==false)
  //       {
  //         // If there is at least one incomplete tactic, then cannot transit
  //         //printf("can not transit 3 \n");
  //         return false;
  //       }
  //     }
  //   }
  //   //Util::Logger::toStdOut("Can Transit returning true.");
  //   //file<<"can transit \n";
  //   //file.close();
  //   return true;
  // } // canTransit

//############# Try transit #################

  // bool PExec::tryTransit(void)
  // {    
  //     if (currTacticIdx + 1 < playList[playID]->maxTacticsPerRole)
  //     {
  //       ++currTacticIdx;
  //       return true;
  //     }
  //   return false;
  // } // tryTransit
//############################################

  bool PExec::transit(krssg_ssl_msgs::BeliefState &bs)
  {
    fstream f;
    f.open("/home/gunjan/catkin_ws/src/play/playRunning.txt",fstream::out | fstream::app);
    //f<<"currTacticIdx : "<<currTacticIdx[0]<<","<<currTacticIdx[1]<<","<<currTacticIdx[2]<<","<<currTacticIdx[3]<<","<<currTacticIdx[4]<<","<<currTacticIdx[5]<<endl;
    if (playID == PlayBook::None)
    {
      return true;
    }
    Play* currPlay = playList[playID];    
    bool transition=false;
    for (int roleID = 0; roleID < HomeTeam::SIZE; roleID++)
    {
      std::string tID  = currPlay->roleList[roleID][currTacticIdx[roleID]].first;
      Tactic::Param tParam  = currPlay->roleList[roleID][currTacticIdx[roleID]].second;
      selTactic = TacticFactory::instance()->Create(tID, roleBotMapping[roleID]);
      
      f<<roleID<<","<<tID<<endl;

      if ((*selTactic).isActiveTactic()==true)
      {
          if ((*selTactic).isCompleted(bs,tParam)==true)
          {
            if(currTacticIdx[roleID]+1 < currPlay->roleList[roleID].size())
            {
              currTacticIdx[roleID]++;
              transition=true;
            }
          }
      }
      selTactic.reset();
    }

    //f<<"transition -- "<<transition<<endl;
     f.close();
     return transition;
  }

  Robot** PExec::selectPlay(krssg_ssl_msgs::BeliefState &bs)
  {
    select();
    fstream f;
    playResult = Play::NOT_TERMINATED;
    for (int i = 0; i < HomeTeam::SIZE; ++i)
    {
      currTacticIdx[i]=0;
    }
    assignRoles(bs);   
    return robot;
  } // selectPlay

  Robot** PExec::executePlay(krssg_ssl_msgs::BeliefState &bs)
  {
    //##########################TODO : use updateParams here for each play################################
    
    //ROS_INFO("cantransit :%d , tryTransit: %d",canTransit(),tryTransit());
    //if (canTransit() && tryTransit())
    // fstream f;
    // f.open("/home/gunjan/catkin_ws/src/play/playParams.txt",fstream::out|fstream::app);
    // Play* currPlay = playList[playID];
    // for (int roleIdx = 0; roleIdx < HomeTeam::SIZE; ++roleIdx) // Iterating over all roles
    // {
    //   if (currTacticIdx[roleIdx] < currPlay->roleList[roleIdx].size()) // Tactic exists for the current role iteration
    //   {
    //     std::string    tID     = currPlay->roleList[roleIdx][currTacticIdx[roleIdx]].first;
    //     Tactic::Param tParam  = currPlay->roleList[roleIdx][currTacticIdx[roleIdx]].second;
    //     if(tID.compare("TReceive")==0)
    //     {
    //       f<<"params --- "<<tParam.ReceiveP.x<<","<<tParam.ReceiveP.y<<endl;
    //     }
    //   }
    // }
    //f.close();
    printf("playID : %d \n" ,playID);
    playList[playID]->updateParam();
  	for(int i=0;i<HomeTeam::SIZE;i++)
  	{
  		Tactic::Param tParam = playList[playID]->roleList[i][currTacticIdx[i]].second;
  		robot[roleBotMapping[i]]->tParamJSON = TacticFactory::instance()->Create(robot[roleBotMapping[i]]->tID,roleBotMapping[i])->paramToJSON(tParam);
  	}
    if(transit(bs))
    {
      assignRoles(bs);
    }
    return robot;
  } // executePlay

  void PExec::evaluatePlay(void)
  {
    if (playID == PlayBook::None)
    {
      return;
    }
    
    updateWeights(playResult);
  } // evaluatePlay

  bool PExec::playTerminated(krssg_ssl_msgs::BeliefState &bs)
  {
    if (playID == PlayBook::None)
    {
      //Util::Logger::toStdOut("Last Play was None!\n");
      ROS_INFO("Last Play was None");
      return true;
    }
    if(playList[playID]->timedOut())
    {
      //Util::Logger::toStdOut("Play Timed out.\n");
      playResult = Play::TIMED_OUT;
      return true;
    }
    /* The completion of a play is defined to be the completion of all the tactics assigned
     * to all the bots. Since this information is available in the Tactic class and not known
     * to the Play class, the evaluation of the completion of the play is done here
     * instead of being done in the done() function of the individual plays
     */
    fstream f;
    f.open("/home/ssl/catkin_ws/src/plays/completed.txt",fstream::out | fstream::app);
    // f<<"play completed "<<playCompleted(bs)<<endl;
    f.close();
    if (playCompleted(bs))
    {
      playResult = Play::COMPLETED;
      return true;
    }

    Play::Result result = playList[playID]->done();
    if (result == Play::NOT_TERMINATED)
    {
      return false;
    }
    else
    {
      playResult = result;
      return true;
    }
  } // playTerminated

  bool PExec::playCompleted(krssg_ssl_msgs::BeliefState &bs)
  {
    Play* currPlay = playList[playID];
    for (int roleID = 0; roleID < HomeTeam::SIZE; ++roleID)
    {

      std::string tID  = currPlay->roleList[roleID][currTacticIdx[roleID]].first;
      Tactic::Param tParam  = currPlay->roleList[roleID][currTacticIdx[roleID]].second;
      selTactic = TacticFactory::instance()->Create(tID, roleBotMapping[roleID]);
      
      fstream f;
      f.open("/home/ssl/catkin_ws/src/plays/completed.txt",fstream::out | fstream::app);
      // f<<"roleBotMapped ID ="<<roleBotMapping[roleID]<<" size -- "<<currTacticIdx[roleID]<<" -- "<<currPlay->roleList[roleID].size()<<" -- is completed ="<<(*selTactic).isCompleted(bs,tParam)<<endl;
      f.close();

      if(currTacticIdx[roleID]<currPlay->roleList[roleID].size()-1 || (*selTactic).isCompleted(bs,tParam)==false)
        return false;

      selTactic.reset();
    }
    return true;
  }
} // namespace Strategy
