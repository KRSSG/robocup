// DO NOT EDIT: This file is auto-generated

#include <cstdlib>
#include "playBook.h"
#include "play.hpp"
#include "pNone.hpp"
#include "pSetPosition.hpp"
#include "pPassTest.hpp"
#include "pTestPlay.hpp"
#include "refPlays.hpp"

namespace Strategy
{
  PlayBook::PlayBook(const krssg_ssl_msgs::BeliefState& state) : 
    state(state)
  {
    reload();
  } // PlayBook

  PlayBook::~PlayBook()
  { } // ~PlayBook

  void PlayBook::reload(void)
  {
    // Initializing all play objects with NULL
    for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
    {
      playList[pID] = NULL;
    }

    // Inserting all the name-object pairs into the  Play List
    playList[PlayBook::None]                  = new PNone(state);
    playList[PlayBook::SetPosition]           = new PSetPosition(state);
    playList[PlayBook::TestPlay]           = new PTestPlay(state);
    playList[PlayBook::PassTest]           = new PPassTest(state);

    //Referee Plays
    playList[PlayBook::Halt]              =   new HaltRP(state);
    //playList[PlayBook::OffensePlay]           = new POffensePlay(state); 
    //playList[PlayBook::DefensiveGoToGoal]     = new PDefensiveGoToGoal(state);
    
    // Checking if all the play ids have been associated with their corresponding Play objects
    for (int pID=0; pID<PlayBook::MAX_PLAYS; ++pID)
    {
      assert(playList[pID] != NULL); // Play has not been assigned a Play object
    }
  }
}
