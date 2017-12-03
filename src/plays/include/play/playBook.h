#ifndef PLAY_BOOK_H
#define PLAY_BOOK_H

#include "krssg_ssl_msgs/BeliefState.h"

// Forward Declarations
namespace Strategy
{
  class BeliefState;
  class Play;
}

namespace Strategy
{
  class PlayBook
  {
  public:
    enum PlayID
    {
      None,
      SetPosition, 
      PassTest, 
      TestPlay,
      // Referee Plays
      Halt,
      MAX_PLAYS
    };

  protected:
    const krssg_ssl_msgs::BeliefState& state;
    Play*              playList[MAX_PLAYS];

  public:
    PlayBook(const krssg_ssl_msgs::BeliefState& state);
    
    ~PlayBook();

    void reload(void);
  }; // class PlayBook
} // namespace Strategy

#endif // PLAY_BOOK_H