#ifndef TACTIC_H
#define TACTIC_H
#include <list>
// #include <cstring>
#include <string>
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/gr_Robot_Command.h>


using namespace std;
using krssg_ssl_msgs::BeliefState;
using krssg_ssl_msgs::gr_Robot_Command;
namespace Strategy
{
  class Tactic
  {
  public:
    union Param;
    inline Tactic(int botID):botID(botID) {}
    inline ~Tactic()
    {
    } // ~Tactic
    // True if tactic execution looks completed
    virtual bool isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const = 0;
    // True if the tactic is active, i.e., it involves ball manipulation
    virtual bool isActiveTactic() const = 0;
    // TODO As per the design, this function must be static but it cannot be both virtual as well as static
    /* This function assigns a score to all the free bots (bots that have not been
     * assigned any role yet) depending on how suitable they are in being assigned
     * the tactic. It then chooses the bot with the highest score and assigns it the tactic.
     */
    virtual int chooseBestBot(const BeliefState &bs, std::list<int>& freeBots, const Tactic::Param& tParam, int prevID = -1) const = 0;
    /* This function takes in the current tactic parameter for a robot and
     * using the belief state info and the skill transition rules,
     * it either decides to transit to another skill or continues to run
     * the current skill.
     */
    virtual gr_Robot_Command execute(const BeliefState &state, const Param& tParam) = 0;
    union Param
    {
      /* Union of parameters for each tactic enumerated in Tactic::ID
       * Creating object initializes each member by default to zero
       */
              
      // Parameters for tactic Goalie
      struct GoalieP
      { } GoalieP;

      // Parameters for tactic Clear
      struct ClearP
      { } ClearP;

      // Parameters for the tactic Block
      struct BlockP
      {
        float dist;   // Distance from the goal where the bot has to defend the goal
        int side;
      } BlockP;
      struct DefendLineP
      {
        int x1, x2, y1, y2;
    int radius;
      } DefendLineP;

      struct
      {
        bool intercept;
      } GoToBallP;
    
      struct PositionP
      {
        float x ;
        float y ;
        float finalSlope ;
        float finalVelocity ;
        bool align;
      } PositionP , PositionForStartP, PositionForPenaltyP;

      struct AttackerP
      {
        float x ;
        float y ;
        float finalSlope ;
        float finalVelocity ;
        bool align;
      } AttackerP;

      // Parameters for the tactic Stop
      struct StopP
      { } StopP;

      // Parameters for the tactic Velocity
    
      struct VelocityP
      {
        float v_x;
        float v_y;
        float v_t;
      } VelocityP;

      // Parameters for the tactic Kick
      struct KickP
      {
        float power;  // >= 0.0 and <= 1.0
      } KickP;
    
    struct DefendPointP
      {
        int x, y;
    int radius;
      } DefendPointP;
    
    struct DefendP
      {
        int x;
      } DefendP;
    
    struct MarkBotP
      {
        int awayBotID ;
      } MarkBotP;
      
      struct AttackSupportP
      {
        int id;
      } AttackSupportP;


      struct PassP
      {
        float x;
        float y;
      } PassToPointP,ReceiveP;
      
      struct DefendARc_left
      { } DefendARCP_left; 

      struct DefendARc_right
      { } DefendARCP_right;      
      
      struct InterecptP {
        int awayBotID;
        //where = 0 corresponds to intercepting the pass 
        //and where = 1 corresponds to intercepting the goal
        int where;
      } InterceptP;

      
      struct DribbleTurnPassP {
        int x;
        int y;
      } DribbleTurnPassP;

      Param() {
        /** Create a Parameter Object for Tactics
         * Initializes all the members initially to zero by default
         */
        memset(this, 0, sizeof(Param) );
      }
      // Param(Tactic::ID tacticID) {
      //   /** Create a Parameter Object for Tactics
      //    * Initializes all the members initially to zero by default
      //    */
      //   memset(this, 0, sizeof(Param) );
      //   switch(tacticID) {
      //     case Tactic::Block:
      //       this->BlockP.dist = 100;
      //       this->BlockP.side = 1;
      //       break;
      //     default:
      //       break;
      //   }
      // }
    };
    // again, need virtual static functions for each tactic that can parse JSON strings to create Tactic::Param objects, 
    // and vice versa. However can't make these both virtual and static
    virtual Param paramFromJSON(string json) = 0;
    virtual string paramToJSON(Param p) = 0;
  protected:
    int botID;
  }; // class Tactic

  // // base class for tactic param.
  // class TParam {
  // public:
  //   // function to initialize param from json string
  //   virtual void fromJSON(string json) = 0;
  //   // function that returns a string with has its params in JSON format
  //   virtual string toJSON() = 0;
  // };
} // namespace Strategy
#endif // TACTIC_H
