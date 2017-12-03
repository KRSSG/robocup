// It is through the skill layer that the Strategy layer talks to the Simulator/Actual robots

#ifndef SKILLS_H
#define SKILLS_H
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/gr_Robot_Command.h>
#include <stdio.h>
// Forward Declarations
template <class T> class Vector2D;

using namespace krssg_ssl_msgs;
namespace Strategy
{
  /* Procedure for adding a skill successfully to the SkillSet is:
   * 1. Enumarate it in enum SkillSet::SkillID just before MAX_SKILLS
   * 2. Identify its input parameters and define a struct for it in union SkillSet::SParam
   * 3. Declare the function prototype for the skill.
   * 4. Map the Skill ID to the corresponding function in the constructor of SkillSet
   * 5. Define the function that implements the skill declared in step 3.
   * 6. Add the .cpp file for the new skill in the CMakeLists variable 'skillsSrc'
   * NOTE: Variable names of enums and corresponding skill function names must be the same,
   *       except that the enum name starts with a capital letter. Name of the corresponding
   *       struct should be the same as that of enum with a trailing 'P' character
   */
   // SkillSet will be a singleton class with only 1 instance.
  class SkillSet
  {
  public:
    static SkillSet *instance();
    enum SkillID
    {

      Spin,
      Kick,
      Stop,
      Dribble,
      Velocity,
      GoToBall, //Add comm.addLine & comm.addCircle, logger,timer
      GoToPoint,
      DefendPoint,  //comm.addcircle
      DribbleToPoint, //comm.addCircle,logger,Util
      ReceiveBall, //to be written
      GoalKeeping,  //ballDestination uninitialized
      KickToPoint,
      TurnToPoint,  //addCircle
      TurnToAngle,
      DribbleTurn,  //Dribble while turning

      MAX_SKILLS,
    };

    // Union of parameters for each skill enumerated in SkillID
    union SParam
    {
      // Parameters for the skill Spin
      struct
      {
        float radPerSec;
      } SpinP;

      // Parameters for the skill Stop
      struct
      {} StopP;

      struct
      {
        bool intercept;
      } GoToBallP;

      // Parameters for the skill Kick
      struct
      {
        float power;
      } KickP;
      //Parameters for the skill KickToPoint
      struct
      {
        float power;
        float x;
        float y;
      } KickToPointP;
      // Parameters for the skill Velocity
      struct
      {
        float v_x;
        float v_y;
        float v_t;
      } VelocityP;

      //Parameters for skill goToPoint and dribble to point
      struct
      {
        float x;
        float y;
        float finalslope;
        float radius;
      }  DefendPointP, DribbleToPointP, GoalKeepingP, TurnToAngleP;
      struct
      {
        float x;
        float y;
        float max_omega;
      } TurnToPointP;
      struct
      {
        float x;
        float y;
        float max_omega;
        float turn_radius;
      } DribbleTurnP;
      struct
      {
        float x;
        float y;
        float finalslope;
        bool align;
        float finalVelocity;
      } GoToPointP;
    };


  protected:
    typedef gr_Robot_Command (SkillSet::*skillFncPtr)(const SParam&, const BeliefState&, int );

    skillFncPtr        skillList[MAX_SKILLS];
  public:
    //------- List of robot skills -------//
    gr_Robot_Command spin(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command kick(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command stop(const SParam& param, const BeliefState &state, int botID);

    gr_Robot_Command dribble(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command velocity(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command goToBall(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command goToPoint(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command defendPoint(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command dribbleToPoint(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command receiveBall(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command goalKeeping(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command kickToPoint(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command turnToPoint(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command turnToAngle(const SParam& param, const BeliefState &state, int botID);
    gr_Robot_Command dribbleTurn(const SParam &param, const BeliefState &state, int botID);

  private:
    SkillSet(); // private so can't be called.
    SkillSet(SkillSet const&) {}; // copy constructor is private.

    static SkillSet* m_pInstance; // pointer to the instance.
  public:
    // Executing the skill function corresponding to the given ID and param function parameters
    inline gr_Robot_Command executeSkill(SkillID ID, SParam& param, const BeliefState &state, int botID)
    {
      return (*this.*skillList[ID])(param, state, botID);
    } // executeSkill
  }; // class SkillSet
} // namespace Strategy

#endif // SKILLS_H
