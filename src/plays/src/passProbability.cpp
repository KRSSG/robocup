
float receiveProbability(const krssg_ssl_msgs::BeliefState& state,int passer_id,int receiver_id,Vector2D<int> passPoint)
{
    float Prob_total,Prob_scoring,Prob_receiving;
    float pa1=1,pa2=1,pa3=1,pa4=1; 
    /*
      pa1=probability of receiving pass based on number of opponents close to ball
      pa2=probability of receiving pass based on opponents who can intercept the pass
      pa3=probability of receiving pass based on length of pass 
      pa4=probability of receiving pass based on location of passPoint
    */ 

      //calculating pa1
     {
        float temp=1.0000f;
        for (int id = 0; id < HomeTeam::SIZE; ++id)
        {
          Vector2D<int> ballPos(state.ballPos.x,state.ballPos.y);
          Vector2D<int> passerPos(passerPos.x,passerPos.y);
          Vector2D<int> receivePos(passPoint.x,passPoint.y);

          if(Vector2D<int>::dist(ballPos,passerPos)<4*BOT_RADIUS && fabs(Vector2D<int>::angle(passerPos,receivePos)-Vector2D<int>::angle(passerPos,Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y)))< PI/10 ) 
          {
            temp*=fabs(Vector2D<int>::angle(passerPos,receivePos)-Vector2D<int>::angle(passerPos,Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y)))/PI;
          }
        } 
        pa1=temp+0.01f;
      }
 
      //calculating pa2
 
      {
         Vector2D<int> passerPos(state.ballPos.x,state.ballPos.y);
         Vector2D<int> receiverPos(state.homePos[receiver_id].x,state.homePos[receiver_id].y);
         float dist1=Vector2D<int>::dist(passerPos,receiverPos); // distance between ball and receiver
         float dist2; // to store distance of interceptor from line joining passer and receiver
         float dist3; // to store distance between interceptor and passer
         float dist4; // to store distance of interceptor and passer along the line joining passer and receiver
         pa2=1;
         float temp;
         for(int id=0;id<HomeTeam::SIZE;++id)
         {
            Vector2D<int> interceptorPos(state.awayPos[id].x,state.awayPos[id].y);
            dist3=Vector2D<int>::dist(passerPos,interceptorPos);
            if(passerPos.x!=receiverPos.x)
            {
              double angle=atan((passerPos.y-receiverPos.y)/(passerPos.x-receiverPos.x));
              double c=passerPos.y-tan(angle)*passerPos.x;
              dist2=abs(interceptorPos.y-tan(angle)*interceptorPos.x-c)/sqrt(1+pow(tan(angle),2));
              dist4=sqrt(pow(dist3,2)-pow(dist2,2));
            }
            else
            {
              dist2=abs(passerPos.x-interceptorPos.x);
              dist4=sqrt(pow(dist3,2)-pow(dist2,2));
            }
           if(dist4>=dist1*1.5)
            temp=1;
           else if(!((interceptorPos.x-state.ballPos.x)*(receiverPos.x-state.ballPos.x)>0))
            temp=1;
          else   
            temp=dist2*MAX_BALL_SPEED/(MAX_BOT_SPEED*dist4*10.0);

         
           if(temp<pa2)
            pa2=temp; 
         }
      }


      //calculating pa3
     {
  
         Vector2D<int> passerPos(state.homePos[passer_id].x,state.homePos[passer_id].y);
         Vector2D<int> receiverPos(state.homePos[receiver_id].x,state.homePos[receiver_id].y);
         float dista=Vector2D<int>::dist(passerPos,receiverPos);
     
         pa3=exp(-3*pow(3.0*dista/(HALF_FIELD_MAXX*2)-1,2));
    }

    //calculating pa4
    {
    //CONSTANTS ARE SET FOR HALF_FIELD_MAXX=900 AND HALF_FIELD_MAXY=600
    double prx,pry,pconst=0.7;
    int Danger_Self_Goal=-7*HALF_FIELD_MAXX/8,p_thres=7*HALF_FIELD_MAXX/8;

    //FINDING PROBABILITY IN X DIRECTION
    if(passPoint.x<Danger_Self_Goal)
      prx=0;
    else if(passPoint.x<0)
      prx=pconst*erf((passPoint.x-Danger_Self_Goal)/10.0);
    else if(passPoint.x<p_thres)
      prx=pconst+0.3*erf((passPoint.x)/10.0);
    else if(passPoint.x<HALF_FIELD_MAXX)
      prx=1-erf(passPoint.x-10);
    else prx=0;

    //FINDING PROBABILITY IN Y DIRECTION
    if(passPoint.y<-HALF_FIELD_MAXY)
      pry=0;
    else if(passPoint.y<0)
      pry=erf((passPoint.y+HALF_FIELD_MAXY)/8000.0);
    else if(passPoint.y<HALF_FIELD_MAXY)
      pry=1-erf((passPoint.y)/8000.0);
    else pry=0;

    //COMBINING RESULTS
     pa4=sqrt(prx*pry)+0.001f;

  }
 
  Prob_receiving=pa1*pa2*pa3*pa4;
  return Prob_receiving;
}

float shootProbability(const krssg_ssl_msgs::BeliefState& state,int passer_id,int receiver_id,Vector2D<int> passPoint)
{
     float pb1=1,pb2=1,pb3=1;
  /*
    pb1=probability of shot reaching faster than goalkeeper 
    pb2=wide enough theta
    pb3=not enough time for defenders to block the shot
  */

    //calculating pb1
    {
      Vector2D<int> ballPos(state.ballPos.x,state.ballPos.y);
      Vector2D<int> passerPos(state.homePos[passer_id].x,state.homePos[passer_id].y);
      Vector2D<int> receivePos(passerPos.x,passerPos.y);
      Vector2D<int> oppGoaliePos(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y);
      float goalieReceiverDist=Vector2D<int>::dist(receivePos,Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y) );
      float goaliePoleDist;
      if(Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MAXY ))>Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MINY )))
      {
          goaliePoleDist=Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MAXY ));
      }
      else goaliePoleDist=Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MINY ));
      pb1=goaliePoleDist/goalieReceiverDist;
    }

    //calculating pb2
    {
      float maxX=1,maxY=1;
      Vector2D<int> GoalPole;
      float theta=fabs(Vector2D<int>::angle(Vector2D<int>(maxX,maxY),GoalPole)-Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y)));
      float max_possible_theta=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MAXY))-Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MINY))));
      pb2=theta/max_possible_theta;
    }

    //calculating pb3
    {
      float maxX=1,maxY=1;
      float angle_temp=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[0].x,state.awayPos[0].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY))));
      float dist_temp=Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0));
      float temp=dist_temp*angle_temp;
      for (int id = 0; id < HomeTeam::SIZE; ++id)
      {
        if(fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[id].x,state.awayPos[id].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY)))) * Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0)) > temp)
        {
          temp=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[id].x,state.awayPos[id].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY))) * Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0)));
        }
      }
      pb3=temp/=Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0));
    }
  float Prob_scoring=pb1*pb2*pb3;
  return Prob_scoring;
}