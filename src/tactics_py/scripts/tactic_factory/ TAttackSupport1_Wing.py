



lass TPosition(Tactic):

    def __init__(self, bot_id, state, param=None):
        super(TPosition, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()

        attacker = Vector2D(state.homePos[attacker_ID].x , state.homePos[attacker_ID].y)
		botPos = Vector2D(state.homePos[botID].x , state.homePos[botID].y)
		ballPos = Vector2D(state.ballPos.x , state.ballPos.y)

		attacker_ID = self.param.AttackSupportP.id

		attacker = Vector2D(state.homePos[attacker_ID].x , state.homePos[attacker_ID].y)
		botPos = Vector2D(state.homePos[botID].x , state.homePos[botID].y)
		ballPos = Vector2D(state.ballPos.x , state.ballPos.y)

		target_x
		target_y
		side

		

#--------------------------------------Calculating the heavily defended side---------------------------------------------------
    
    	top_half = 0
		defended_side = 0

		for i in range(6):
			if(state.awayPos[i].x > 0):
				if(state.awayPos[i].y > 0):
					top_half++
				else if(state.awayPos[i].y < 0):
					top_half--
			

		if(top_half > 0)
			defended_side = 1
		else if(top_half < 0)
			defended_side = -1
		else
			defended_side = 0

#-------------------------------------------------------------------------------------------------------------------------------


#---------------------------------------------------------------Region Definition------------------------------------------------------------------------------
		region = 1

		if ballPos.x <= HALF_FIELD_MAXX/2 and fabs(ballPos.y) <= HALF_FIELD_MAXY/2:
			region = 1
		
		if ballPos.x <= 0: 
			region = 1
		
		if ballPos.x >= HALF_FIELD_MAXX/2 and fabs(ballPos.y) <= HALF_FIELD_MAXY/2:
			region = 2
		
		if ballPos.x >= HALF_FIELD_MAXX/2 and fabs(ballPos.y) >= HALF_FIELD_MAXY/2:
			region = 3


#---------------------------------------------------------------For Region 1--------------------------------------------------------------------------------------
		
		if region == 1:

			side = (-1) * defended_side

			if state.ballPos.x < 0:
				target_x = 0.25 * HALF_FIELD_MAXX - MARGIN_PASS_AHEAD
				target_y = 0.75 * HALF_FIELD_MAXY * side
			

			else if state.ballPos.x < (HALF_FIELD_MAXX / 3):
				target_x = (state.ballPos.x + HALF_FIELD_MAXX)/2 - MARGIN_PASS_AHEAD 

				target_y = 0.6 * HALF_FIELD_MAXY * side
			

		else:
			target_x = (state.ballPos.x + HALF_FIELD_MAXX)/2  - MARGIN_PASS_AHEAD

			target_y = (1- (state.ballPos.x / HALF_FIELD_MAXX)) * HALF_FIELD_MAXY * side

			if fabs(target_y) < DBOX_HEIGHT/1.3:
				target_y = DBOX_HEIGHT/1.3 * side
			else if fabs(target_y) > 0.6* HALF_FIELD_MAXY:
				target_y = target_y = 0.6 * HALF_FIELD_MAXY * side


		    target = Vector2D(target_x , target_y)
			theta = normalizeAngle(ballPos.angle(target)) 
			our_distance_from_line = 999999
			opp_distance_from_line = 999999

			for id in range(6):
				opp_test = Vector2D(state.awayPos[id].x , state.awayPos[id].y)
				home_test = Vector2D(state.homePos[id].x , state.homePos[id].y)

				v = Vector2D()
				opp_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.awayPos[id].x)) - ((target_x - state.ballPos.x) * (state.awayPos[id].y)) + (target_x * state.ballPos.y) - (target_y * state.ballPos.x)) / math.sqrt(v.absSq((botPos - ballPos)))

				if( v.absSq((ballPos - opp_test)) > v.absSq((ballPos - target)) or v.absSq((botPos - opp_test)) > v.absSq((ballPos - target)) ):
					opp_distance_from_line = 999999

				our_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.homePos[id].x)) - ((target_x - state.ballPos.x) * (state.homePos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / math.sqrt(v.absSq((botPos - ballPos)))

				if( v.absSq((ballPos - home_test)) > v.absSq((ballPos - target) or v.absSq((botPos - home_test)) > v.absSq((botPos - ballPos)) or id == attacker_ID or id == botID):
					our_distance_from_line = 999999

				if ( opp_distance_from_line < 100 or our_distance_from_line < 60):

					target_x += 4 * BOT_RADIUS
					target_y -= 4 * BOT_RADIUS * side

					if(target_x >= HALF_FIELD_MAXX):
						target_x -= 8 * BOT_RADIUS

				self.sParam.GoToPointP.x = target_x
				self.sParam.GoToPointP.y = target_y
				self.sParam.GoToPointP.finalVelocity = 0

				final_target = Vector2D(target_x , target_y)

				angleToTurn = v.normalizeAngle(ballPos.angle(final_target))

				sParam.GoToPointP.finalslope = angleToTurn

				sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
			    #ASK


		if region == 2:
			v = Vector2D()
			if ballPos.y > 0:
				side = 1
			else:
				side = -1

			if ballPos.x < HALF_FIELD_MAXX/4:
				target_y = HALF_FIELD_MAXY * side / 1.2
			else:
				target_y = side * (HALF_FIELD_MAXY * (HALF_FIELD_MAXX - ballPos.x) / HALF_FIELD_MAXX)


			if fabs(target_y) > HALF_FIELD_MAXY / 1.2:
				target_y = HALF_FIELD_MAXY * side / 1.2
			elif fabs(target_y) < DBOX_HEIGHT/1.2:
				target_y = DBOX_HEIGHT / 1.2

			target_x = HALF_FIELD_MAXX - ballPos.x/4

			if target_x > (HALF_FIELD_MAXX - DBOX_WIDTH/2):
				target_x = HALF_FIELD_MAXX - DBOX_WIDTH/2
			elif target_x < HALF_FIELD_MAXX / 1.5:
				target_x = HALF_FIELD_MAXX / 1.5


			target = Vector2D( target_x , target_y)
			theta = v.normalizeAngle(ballPos.angle(target))
			our_distance_from_line = 999999
			opp_distance_from_line = 999999


			for id in range(6):
				opp_test = Vector2D(state.awayPos[id].x , state.awayPos[id].y)
				home_test = Vector2D(state.homePos[id].x , state.homePos[id].y)

				opp_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.awayPos[id].x)) - ((target_x - state.ballPos.x) * (state.awayPos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / math.sqrt(v.absSq((botPos - ballPos)))
				if( v.absSq((ballPos - opp_test)) > v.absSq((ballPos - target)) or v.absSq((botPos - opp_test)) > (v.absSq(ballPos - target))):
					opp_distance_from_line = 999999
				our_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.homePos[id].x)) - ((target_x - state.ballPos.x) * (state.homePos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / sqrt(v.absSq(botPos - ballPos))
				if( v.absSq((ballPos - home_test)) > v.absSq((ballPos - target)) or v.absSq((botPos - home_test)) > v.absSq((ballPos - target)) or id == attacker_ID or id == bot_id):
					our_distance_from_line = 999999
				if ( opp_distance_from_line < 100 or our_distance_from_line < 60):

					target_x -= 4 * BOT_RADIUS
					target_y -= 4 * BOT_RADIUS * side

					if(target_x >= HALF_FIELD_MAXX):
						target_x -= 8 * BOT_RADIUS
				
				if(fabs(ballPos.x) > (HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH))):
					target_x = HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH)
				else
					target_x = HALF_FIELD_MAXX - DBOX_WIDTH / 1.5


				self.sParam.GoToPointP.x = target_x
				self.sParam.GoToPointP.y = target_y
				self.sParam.GoToPointP.finalslope = 0

				final_target = Vector2D(target_x, target_y)
				angleToTurn = v.normalizeAngle(ballPos.angle(final_target))

				self.sParam.GoToPointP.finalslope = angleToTurn

				sGoToPoint.execut(self.sParam, state, self.bot_id, pub)
				#ASK

		#---------------------------------------------------------------For Region 3--------------------------------------------------------------------------------------

		if region == 3:
			v =Vector2D()
			if ballPos.y > 0:
				side = -1
			else:
				side = 1

			if(fabs(ballPos.y) > DBOX_HEIGHT/1.3):
				if( ballPos.x > (HALF_FIELD_MAXX - DBOX_WIDTH)):
					target_x = HALF_FIELD_MAXX - DBOX_WIDTH
				else
					target_x = HALF_FIELD_MAXX - DBOX_WIDTH / 1.5

				target_y = -1 * ballPos.y

				if(fabs(target_y) > HALF_FIELD_MAXY / 3):
					target_y = HALF_FIELD_MAXY * side / 3
				elif fabs(target_y) < DBOX_HEIGHT/1.2:
					target_y = DBOX_HEIGHT * side /1.2

			else:
				target_x = HALF_FIELD_MAXX - DBOX_WIDTH

				target_y = DBOX_HEIGHT * side

			target = Vector2D(target_x, target_y)
			theta = V.normalizeAngle(ballPos.angle(target))

			for id in range(6):
				opp_test = Vector2D(state.awayPos[id].x , state.awayPos[id].y)
			    home_test = Vector2D(state.homePos[id].x , state.homePos[id].y)

			    opp_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.awayPos[id].x)) - ((target_x - state.ballPos.x) * (state.awayPos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / math.sqrt(v.absSq((botPos - ballPos)))

				if( v.absSq((ballPos - opp_test)) > v.absSq((ballPos - target)) or v.absSq((botPos - opp_test)) > v.absSq((ballPos - target)) ):
					opp_distance_from_line = 999999

				our_distance_from_line = fabs(((target_y - state.ballPos.y) * (state.homePos[id].x)) - ((target_x - state.ballPos.x) * (state.homePos[id].y)) + (target_x* state.ballPos.y) - (target_y * state.ballPos.x)) / math.sqrt(v.absSq((botPos - ballPos))):

				if( v.absSq((ballPos - home_test)) > v.absSq((ballPos - target)) or v.absSq((botPos - home_test)) > v.absSq((ballPos - target)) or id == attacker_ID or id == botID):
					our_distance_from_line = 999999

				if ( opp_distance_from_line < 180 or our_distance_from_line < 150)

					target_x -= 4 * BOT_RADIUS
					target_y += 4 * BOT_RADIUS * side

					if target_x >= HALF_FIELD_MAXX:
						target_x -= 8 * BOT_RADIUS

				self.sParam.GoToPointP.x = target_x;
				self.sParam.GoToPointP.y = target_y;
				self.sParam.GoToPointP.finalVelocity = 0;

				final_target = Vector2D( target_x , target_y)
				angleToTurn = v.normalizeAngle(ballPos.angle(final_target))

				self.Param.GoToPointP.finalslope = angleToTurn

				sGoToPoint.execute(self.sParam, state, self.bot_id, pub)

			if(fabs(ballPos.x) > (HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH) )):
				target_x = HALF_FIELD_MAXX - (1.5 * DBOX_WIDTH);
			else:
				target_x = HALF_FIELD_MAXX - DBOX_WIDTH / 1.5;


			self.sParam.GoToPointP.x = target_x

			self.sParam.GoToPointP.y = target_y

			self.sParam.GoToPointP.finalVelocity = 0

			final_target = Vector2D( target_x , target_y)

			theta = v.normalizeAngle(ballPos.angle(final_target))
			self.sParam.GoToPointP.finalslope = theta

			sGoToPoint.execute(self.sParam, state, self.bot_id, pub)

			









		































			


		