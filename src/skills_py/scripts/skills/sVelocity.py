import skill_node

def execute(param, state, bot_id):
	skill_node.send_command(state.isteamyellow, bot_id, param.VelocityP.v_x, param.VelocityP.v_y, param.VelocityP.v_t, 0, false)
