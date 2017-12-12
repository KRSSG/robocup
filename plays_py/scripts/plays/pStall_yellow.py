import sys
sys.path.insert(0,'./../')

from utils import tactics_union

import role_node_stall_yellow


stall_roles = role_node_stall_yellow.role_node()



params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
stall_roles.role_list[0][0] = "TAttacker"
stall_roles.role_list[0][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
stall_roles.role_list[1][0] = "TPrimaryDefender"
stall_roles.role_list[1][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
stall_roles.role_list[2][0] = "TMiddleDefender"
stall_roles.role_list[2][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
stall_roles.role_list[3][0] = "TForward"
stall_roles.role_list[3][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
stall_roles.role_list[4][0] = "TGoalie"
stall_roles.role_list[4][1] = params

print 'Initializing the node...'
stall_roles.node()
