import sys
sys.path.insert(0,'./../')

from utils import tactics_union

import role_node_defend

defend_roles = role_node_defend.role_node()



params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
defend_roles.role_list[0][0] = "TMark"
defend_roles.role_list[0][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
defend_roles.role_list[1][0] = "TPressureCooker"
defend_roles.role_list[1][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
defend_roles.role_list[2][0] = "TPrimaryDefender"
defend_roles.role_list[2][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
defend_roles.role_list[4][0] = "TGoalie"
defend_roles.role_list[4][1] = params

print 'Initializing the node...'
defend_roles.node()