import sys
sys.path.insert(0,'./../')

from utils import tactics_union

import role_node_offence


offence_roles = role_node_offence.role_node()



params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
offence_roles.role_list[0][0] = "TAttackerX"
offence_roles.role_list[0][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
offence_roles.role_list[3][0] = "TReceiver"
offence_roles.role_list[3][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
offence_roles.role_list[1][0] = "TPrimaryDefender"
offence_roles.role_list[1][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
offence_roles.role_list[2][0] = "TMidFielderX"
offence_roles.role_list[2][1] = params

params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
offence_roles.role_list[4][0] = "TGoalie"
offence_roles.role_list[4][1] = params

print 'Initializing the node...'
offence_roles.node()
