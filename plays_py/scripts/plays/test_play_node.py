import sys
sys.path.insert(0,'./../')

from utils import tactics_union
import role_node

test_roles = role_node.role_node()

"""
Edit the roles to test individually
"""

# Position all the bots at some random points
'''
params = tactics_union.Param()
params.PositionP.x = 764.987
params.PositionP.y = 764.987
params.PositionP.finalSlope = 0.0
params.PositionP.finalVelocity = 0.0
params.PositionP.align = False
test_roles.role_list[0][0] = "TPosition"
test_roles.role_list[0][1] = params

params = tactics_union.Param()
params.PositionP.x = 100.0
params.PositionP.y = 100.0
params.PositionP.finalSlope = 0.0
params.PositionP.finalVelocity = 0.0
params.PositionP.align = False
test_roles.role_list[1][0] = "TPosition"
test_roles.role_list[1][1] = params

params = tactics_union.Param()
params.PositionP.x = -100.0
params.PositionP.y = -100.0
params.PositionP.finalSlope = 0.0
params.PositionP.finalVelocity = 0.0
params.PositionP.align = False
test_roles.role_list[2][0] = "TPosition"
test_roles.role_list[2][1] = params

params = tactics_union.Param()
params.PositionP.x = 200.0
params.PositionP.y = 200.0
params.PositionP.finalSlope = 0.0
params.PositionP.finalVelocity = 0.0
params.PositionP.align = False
test_roles.role_list[3][0] = "TPosition"
test_roles.role_list[3][1] = params

params = tactics_union.Param()
params.PositionP.x = -200.0
params.PositionP.y = -200.0
params.PositionP.finalSlope = 0.0
params.PositionP.finalVelocity = 0.0
params.PositionP.align = False
test_roles.role_list[4][0] = "TPosition"
test_roles.role_list[4][1] = params

params = tactics_union.Param()
params.PositionP.x = 450.0
params.PositionP.y = 450.0
params.PositionP.finalSlope = 0.0
params.PositionP.finalVelocity = 0.0
params.PositionP.align = False
test_roles.role_list[5][0] = "TPosition"
test_roles.role_list[5][1] = params

'''
params = tactics_union.Param()
params.IntercptP.awayBotID = 3
params.IntercptP.where = 0
test_roles.role_list[0][0] = "TPosition"
test_roles.role_list[0][1] = params

# params = tactics_union.Param()
# params.IntercptP.awayBotID = 3
# params.IntercptP.where = 0
# test_roles.role_list[1][0] = "TReceiver"
# test_roles.role_list[1][1] = params


print 'Initializing the node...'
test_roles.node()
