#!/usr/bin/env python
import rospy
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import gr_Commands


def send_command(pub, team, bot_id, v_x, v_y, v_w, kick_power, dribble, chip_power = 0):
    """ 
    Publish the command packet
    team : 'True' if the team is yellow 
    """
    gr_command = gr_Robot_Command()
    final_command = gr_Commands()
    
    """
    Set the command to each bot
    """
    gr_command.id          = bot_id
    gr_command.wheelsspeed = 0
    gr_command.veltangent  = v_x/1000
    gr_command.velnormal   = v_y/1000
    gr_command.velangular  = v_w
    gr_command.kickspeedx  = kick_power
    gr_command.kickspeedz  = chip_power
    gr_command.spinner     = dribble

    final_command.timestamp      = rospy.get_rostime().secs
    final_command.isteamyellow   = team
    final_command.robot_commands = gr_command

    
    def debug():
        """
        Log the commands
        """
        print 'botid: {}: [{}]\n'.format(bot_id, final_command.timestamp)
        print 'vel_x: {}\nvel_y: {}\nvel_w: {}\n'.format(v_x, v_y, v_w)
        print 'kick_power: {}\nchip_power: {}\ndribble_speed:{}\n\n'.format(kick_power, chip_power, dribble)
    
    pub.publish(final_command)

