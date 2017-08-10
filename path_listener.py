import rospy

vrtx = []
rgb = []

def debug_path(msg):
    rospy.longinfo("HEY! I AM IN debug_path")
    vrtx = []
    for v in msg.vertices:
        x = int(v.x)
        y = int(v.y)
        vrtx.append([x,y])
    
    rgb = []
    rgb.append(int(msg.rgb.r))
    rgb.append(int(msg.rgb.g))
    rgb.append(int(msg.rgb.b))


def path_listener():
    rospy.init_node('path_listener', anonymous=True)
    rospy.Subscriber("path_planner", String, debug_path)
    rospy.spin()


if __name__ == '__main__':
    path_listener()

