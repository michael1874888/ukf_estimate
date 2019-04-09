import rospy
import collections
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

window_length = 9
angles = []
angle_filt = 3.1415926/2
box2 = 0
#rospy.init_node('angle_filter', anonymous=True)
#rate = rospy.Rate(50)

def callback(box):
    global angle_filt, box2
    #print('angles1')
    #print(angles)
    if box.data[0] != -1:
        if abs(angle_filt - box.data[6]) > 3.1415926:
            angle_sub = 2*3.1415926 - abs(angle_filt - box.data[6])
        else:
            angle_sub = abs(angle_filt - box.data[6])
            
        if angle_sub < 3.1415926/4:
            angles.append(box.data[6])
        #print('angles2')
        #print(angles)
        angle_sum = 0
        if len(angles) >= window_length:
            if len(angles) > window_length:
                angles.pop(0)
            angles_sort = angles[:]
            #print('angles3')
            #print(angles)
            angles_sort.sort()
            #print('angles_sort')
            #print(angles_sort)
            angle_filt = angles_sort[(window_length-1)/2]
            #print('angle_filt')
            #print(angle_filt)
        else:
            angle_filt = box.data[6]
            
        box2 = box.data[6]
        #rate.sleep()
    
def listener():
    
    rospy.init_node('angle_filter', anonymous=True)

    rospy.Subscriber("YOLO/box", Float32MultiArray, callback)
    
    pub = rospy.Publisher('car_angle_med', Float64, queue_size=1)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        print('angle_filt')
        print(angle_filt/3.1415926*180)
        print('box_data')
        print(box2/3.1415926*180)
        pub.publish(angle_filt)
        rate.sleep()

if __name__ == '__main__':
    listener()
