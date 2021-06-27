import cv2
import numpy as np 
from PIL import Image 
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import math
from scipy.spatial.transform import Rotation as Rot
from cola2_msgs.msg import Setpoints, DVL
from std_msgs.msg import String,Float64MultiArray,MultiArrayLayout,MultiArrayDimension,Float64
import random
#import cola2_msgs.msg
#import ros
br=CvBridge()

setpoints=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
yaw_r=[0,0,0,0,-0.4,0,-0.25,-0.25]
yaw_l=[0,0,0,0,0,-0.4,0.25,0.25]
up=[0,0,0,0,1,1,0,0]
down=[0,0,0,0,-1,-1,0,0]
front=[1,1,1,1,0,0,0,0]
back=[-1,-1,-1,-1,0,0,0,0]

count=0
fase=0
ss
init_search_time=float()
back1=float()
pitch2=float()
pitch1=float()
front1=float()
zmm1=float()
zm1=float()


pub_thrusters_ =rospy.Publisher('/iris/controller/thruster_setpoints', Setpoints, queue_size=1)
def main():
    rospy.init_node('iris_img_viewer', anonymous=True)
    while not rospy.is_shutdown():
        rospy.Subscriber("/pose_publisher", Pose, motor_controller_pose)
        rospy.Subscriber('/iris/diretionions_publisher', Float64MultiArray, motor_controller_by_leds)
        rospy.spin()


def motor_controller_pose(pose):
    global fase
    global count
    count=count+1
    fase=2
    porportional_control=[0.4, -0.4, 0.2] #x_pointer*p_xp,z*p_z, front*distance*p_distance (p_xp,p_z,p_distance)
    distance=math.sqrt(pose.Point.y**2+pose.Point.x**2+pose.Point.z**2)
    x_pointer_input=porportional_control[0]*pose.Quaternion.w
    z_input=porportional_control[1]*pose.Point.z
    front_input=porportional_control[2]*distance
    total_rotor_input=motor_encoder_trust(x_pointer_input,z_input,front_input)
    #publicar o total_rotor_input
    publish_to_rotors(total_rotor_input)

def motor_encoder_trust(x_pointer_input,z_input,front_input):
    bigguer_rotor_value=0
    if x_pointer_input>0:
        x_pointer_input_rotor=yaw_r*x_pointer_input
    else:
        x_pointer_input_rotor=yaw_l*(-x_pointer_input)
    z_input_rotor=up*z_input
    front_input_rotor=front*front_input
    total_rotor_input=x_pointer_input_rotor+z_input_rotor+front_input_rotor
    for i in range(0,8):
        if total_rotor_input[i]>bigguer_rotor_value:
            bigguer_rotor_value=total_rotor_input[i]
    if bigguer_rotor_value > 1:
        scale_rotor_input=1/bigguer_rotor_value
        total_rotor_input=total_rotor_input*scale_rotor_input
    return total_rotor_input


def motor_controller_by_leds(control_array):
    global fase
    global count
    count=count+1
    if control_array[1]==0:
        fase=2
        ajuste_pose(control_array)
    else:
        if control_array[1]==1:
            fase=2
            slow_final_aproach(control_array)
        else:
            blind_ajuste_pose(control_array)


def slow_final_aproach(control_array):
    global fase
    porportional_control=[0.8,-0.3, 0.3]       # (x_point, alt, distance)   o da distancia n e na realidade com base na distancia mas sim no tamanho do led avistado
    led_size_parked=200
    if control_array[3]>led_size_parked:
        fase=3 ##acabou_com_sucesso_a_manobra
        publish_to_rotors([0,0,0,0,0,0,0,0])
    else:
        x_pointer_input=porportional_control[0]*control_array[1]
        z_input=porportional_control[1]*control_array[2]
        distance_factor=1/led_size_parked*control_array[3]
        front_input=porportional_control[2]*distance_factor
        total_rotor_input=motor_encoder_trust(x_pointer_input,z_input,front_input)
        #publicar o total_rotor_input
        publish_to_rotors(total_rotor_input)


def ajuste_pose(control_array):
    led_size_parked=200
    porportional_control=[0.4,0.4, 0.5]
    x_point_array=0
    z_array=0
    for i in range(1,5):
        if control_array[i]==0:
            if i==1:
                z_array=z_array+1
            if i==2:
                z_array=z_array-1
            if i==3:
                x_point_array=x_point_array+1
            if i==4:
                x_point_array=x_point_array-1
    x_pointer_input=porportional_control[0]*x_point_array
    z_input=porportional_control[1]*z_array
    distance_factor=1/led_size_parked*control_array[5]
    front_input=porportional_control[2]*distance_factor
    total_rotor_input=motor_encoder_trust(x_pointer_input,z_input,front_input)
    #publicar o total_rotor_input
    publish_to_rotors(total_rotor_input)


def blind_ajuste_pose(control_array):
    global fase
    global init_search_time
    if control_array[1]+control_array[2]!=0:
        fase=2
        center_led_viewed(control_array)
    else:
        if fase!=1:
            fase=1
            time_ros=ros.Time.now()
            init_search_time=time_ros.sec+time_ros.nsec*10**(-9)
            define_rout()
            follow_rout_defined(0)
        else:
            time_ros=ros.Time.now()
            search_time=time_ros.sec+time_ros.nsec*10**(-9)-init_search_time
            follow_rout_defined(search_time)


def define_rout():
    global pitch1
    global zm1
    global zmm1
    global front1
    global back1
    global pitch2
    pitch1=[[0,2][4,6][10,12][18,19][25,30]]
    zm1=[[2,4][12,14]]
    zmm1=[[6,10]]
    front1=[[14,18][19,25][30,35]]


def follow_rout_defined(search_time):
    total_rotor_input=np.zeros(8)
    trust_defined=0
    if len(pitch1)!=0:
        for i in range (0,len(pitch1)):
            if search_time >= pitch1[i][0] and pitch1[i][1] > search_time:
                total_rotor_input=yaw_r
                trust_defined=1
    if len(zm1)!=0:
        for i in range (0,len(zm1)):
            if search_time >= zm1[i][0] and zm1[i][1] > search_time:
                total_rotor_input=up
                trust_defined=1
    if len(zmm1)!=0:
        for i in range (0,len(zmm1)):
            if search_time >= zmm1[i][0] and zmm1[i][1] > search_time:
                total_rotor_input=down
                trust_defined=1
    if len(front1)!=0:
        for i in range (0,len(front1)):
            if search_time >= front1[i][0] and front1[i][1] > search_time:
                total_rotor_input=front
                trust_defined=1
    if len(back1)!=0:
        for i in range (0,len(back1)):
            if search_time >= back1[i][0] and back1[i][1] > search_time:
                total_rotor_input=back
                trust_defined=1
    if len(pitch2)!=0:
        for i in range (0,len(pitch2)):
            if search_time >= pitch2[i][0] and pitch2[i][1] > search_time:
                total_rotor_input=back
                trust_defined=1
    if trust_defined==0:
        rand_front=random.uniform(0, 1)
        rand_right=random.uniform(0, 1)
        total_rotor_input=rand_front*front+rand_right*yaw_r
    for i in range(0,8):
        if total_rotor_input[i]>bigguer_rotor_value:
            bigguer_rotor_value=total_rotor_input[i]
    if bigguer_rotor_value > 1:
        scale_rotor_input=1/bigguer_rotor_value
        total_rotor_input=total_rotor_input*scale_rotor_input
    #publicar o total_rotor_input
    publish_to_rotors(total_rotor_input)
    




def center_led_viewed(control_array):
    porportional_control=[0.5,0.4, 0.3]
    x_point_array=control_array(4)
    z_array=control_array(5)
    x_pointer_input=porportional_control[0]*x_point_array
    z_input=porportional_control[1]*z_array
    if control_array(3) > 100:
        front_input=-0.5
    else:
        distance_factor=1/200*control_array[3]
        front_input=porportional_control[2]*distance_factor
    total_rotor_input=motor_encoder_trust(x_pointer_input,z_input,front_input)
    #publicar o total_rotor_input
    publish_to_rotors(total_rotor_input)




def publish_to_rotors(total_rotor_input):
    msg_setpoimsg_setpointsnt=Setpoints()
    msg_setpoints.header.seq = count
    msg_setpoints.header.stamp=ros.Time.now()
    msg_setpoints.header.frame_id = "/iris/base_link"
    msg_setpoints.setpoints = total_rotor_input
    pub_thrusters_.publish(msg_setpoints)




#############################################################################################################
        
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass