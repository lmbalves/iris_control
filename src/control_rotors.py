import cv2
import numpy as np 
from PIL import Image 
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import math
# from scipy.spatial.transform import Rotation as Rot
from cola2_msgs.msg import Setpoints, DVL
from std_msgs.msg import String,Float64MultiArray,MultiArrayLayout,MultiArrayDimension,Float64
import random

br=CvBridge()
max_trust=[1,1,1,1,1,1,0.5,0.5]
setpoints=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
#yaw_r=[0,0,0,0,-0.4,0,-0.25,-0.25]
#yaw_l=[0,0,0,0,0,-0.4,0.25,0.25]
yaw_l=[0,0,0,0,-0.4,0,-0.25,-0.25]
yaw_r=[0,0,0,0,0,-0.4,0.25,0.25]

up=[0,0,0,0,1,1,0,0]
down=[0,0,0,0,-1,-1,0,0]

front=[1,1,1,1,0,0,0,0]
back=[-1,-1,-1,-1,0,0,0,0]

drag_right=[0,0,0,0,-0.15,-0.3,-0.5,0.4]
drag_left=[0,0,0,0,-0.3,-0.15,0.5,-0.4]
#last_pose=pose()    #futuramente para aceitar ou n a pose
#last_pose_time=0.0  #futuramente para aceitar ou n a pose
finish_sec=0
count=0
fase=0
class time_motors:
    def __init__(self,bai,bao,p1i,p1o,p2i,p2o,fri,fro,zmmi,zmmo,zmi,zmo):
        self.back1_in=bai
        self.back1_out=bao
        self.pitch1_in=p1i
        self.pitch1_out=p1o
        self.pitch2_in=p2i
        self.pitch2_out=p2o
        self.front1_in=fri
        self.front1_out=fro
        self.zmm1_in=zmmi
        self.zmm1_out=zmmo
        self.zm1_in=zmi
        self.zm1_out=zmo
    pass
time_motor_set=time_motors([-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1])
#print(time_motor_set.back1_in)

class time_def:  
	def __init__(self,secs,nsecs):
		self.secs=secs
		self.nsecs=nsecs
	pass

procura_dock=1
pub_thrusters_ =rospy.Publisher('/iris/controller/thruster_setpoints', Setpoints, queue_size=1)
def main():
    global procura_dock
    k=0
    
    rospy.init_node('iris_img_viewer', anonymous=True)
    while not rospy.is_shutdown():
        if fase==3:
            procura_dock=0
            k=termination_stage(k)

        if procura_dock==1:
            rospy.Subscriber("/iris/pose_publisher", Pose, motor_controller_pose)
            rospy.Subscriber('/iris/diretionions_publisher', Float64MultiArray, motor_controller_by_leds)
            rospy.spin()

def termination_stage(k):
    global finish_sec
    seconds = rospy.get_time()
    pass_sec=seconds-finish_sec
    if pass_sec > 4 and k==0:
        print('real finish')
        publish_to_rotors([0,0,0,0,0,0,0,0])
        k=1
    else:
        if pass_sec > 2:
            publish_to_rotors([0,0,0,0,0.5,0.5,0,0])
        else:
            publish_to_rotors([0.5,0.5,0.5,0.5,0,0,0,0])
    return k


def motor_controller_pose(pose):
    global fase
    global count
    count=count+1
    fase=2
    follow=pose_rejecter(pose)
    if(follow==1):
        w=pose.orientation.w
        y=pose.position.y
        x=pose.position.x
        porportional_control=[10, -3 , 0.2] #x_pointer*p_xp,z*p_z, front*distance*p_distance (p_xp,p_z,p_distance)
        distance=math.sqrt(pose.position.y**2+pose.position.x**2+pose.position.z**2)
        beta=float(math.asin((2*x)/math.sqrt((2*x)**2+y**2))-math.asin((x-w)/math.sqrt((x-w)**2+y**2)))
        x_pointer_factor=(beta/math.sqrt(distance))
        x_pointer_input=porportional_control[0]*beta
        z_input=porportional_control[1]*pose.position.z
        front_input=porportional_control[2]*distance
        if distance >8:
            factor=1
        else:
            factor=distance/8
        total_rotor_input=motor_encoder_trust(x_pointer_input,z_input,front_input)*factor
        #publicar o total_rotor_input
        print('pose')
    else:
        total_rotor_input=[0.25,0.25,0.2,0.2,0,0,0,0]
    publish_to_rotors(total_rotor_input)

def pose_rejecter(pose):
    distance=math.sqrt(pose.position.y**2+pose.position.x**2+pose.position.z**2)
    if 1>pose.position.y and 1>pose.position.x:
        aceppt=0
    else:
        aceppt=1
    return aceppt

def motor_encoder_trust(x_pointer_input,z_input,front_input):
    bigguer_rotor_value=0
    if x_pointer_input>0:
        x_pointer_input_rotor=multiply(yaw_r,x_pointer_input)
    else:
        x_pointer_input_rotor=multiply(yaw_l,(abs(x_pointer_input)))
    z_input_rotor=multiply(up,z_input)
    front_input_rotor=multiply(front,front_input)
    #print(front_input_rotor)
    total_rotor_input=x_pointer_input_rotor+z_input_rotor+front_input_rotor
    total_rotor_input_abs=abs(total_rotor_input)

    scale_rotor=1
    for i in range(0,8):
        if total_rotor_input[i]>max_trust[i]:
            scale_rotor_input=max_trust[i]/total_rotor_input[i]
            if scale_rotor>scale_rotor_input:
                scale_rotor=scale_rotor_input
    for i in range(0,8):
        if total_rotor_input_abs[i]>max_trust[i]:
            scale_rotor_input=max_trust[i]/total_rotor_input_abs[i]
            if scale_rotor>scale_rotor_input:
                scale_rotor=scale_rotor_input
    total_rotor_input=total_rotor_input*scale_rotor

    #for i in range(0,8):
    #    if total_rotor_input_abs[i]>bigguer_rotor_value:
    #        bigguer_rotor_value=total_rotor_input_abs[i]
    #if bigguer_rotor_value > 1:
    #    scale_rotor_input=1/bigguer_rotor_value
    #    total_rotor_input=total_rotor_input*scale_rotor_input
    return total_rotor_input

def motor_encoder_trust2(x_pointer_input,z_input,drag_input):
    bigguer_rotor_value=0
    if x_pointer_input>0:
        x_pointer_input_rotor=multiply(yaw_r,x_pointer_input)
    else:
        x_pointer_input_rotor=multiply(yaw_l,(abs(x_pointer_input)))

    z_input_rotor=multiply(up,z_input)

    if drag_input>0:
        drag_input_rotor=multiply(drag_right,drag_input)
    else:
        drag_input_rotor=multiply(drag_left,(abs(drag_input)))

    total_rotor_input=x_pointer_input_rotor+z_input_rotor+drag_input_rotor
    total_rotor_input_abs=abs(total_rotor_input)

    scale_rotor=1
    for i in range(0,8):
        if total_rotor_input[i]>max_trust[i]:
            scale_rotor_input=max_trust[i]/total_rotor_input[i]
            if scale_rotor>scale_rotor_input:
                scale_rotor=scale_rotor_input
    for i in range(0,8):
        if total_rotor_input_abs[i]>max_trust[i]:
            scale_rotor_input=max_trust[i]/total_rotor_input_abs[i]
            if scale_rotor>scale_rotor_input:
                scale_rotor=scale_rotor_input
    total_rotor_input=total_rotor_input*scale_rotor
    return total_rotor_input

def multiply(vector,float_val):
    result=np.zeros(len(vector))
    for i in range(0,len(vector)):
        result[i]=float_val*vector[i]
    return result

def motor_controller_by_leds(control_array):
    control_array=control_array.data
    global fase
    global count
    count=count+1
    print(control_array[0])
    if control_array[0]==0:
        fase=2
        ajuste_pose(control_array)
    else:
        if control_array[0]==1:
            fase=2
            slow_final_aproach(control_array)
        else:
            if control_array[0]==3:
                #print(control_array[4])
                blind_ajuste_pose(control_array)
            else:
                if control_array[0]==2:
                    ajuste_x(control_array)

def slow_final_aproach(control_array):
    global fase
    global finish_sec
    porportional_control=[15,-15,0.7]       # (x_point, alt, distance)   o da distancia n e na realidade com base na distancia mas sim no tamanho do led avistado
    led_size_parked=400
    if control_array[3]>3*led_size_parked:
        print("finish")
        fase=3 ##acabou_com_sucesso_a_manobra
        publish_to_rotors([0,0,0,0,0,0,0,0])
        finish_sec=rospy.get_time()
    else:
        x_pointer_input=float(porportional_control[0]*control_array[1])
        z_input=float(porportional_control[1]*(control_array[2]+0.1))
        distance_factor=float(led_size_parked/control_array[3])
        front_input=porportional_control[2]*distance_factor
        print(x_pointer_input)
        print(z_input)
        print(front_input)
        total_rotor_input=motor_encoder_trust(x_pointer_input,z_input,front_input)
        if 0.1>front_input and front_input>0:
            front_input=0.1
        if control_array[3]>40:
            total_rotor_input=total_rotor_input*0.5
        print(total_rotor_input)
        #publicar o total_rotor_input
        publish_to_rotors(total_rotor_input)

def ajuste_x(control_array):
    porportional_control=[0.2,0.5,-1] # (x_point_control control_arrasto z_control)
    dist_xy=math.sqrt(control_array[1]**2+control_array[2]**2)
    drag_value=control_array[1]*(1+dist_xy/4)
    x_pointer_value=control_array[2]/(1+dist_xy/4)-drag_value/2
    
    x_pointer_input=float(porportional_control[0]*x_pointer_value)
    drag_input=float(porportional_control[1]*drag_value)
    z_input=float(porportional_control[2]*control_array[3])

    total_rotor_input=motor_encoder_trust2(x_pointer_input,z_input,drag_input)
    print(total_rotor_input)
    publish_to_rotors(total_rotor_input)


def ajuste_pose(control_array):
    led_size_parked=200
    porportional_control=[0.4,-2, 0.5]  #antes na altura estava +0.4 mas creio q foi esquecimento
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
    distance_factor=float(1/float(led_size_parked)*float(control_array[5]))
    front_input=float(porportional_control[2]*distance_factor)
    total_rotor_input=motor_encoder_trust(x_pointer_input,z_input,front_input)
    #publicar o total_rotor_input
    publish_to_rotors(total_rotor_input)


def blind_ajuste_pose(control_array):
    global fase
    global init_search_time
    #print(control_array[1])
    #print(control_array[2])
    if control_array[1]+control_array[2]!=0:
        fase=2
        center_led_viewed(control_array)
    else:
        if fase!=1:
            fase=1
            init_search_time=rospy.get_time()
            define_rout()
            follow_rout_defined(0)
        else:
            time_ros=rospy.get_time()
            search_time=time_ros-init_search_time
            follow_rout_defined(search_time)


def define_rout():
    global time_motor_set
    p1i=[0,4,10,18,25]
    p1o=[2,6,12,19,30]
    zmi=[2,12]
    zmo=[4,14]
    zmmi=[6]
    zmmo=[10]
    fri=[14,19,30]
    fro=[18,25,35]
    bai=[-1]
    bao=[-1]
    p2i=[-1]
    p2o=[-1]
    time_motor_set=time_motors(bai,bao,p1i,p1o,p2i,p2o,fri,fro,zmmi,zmmo,zmi,zmo)



def follow_rout_defined(search_time):
    bigguer_rotor_value=0
    total_rotor_input=np.zeros(8)
    trust_defined=0
    if time_motor_set.pitch1_in[0]!=-1:
        for i in range (0,len(time_motor_set.pitch1_in)):
            if search_time >= time_motor_set.pitch1_in[i] and time_motor_set.pitch1_out[i]> search_time:
                total_rotor_input=yaw_r
                trust_defined=1
    if time_motor_set.zm1_in[0]!=1:
        for i in range (0,len(time_motor_set.zm1_in)):
            if search_time >= time_motor_set.zm1_in[i] and time_motor_set.zm1_out[i] > search_time:
                total_rotor_input=up
                trust_defined=1
    if time_motor_set.zmm1_in[0]!=1:
        for i in range (0,len(time_motor_set.zmm1_in)):
            if search_time >= time_motor_set.zmm1_in[i] and time_motor_set.zmm1_out[i] > search_time:
                total_rotor_input=up
                trust_defined=1
    if time_motor_set.front1_in[0]!=1:
        for i in range (0,len(time_motor_set.front1_in)):
            if search_time >= time_motor_set.front1_in[i] and time_motor_set.front1_out[i] > search_time:
                total_rotor_input=up
                trust_defined=1
    if time_motor_set.back1_in[0]!=1:
        for i in range (0,len(time_motor_set.back1_in)):
            if search_time >= time_motor_set.back1_in[i] and time_motor_set.back1_out[i] > search_time:
                total_rotor_input=up
                trust_defined=1
    if time_motor_set.pitch2_in[0]!=-1:
        for i in range (0,len(time_motor_set.pitch2_in)):
            if search_time >= time_motor_set.pitch2_in[i] and time_motor_set.pitch2_out[i]> search_time:
                total_rotor_input=yaw_r
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
    porportional_control=[3,-4, 4]#antes estava 0.4 mas cerio q foi esquecimento
    x_point_array=control_array[4]
    z_array=control_array[5]
    x_pointer_input=porportional_control[0]*x_point_array
    z_input=porportional_control[1]*z_array
    if control_array[3] > 100:
        front_input=-0.5
    else:
        distance_factor=float(float(control_array[3])/200)
        front_input=porportional_control[2]*distance_factor
        #print(front_input)
    total_rotor_input=motor_encoder_trust(x_pointer_input,z_input,front_input)*0.7
    #publicar o total_rotor_input
    publish_to_rotors(total_rotor_input)




def publish_to_rotors(total_rotor_input):
    global time
    msg_setpoints=Setpoints()
    msg_setpoints.header.seq = count
    time=rospy.get_rostime()
    msg_setpoints.header.stamp=time
    msg_setpoints.header.frame_id = "/iris/base_link"
    msg_setpoints.setpoints = total_rotor_input
    pub_thrusters_.publish(msg_setpoints)




#############################################################################################################
        
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass