#experiencia com novo mapa de luzes
import cv2
import numpy as np 
from PIL import Image 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
# from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Pose
br=CvBridge()
from std_msgs.msg import String,Float64MultiArray,MultiArrayLayout,MultiArrayDimension,Float64
#from cola2_msgs.msg import Setpoints, DVL
#a=Setpoints()

# dados para aplicar filtro de cor hsv
class color_filter_hsv:  
	def __init__(self,hm,hM,sm,sM,vm,vM):
		self.hm=hm
		self.hM=hM
		self.sm=sm
		self.sM=sM
		self.vm=vm
		self.vM=vM
	pass

# define variables
green_filter=color_filter_hsv(35,85,30,255,170,255) #4 green
red_filter=color_filter_hsv(143,212,93,255,40,255)#1 red
red_filter2=color_filter_hsv(0,20,110,255,100,255)#2 red
blue_filter=color_filter_hsv(110,135,175,255,175,255)#3 blue
#blue_filter=color_filter_hsv(100,135,175,255,100,255)#3 blue

old_led_pos=np.ones((8,2))*-1
frames=0
D=np.zeros((1,5))
k=np.array([[1306.267846340393, 0.0, 680.0],	[0.0, 1306.267846340393, 512.0],[0.0, 0.0, 1.0]])
#led_world_pos=([[0.5, 0, 0.375],[0, 0, 0.375],[-0.5, 0, 0.375],[0.5, 0, -0.375],[0, 0, -0.375],[-0.5, 0, -0.375],[0.2,-1,0],[-0.2,-1,0]])
led_world_pos=([[0.5, 0, 0.375],[0, 0, 0.375],[-0.5, 0, 0.375],[0.5, 0, -0.375],[0, 0, -0.375],[-0.5, 0, -0.375],[0.2,-1,-0.05],[-0.2,-1,0.05]])
#led_world_pos=([[0.5, 0, 0.575],[0, 0, 0.575],[-0.5, 0, 0.575],[0.5, 0, -0.175],[0, 0, -0.175],[-0.5, 0, -0.175],[0.2,-1,0.2],[-0.2,-1,0.2]])


pose_publisher = rospy.Publisher('/iris/pose_publisher', Pose, queue_size=1)
direction_publisher = rospy.Publisher('/iris/diretionions_publisher', Float64MultiArray, queue_size=1)
def main():
    rospy.init_node('iris_img_viewer', anonymous=True)
    while not rospy.is_shutdown():
        rospy.Subscriber("/iris/proscilica_front/image_color", Image, img_extract_points)
        rospy.spin()


def img_extract_points(img):
    img=tuner_img_and_color_filter(img)

    contornos_blue=contrornos_led(img,blue_filter,0)
    contornos_green=contrornos_led(img,green_filter,0)
    contornos_red=contrornos_led(img,red_filter,red_filter2)

    centerList_blue_led=color_point(contornos_blue,img)
    centerList_green_led=color_point(contornos_green,img)
    centerList_red_led=color_point(contornos_red,img)

    #print(a)
    #print(led_size(contornos_green))
    print(len(centerList_blue_led))
    print(len(centerList_green_led))
    print(len(centerList_red_led))
    print(rospy.get_time())
    #contronos_leds=np.append([contornos_blue, contornos_green, contornos_red])
    #print(np.shape(contronos_leds))
    data_assiciation_master(centerList_green_led,centerList_red_led,centerList_blue_led,contornos_green,contornos_red,contornos_blue)
    return

def tuner_img_and_color_filter(img):
    global green_filter, blue_filter, red_filter, red_filter2
    img = br.imgmsg_to_cv2(img)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.undistort(img, k, D,None,k)
    img = cv2.blur(img,(5,5))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split (hsv)
    mean_value_img = v.mean ()
    print(mean_value_img)
    if mean_value_img < 90:
        blue_filter=color_filter_hsv(95,135,140,255,95,255)     #4 blue
        red_filter2=color_filter_hsv(0,20,0,255,40,255)         #3 red
    else:
        blue_filter=color_filter_hsv(110,135,175,255,175,255)   #3 blue
        red_filter2=color_filter_hsv(0,20,110,255,100,255)      #2 red
    return img
        


def contrornos_led(img,filter1,filter2):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([filter1.hm, filter1.sm, filter1.vm], np.uint8)
    upper = np.array([filter1.hM, filter1.sM, filter1.vM], np.uint8)
    blue_mask = cv2.inRange(hsv, lower, upper)
    if filter2!=0:
        lower = np.array([filter2.hm, filter2.sm, filter2.vm], np.uint8)
        upper = np.array([filter2.hM, filter2.sM, filter2.vM], np.uint8)
        blue_mask2 = cv2.inRange(hsv, lower, upper)
        blue_mask = cv2.bitwise_or(blue_mask, blue_mask2)
    kernel = np.ones((3,3),np.uint8)
    blue_mask = cv2.erode(blue_mask,kernel,iterations = 1)
    kernel = np.ones((20,20),np.uint8)
    blue_mask = cv2.dilate(blue_mask,kernel,iterations = 1)
    ret,blue_mask = cv2.threshold(blue_mask,10,255,cv2.THRESH_BINARY)
    _, contours3, _= cv2.findContours(blue_mask ,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    return contours3


def color_point(contours3,img):    
    blue_white_blocks=cv2.drawContours(img, contours3, -1, (0,255,0), 3)
    centerList_blue_led = []
    for c in contours3:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(img, "center", (cX - 20, cY - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        center=[cX,cY]
        centerList_blue_led.append(center)
    return centerList_blue_led



# make the data association for the leds
def data_assiciation_master(centerList_green_led,centerList_red_led,centerList_blue_led,contornos_green,contornos_red,contornos_blue):
    global old_led_pos
    global frames
    if len(centerList_green_led)==3 and len(centerList_red_led)==3:
        # se virmos todos os leds da base calcular a pos com a certeza de q estamos a fazer a acossiacao correta 
        data_association0(centerList_green_led,centerList_red_led,centerList_blue_led,contornos_green,contornos_red,contornos_blue)
        frames=2
    else:
        if len(centerList_red_led)<=3  and len(centerList_green_led)<=3 and len(centerList_blue_led)<=2 and frames>0:
            oriented_led=data_association1(centerList_green_led,centerList_red_led,centerList_blue_led) 
            if (len(centerList_green_led)+len(centerList_red_led)+len(centerList_blue_led))>=4:
                position_calc(centerList_green_led,centerList_red_led,centerList_blue_led,oriented_led,contornos_green,contornos_red,contornos_blue)
            else:
                direction_to_search(oriented_led,contornos_green,contornos_red,contornos_blue)
                frames=frames-1
        else:
            direction_to_search_blind(centerList_green_led,centerList_red_led,centerList_blue_led,contornos_green,contornos_red,contornos_blue)
            frames=frames-1
        


    

# calc the pose then are seen 6 leds
def data_association0(centerList_green_led,centerList_red_led,centerList_blue_led,contornos_green,contornos_red,contornos_blue):
    global old_led_pos
    global frames
    alt_med_red=(centerList_red_led[0][1]+centerList_red_led[1][1]+centerList_red_led[2][1])/3
    alt_med_green=(centerList_green_led[1][1]+centerList_green_led[0][1]+centerList_green_led[2][1])/3
    x_img_red_med=(centerList_red_led[0][0]+centerList_red_led[1][0]+centerList_red_led[2][0])/3
    x_img_green_med=(centerList_green_led[0][0]+centerList_green_led[1][0]+centerList_green_led[2][0])/3
    alt_leds=math.sqrt((centerList_red_led[0][1]-centerList_green_led[0][1])**2+(centerList_red_led[0][0]-centerList_green_led[0][0])**2)
    delta_alt_green_red=math.sqrt((alt_med_red-alt_med_green)**2)
    oriented_led=[centerList_red_led[0],centerList_red_led[1],centerList_red_led[2],centerList_green_led[0],centerList_green_led[1],centerList_green_led[2]]
    if  delta_alt_green_red > (alt_leds/5): ## caso o drone esteja razoavelmente horizontal e n ao contrario
        centerList_red_led.sort(key=takeximg)
        centerList_green_led.sort(key=takeximg)
        centerList_blue_led.sort(key=takeximg)
        if alt_med_green > alt_med_red: 		#verificar se n esta de pernas para o ar se nao
            oriented_led=[centerList_red_led[0],centerList_red_led[1],centerList_red_led[2],centerList_green_led[0],centerList_green_led[1],centerList_green_led[2]]
        else:								# verificar se n esta de pernas para o ar se sim
            oriented_led=[centerList_red_led[2],centerList_red_led[1],centerList_red_led[0],centerList_green_led[2],centerList_green_led[1],centerList_green_led[0]]
            if len(centerList_blue_led)==2:
                centerList_blue_led=[centerList_blue_led[1],centerList_blue_led[0]]
    else:
        if delta_alt_green_red <= (alt_leds/5): ## caso o drone esteja razoavelmente horizontal e ao contrario
            if len(centerList_blue_led)==2:
                    centerList_blue_led=[centerList_blue_led[0],centerList_blue_led[1]]
            if x_img_red_med > x_img_green_med: #inclinado para a direita  #organizado por altura
                oriented_led(centerList_red_led[0],centerList_red_led[1],centerList_red_led[2],centerList_green_led[0],centerList_green_led[1],centerList_green_led[2])
            else:
                oriented_led(centerList_red_led[2],centerList_red_led[1],centerList_red_led[0],centerList_green_led[2],centerList_green_led[1],centerList_green_led[0])
                if len(centerList_blue_led)==2:
                    centerList_blue_led=[centerList_blue_led[1],centerList_blue_led[0]]	
    if len(centerList_blue_led)==2:
        oriented_led=np.append(oriented_led, centerList_blue_led, axis = 0)
        old_led_pos=oriented_led
    world_pts=np.zeros((len(oriented_led),3))
    img_pts=np.zeros((len(oriented_led),2))
    for i in range (0,len(oriented_led)):
        img_pts[i]=oriented_led[i]
        world_pts[i]=led_world_pos[i]
 
    #print(img_pts)
    #print(world_pts)
    pos=[0,0,0]
    (_, rotation_vector, translation_vector) = cv2.solvePnP(world_pts, img_pts, k, D)
    (R, _) = cv2.Rodrigues(rotation_vector)
    
    #print(translation_vector)
    pos = np.dot(-np.transpose(R),translation_vector)
    print(pos)

    distancia_xy=math.sqrt(pos[0]**2 + pos[1]**2)
    if distancia_xy<3:
        if len(centerList_blue_led)==2:
            near_values=led_view(centerList_blue_led)
            direction_to_search_V=slow_aproach(near_values,centerList_blue_led,contornos_blue)
            direction_to_search_publisher(direction_to_search_V)
        else:
            if pos[0]<0.1 and -0.1<pos[0]:
                direction_to_search_blind(centerList_green_led,centerList_red_led,centerList_blue_led,contornos_green,contornos_red,contornos_blue)
            else:
                direction_to_search_V=[2,pos[0][0],pos[1][0],pos[2][0],translation_vector[0][0]]
                direction_to_search_publisher(direction_to_search_V)
    else:
        publish_pose(rotation_vector,pos,translation_vector)
    
    if len(centerList_blue_led)!=2:
        world_pts2=np.zeros((8,3))
        for i in range (0,8):
            world_pts2[i]=led_world_pos[i]
        old_led_pos1, _ = cv2.projectPoints(world_pts2, rotation_vector, translation_vector, k, D)
        old_led_pos=[[old_led_pos1[0][0][0],old_led_pos1[0][0][1]],[old_led_pos1[1][0][0],old_led_pos1[1][0][1]],[old_led_pos1[2][0][0],old_led_pos1[2][0][1]],[old_led_pos1[3][0][0],old_led_pos1[3][0][1]],[old_led_pos1[4][0][0],old_led_pos1[4][0][1]],[old_led_pos1[5][0][0],old_led_pos1[5][0][1]],[old_led_pos1[6][0][0],old_led_pos1[6][0][1]],[old_led_pos1[7][0][0],old_led_pos1[7][0][1]]]
    #print(old_led_pos)



# make the match in 4 or 5 leds to compute the result(pose) in position_calc
def data_association1(centerList_green_led,centerList_red_led,centerList_blue_led):
    global old_led_pos
    global frames
    beta1=0
    beta2=0
    dist_min=10000
    index=-1
    oriented_led=np.ones((len(old_led_pos),2),dtype=np.float64)*-1

    for i in range (0,len(centerList_red_led)): #achar o ponto mais proximo
        dist_min=10000  #o valor q deveria estar era 275
        index=-1
        for k in range (0,3):
            dist=math.sqrt((old_led_pos[k][0]-centerList_red_led[i][0])**2+(old_led_pos[k][1]-centerList_red_led[i][1])**2)

            if dist < dist_min:
                    dist_min=dist   
                    index=k
        if index !=-1:
            oriented_led[index]=centerList_red_led[i]
    for i in range (0,len(centerList_green_led)):  #achar o ponto mais proximo 
        dist_min=10000  #o valor q deveria estar era 275
        index=-1
        for k in range (3,6):
            dist=math.sqrt((old_led_pos[k][0]-centerList_green_led[i][0])**2+(old_led_pos[k][1]-centerList_green_led[i][1])**2)
            if dist < dist_min:
                    dist_min=dist
                    index=k
        if index !=-1:
            oriented_led[index]=centerList_green_led[i]
    for i in range (0,len(centerList_blue_led)):  #achar o ponto mais proximo 
        dist_min=10000  #o valor q deveria estar era 275
        index=-1
        for k in range (7,8):
            dist=math.sqrt((old_led_pos[k][0]-centerList_blue_led[i][0])**2+(old_led_pos[k][1]-centerList_blue_led[i][1])**2)
            if dist < dist_min:
                    dist_min=dist
                    index=k
        if index !=-1:
            oriented_led[index]=centerList_blue_led[i]
    return oriented_led


def position_calc(centerList_green_led,centerList_red_led,centerList_blue_led,oriented_led,contornos_green,contornos_red,contornos_blue):
    global old_led_pos
    pos=[0,0,0]
    beta=0    
    #guardar na matriz apenas os leds vistos
    see_leds=len(centerList_red_led)+len(centerList_green_led)+len(centerList_blue_led)
    img_pts=np.ones((see_leds,2))*-1
    world_pts=np.zeros((see_leds,3))
    for i in range (0,len(old_led_pos)):
        if oriented_led[i][0] != -1:    
            img_pts[beta]=oriented_led[i]
            world_pts[beta]=led_world_pos[i]
            beta=beta+1

    (_, rotation_vector, translation_vector) = cv2.solvePnP(world_pts, img_pts, k, D)
    (R, _) = cv2.Rodrigues(rotation_vector)
    pos = np.dot(-np.transpose(R),translation_vector)
    print(pos)
    beta=0
    world_pts2=np.zeros((8,3))
    for i in range (0,8):
            world_pts2[beta]=led_world_pos[i]
            beta=beta+1
    old_led_pos1, _ = cv2.projectPoints(world_pts2, rotation_vector, translation_vector, k, D)
    old_led_pos=[[old_led_pos1[0][0][0],old_led_pos1[0][0][1]],[old_led_pos1[1][0][0],old_led_pos1[1][0][1]],[old_led_pos1[2][0][0],old_led_pos1[2][0][1]],[old_led_pos1[3][0][0],old_led_pos1[3][0][1]],[old_led_pos1[4][0][0],old_led_pos1[4][0][1]],[old_led_pos1[5][0][0],old_led_pos1[5][0][1]],[old_led_pos1[6][0][0],old_led_pos1[6][0][1]],[old_led_pos1[7][0][0],old_led_pos1[7][0][1]]]
    frames=2

    distancia_xy=math.sqrt(pos[0]**2 + pos[1]**2)
    if distancia_xy<3:
        if len(centerList_blue_led)==2:
            near_values=led_view(centerList_blue_led)
            direction_to_search_V=slow_aproach(near_values,centerList_blue_led,contornos_blue)
            direction_to_search_publisher(direction_to_search_V)
        else:
            if pos[0]<0.1 and -0.1<pos[0]:
                direction_to_search_blind(centerList_green_led,centerList_red_led,centerList_blue_led,contornos_green,contornos_red,contornos_blue)
            else:
                direction_to_search_V=[2,pos[0][0],pos[1][0],pos[2][0],translation_vector[0][0]]
                direction_to_search_publisher(direction_to_search_V)
    else:
        publish_pose(rotation_vector,pos,translation_vector)


# organize and publish the pose
def publish_pose(rotation_vector,pos,translation_vector):
    global pose_publisher
    p=Pose()
    p.orientation.w=translation_vector[0]
    p.orientation.x=rotation_vector[0]
    p.orientation.y=rotation_vector[1]
    p.orientation.z=rotation_vector[2]
    (p.position.x,p.position.y,p.position.z)=pos
    rospy.loginfo(p)
    pose_publisher.publish(p)



def direction_to_search(oriented_led,contornos_green,contornos_red,contornos_blue):
    global direction_publisher
    direction_to_search_V = Float64MultiArray()
    right=[oriented_led[2],oriented_led[5]]
    left=[oriented_led[0],oriented_led[3]]
    center=[oriented_led[1],oriented_led[4]]
    near=[oriented_led[6],oriented_led[7]]
    top=[oriented_led[0],oriented_led[1],oriented_led[2]]
    down=[oriented_led[5],oriented_led[3],oriented_led[4]]
    right_values=led_view(right)
    left_values=led_view(left)
    center_values=led_view(center)
    near_values=led_view(near)
    top_values=led_view(top)
    down_values=led_view(down)
    if 0<near_values and top_values==0 and down_values==0 and right_values==0 and left_values==0:
        direction_to_search_V=slow_aproach(near_values,near,contornos_blue)
    else:
        leds_diameter=gr_medium_size(contornos_green,contornos_red)
        direction_to_search_V=[0,top_values,down_values,left_values,right_values,leds_diameter]
    direction_to_search_publisher(direction_to_search_V)
    

def slow_aproach(near_values,near,contornos_blue):
    direction_to_search_V = Float64MultiArray()
    if (near_values==2):
        x=float(((float(near[0][0])+float(near[1][0]))/2/(1360/2))-1)                   #normalizar o x entre -1 e 1 onde 0  o ponto ideal
        distance=float(np.sqrt((float(near[0][0])-float(near[1][0]))**2+(float(near[0][1])-float(near[1][1]))**2)/1360)   #normalizado mais o menos de 0 a 1 onde um  a largura da foto
        y=float(((float(near[0][1])+float(near[1][1]))/2/(1024/2))-1)                                        #normalizar o y entre -1 e 1 onde 0  o ponto ideal
        size_light=led_size(contornos_blue)                                    #era de relevo ter o tamanho da luz
        size_m_leds=led_diameter_medium(size_light)
        direction_to_search_V=[1,x,y,size_m_leds,distance]
    else:
        if (near[0][0]==-1):
            x=float((float(near[1][0])/(1360/2))-1)
            y=float((float(near[1][1])/(1024/2))-1)
            size_light=led_size(contornos_blue)
            size_led=(size_light[0][0]+size_light[0][1])/2
            direction_to_search_V=([1,x,y,size_led])
        else:
            x=float((float(near[0][0])/(1360/2))-1)
            y=float((float(near[0][1])/(1024/2))-1)
            size_light=led_size(contornos_blue)
            size_led=(size_light[0][0]+size_light[0][1])/2
            direction_to_search_V=[1,x,y,size_led]
    return direction_to_search_V

def direction_to_search_blind(centerList_green_led,centerList_red_led,centerList_blue_led,contornos_green,contornos_red,contornos_blue):
    near_values=len(centerList_blue_led)
    top_values=len(centerList_red_led)
    down_values=len(centerList_green_led)
    if 0<near_values : #and top_values+down_values==0
        direction_to_search_V=slow_aproach(near_values,centerList_blue_led,contornos_blue)
    else:
        [x,y]=medium_point_leds(centerList_green_led,centerList_red_led,centerList_blue_led,near_values,top_values,down_values)
        leds_diameter=gr_medium_size(contornos_green,contornos_red)
        direction_to_search_V=[3,top_values,down_values,leds_diameter,x,y]
    direction_to_search_publisher(direction_to_search_V)


def direction_to_search_publisher(direction_to_search_V):
    global direction_publisher
    my_msg = Float64MultiArray(data = direction_to_search_V)
    rospy.loginfo(my_msg)
    my_msg.layout.dim.append(MultiArrayDimension())
    my_msg.layout.dim[0].label = "direction_to_search"
    my_msg.layout.dim[0].size = len(direction_to_search_V)
    direction_publisher.publish(my_msg)


def led_size(contours3):
    size_led=0
    beta=len(contours3)
    size_leds=np.zeros((beta,2))
    for i in range(0,len(contours3)):
        x_max=0
        y_max=0
        y_min=1080
        x_min=1360
        for ii in range (0,len(contours3[i])):
            if y_max<contours3[i][ii][0][1]:
                y_max=contours3[i][ii][0][1]
            if contours3[i][ii][0][1]<y_min:
                y_min=contours3[i][ii][0][1]

            if x_max<contours3[i][ii][0][0]:
                x_max=contours3[i][ii][0][0]
            if contours3[i][ii][0][0]<x_min:
                x_min=contours3[i][ii][0][0]
        size_led=[x_max-x_min,y_max-y_min]
        size_leds[i]=size_led
    if beta==0:
        k=np.ones((1,2))*-1
        return k
    return size_leds

def led_diameter_medium(size_leds):
    medium_vlue=0
    for ii in range(0, len(size_leds)):
        medium_vlue=medium_vlue+size_leds[ii][0]+size_leds[ii][1]
    medium_vlue=medium_vlue/(2*len(size_leds))
    return medium_vlue

def gr_medium_size(contornos_green,contornos_red):
    leds_diameter=-1
    g_led_size=led_size(contornos_green)
    r_led_size=led_size(contornos_red)
    print(np.shape(g_led_size))
    g_led_diameter=led_diameter_medium(g_led_size)
    r_led_diameter=led_diameter_medium(r_led_size)
    leds_diameter=0
    div=0
    if 0<g_led_diameter:
        leds_diameter=leds_diameter+g_led_diameter
        div=div+1
    if 0<r_led_diameter:
        leds_diameter=leds_diameter+r_led_diameter
        div=div+1
    if 0<div:
        leds_diameter=leds_diameter/div
    return leds_diameter

def medium_point_leds(centerList_green_led,centerList_red_led,centerList_blue_led,near_values,top_values,down_values):
    x=0
    y=0
    total_leds=near_values+top_values+down_values
    if 0<near_values:
        for i in range(0,near_values):
            x=x+centerList_blue_led[i][0]
            y=y+centerList_blue_led[i][1]
            print(centerList_blue_led[i])
    if 0<top_values:
        for i in range(0,top_values):
            x=x+centerList_red_led[i][0]
            y=y+centerList_red_led[i][1]
            print(centerList_red_led[i])
    if 0<down_values:
        for i in range(0,down_values):
            x=x+centerList_green_led[i][0]
            y=y+centerList_green_led[i][1]
            print(centerList_green_led[i])
    if 0<total_leds:
        x_med=float((float(x)/float(total_leds)/(1360/2))-1)
        y_med=float((float(y)/float(total_leds)/(1080/2))-1)
    else:
        x_med=-2
        y_med=-2
    return [x_med,y_med]

def led_view(vector):
    view=0
    for i in range(0,len(vector)):
        if vector[i][0]!=-1:
            view=view+1
    return view

def takeximg(elem):
    return elem[0]

def takeyimg(elem):
    return elem[1]

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass