#!/usr/bin/python
# -*- coding: utf-8 -*-

##############################################################################################
# 文件：main.py
# 作者：Zonesion wanghao 20220412
# 说明：aiarm 主应用程序
# 修改：20220510  增加config文件统一存储参数
#       20220621  适配新的机械臂，修改具体点位
# 注释：
##############################################################################################
import rospy
import time
import copy 
import sys   
import signal
import cvwin
import threading
import math
from geometry_msgs.msg import PoseStamped, Pose
import yaml
this = sys.modules[__name__]

##############################################################################################
# 公共参数配置文件
##############################################################################################
this.config_path="/home/zonesion/catkin_ws/src/marm_visual_control/config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.load(f.read())

##############################################################################################
# 物料定位相关参数
##############################################################################################
#颜色识别参数
this.color_param=config['color_param']
this.bin_param=config['bin_param']
#定位板相关参数
loc_plate=[141,192,465,386]         #物料定位框         单位 pix 640*480像素 定位板框 左上角x,y 和 右下角x,y 
loc_plate_act=[0.153,0.173,0.092]   #定位板实际长度     单位 m 定位板框实际长度 上底 下底 高[0.155,0.173,0.093]
loc_plate_act_origin=[0.21,0.000]   #定位板原点偏移     单位 m 实际定位原点 相对base_link
loc_x_off_mx=20                     #相机倾斜视角X像素偏移 单位 pix  最大值                 
loc_x_off_mi=0                      #相机倾斜视角X像素偏移 单位 pix  最小值
loc_y_off_mx=10                     #相机倾斜视角y像素偏移 单位 pix  最大值
loc_opt_par_x=1.0                   #X方向的优化参数
loc_opt_par_y=1.0                   #Y方向的优化参数

##############################################################################################
# 机械臂相关参数和位置点定义
##############################################################################################
#机械臂夹具打开角度
g_open=config["g_open"]
#机械臂抓取高度
arm_g_height=0.134
#拍照位
arm_cam_joint=[-0.09012192015910722, -0.04428321319882354, 0.8573005610356529, 1.9794888436393088, -0.018]
#过渡位
arm_trans_joint=[1.3780610085141091e-05, 0.005306861269549153, 0.97426785523231, 1.3754647497671328, 5.618289536439448e-05]
#放料位
place_green_pre_pos=[-1.3265524118232723, 0.3655966425860558, 0.958677844890208, 1.3130885033703161, 0.26166659383462304]
place_green_pos=[-1.3448637572281599, 0.7148921367093759, 0.8301213995891474, 1.383962933474151, 0.2529428516342751]
place_red_pre_pos=[-1.569808929949258, 0.30870103957173256, 0.8855745835008368, 1.540034105658909, 0.033599209040843736]
place_red_pos=[-1.5681242281313106, 0.6839995863555675, 0.8778300129793662, 1.3738957859199925, 0.03458501462580535]
place_blue_pre_pos=[-1.271269903982289, 0.2946518684133466, 0.8931614029547874, 1.6595248994610352, 0.303024230637421]
place_blue_pos=[-1.286016086089582, 0.3698417631423453, 1.4507393370560877, 1.1048128607753822, 0.3105957893466142]
place_yellow_pre_pos=[-1.5616071144115444, 0.1852763916838321, 1.066325580111766, 1.6094755035417987, 0.02467022151749461]
place_yellow_pos=[-1.5579521399156477, 0.2752727085124672, 1.5744065335667794, 1.085966129981658, 0.04446556249672398]

def quit(signum, frame):
    print('EXIT APP') 
    sys.exit()
    # rospy.signal_shutdown("arm is stopping")                  #发出机械臂停止运动信号

from arm import Arm
from camera import AiCamera

class AiArm(Arm,AiCamera):
    def __init__(self,g_open):
        super(AiArm,self).__init__(g_open,xarm="xarm")          #定义为在arm（3399）端运行此程序。初始化Arm类
        super(Arm,self).__init__("pill_detection_20220426",['box1', 'box2', 'box3', 'box4'])#初始化AiCamera类
       
def aiarmAPP():
    aiarm=AiArm(g_open)                                         #初始化AIarm
    aiarm.all_gohome()                                          #机械臂和夹具都回原位
    while not rospy.is_shutdown():  
        if aiarm.current_pos!='camera':
            aiarm.set_joint_value_target(arm_cam_joint)         #移动到相机位
            aiarm.current_pos='camera'
            rospy.sleep(4)                                      #消除抖动
        sta,rect,type=aiarm.pill_detect(20)
        if sta：
            print(type)
            #开始抓取
    aiarm.all_gohome()
    rospy.sleep(0.1)
    rospy.logwarn('EXIT APP') 
    aiarm.shutdown()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                          
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("AIARM_NODE", log_level=rospy.INFO)         #初始化节点
    aiarmAPP()                                                  #开始机械臂的物料筛选应用