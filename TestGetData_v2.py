#!/usr/bin/python
## Human Skeleton Data

from turtle import Turtle
import rospy
from std_msgs.msg import Int16
from ros_openpose.srv import Trigger,TaskLabel
import numpy as np
import pandas as pd
import time
from ros_openpose.msg import Frame, Person, BodyPart, Pixel

class RawDataGet():
    '''
    The skeleton is considered as a combination of line strips.
    Hence, the skeleton is decomposed into 3 LINE_STRIP as following:
        1) upper_body : from nose to mid hip
        2) hands : from left-hand wrist to right-hand wrist
        3) legs : from left foot toe to right foot toe

    See the link below to get the id of each joint as defined in Kinect v2
    src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#keypoint-ordering
    Result for BODY_25 (25 body parts consisting of COCO + foot)
    const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS {
        { 0,      "Nose"},    {13,      "LKnee"}
        { 1,      "Neck"},    {14,     "LAnkle"}
        { 2, "RShoulder"},    {15,       "REye"}
        { 3,    "RElbow"},    {16,       "LEye"}
        { 4,    "RWrist"},    {17,       "REar"}
        { 5, "LShoulder"},    {18,       "LEar"}
        { 6,    "LElbow"},    {19,    "LBigToe"}
        { 7,    "LWrist"},    {20,  "LSmallToe"}
        { 8,    "MidHip"},    {21,      "LHeel"}
        { 9,      "RHip"},    {22,    "RBigToe"}
        {10,     "RKnee"},    {23,  "RSmallToe"}
        {11,    "RAnkle"},    {24,      "RHeel"}
        {12,      "LHip"},    {25, "Background"}


    hand output ordering
    src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/doc/media/keypoints_hand.png
    We are using 5 LINE_STRIP to draw a hand
    '''

    def __init__(self):
        
        rospy.init_node('HumanDataRaw')  ## it tells rospy the name of your node
        self._sub_human = rospy.Subscriber('/frame', Frame, self.HumanDataCallback)
        self.body_part = dict()
        self.body_part_names = ["Nose", "Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist","MidHip", "RHip", "RKnee", "RAnkle", "LHip", 
                                "LKnee", "LAnkle", "REye", "LEye", "REar", "LEar", "LBigToe", "LSmallToe", "LHeel", "RBigToe", "RSmallToe", "RHeel", "Background"]
        self.body_part_names_of_interests = ["Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist","MidHip"]
        


        ## Service Value Initialization
        self.record_data_flag = False
        self.task_label = 0.0
        ## Ros service for run experiment and change the CSV order
        # rospy.Service("service name", service_class, handler)   handler : handle the message
        self.prefix = "/get_human_pose/"
        rospy.Service(self.prefix + "trigger_experiment_flag",Trigger,self.triggerBodyDataRecord)  ## Trigger to start experiment
        #TO DO LIST make or find srv file for changing the integer value
        rospy.Service("Make_Task_Label",TaskLabel,self.LabelChange)  ## Change task label data 
        
        ## For checking Time
        self.prev_time = rospy.Time.now
        self.cur_time = rospy.Time.now
        self.start_time = rospy.Time.now
        
        ## For saving data
        self.upper_raw_list = []
        self.step_time_list = []
        self.task_label_list = []
        


        pass
    
    
    def triggerBodyDataRecord(self, req):
        if(req == 0):
            print("Data recording flag off!")
            trigger_flag = False
            self.record_data_flag = trigger_flag
            
        elif(req == 1):
            print("Data recording flag triggered!")
            trigger_flag = True
            self.record_data_flag = trigger_flag
            self.sample_index =+ 1
        else:
            self.record_data_flag = False
            pass
     
            
    def LabelChange(self, req):
        print("Task Change!")
        if(req == 1):
            print("Repetitive work")
            task_num = 1
            self.task_label = task_num
            task_name = str("Repetitive Work")
            self.task_label_name = task_name
            pass
        elif(req == 2):
            print("Assembly work")
            task_num = 2
            self.task_label = task_num
            task_name = str("Assembly Work")
            self.task_label_name = task_name
        else:
            pass
        
    # This function make CSV row title ex) RShoulder_x, RShoulder_y, RShoulder_z etc..    
    def MakeBodyPartXYZ(self):
        list_bodypart_names_xyz = list()
        for body_name in self.body_part_names_of_interests:
            list_bodypart_names_xyz.append(body_name + "_x")
            list_bodypart_names_xyz.append(body_name + "_y")
            list_bodypart_names_xyz.append(body_name + "_z")
        self.list_bodypart_names_xyz = list_bodypart_names_xyz
        

    
    def GetUpperLimbVariables(self):
        #upper_limb_data = self.body_part.get(self.body_part_names_of_interests)
        upper_limb_data_set = dict()
        print("len(self.list_bodypart_names_xyz)",len(self.list_bodypart_names_xyz))
        
        
        for i in range(len(self.body_part_names_of_interests)):
            upper_limb_data_set[self.list_bodypart_names_xyz[3*i-3]] = self.body_part[self.body_part_names_of_interests[[i-1]]][0]
            upper_limb_data_set[self.list_bodypart_names_xyz[3*i-2]] = self.body_part[self.body_part_names_of_interests[[i-1]]][1]
            upper_limb_data_set[self.list_bodypart_names_xyz[3*i-1]] = self.body_part[self.body_part_names_of_interests[[i-1]]][2]
        return upper_limb_data_set

    def HumanDataCallback(self, data):
        self.frame_rawdata = data
        self.frame_id = data.header.frame_id
        self.data_step_time = data.header.stamp
        self.person_data = data.persons
        #print("self.person_data:", self.person_data)

        if(len(self.person_data) > 0):
            for i in range(len(self.person_data)):
                body_parts = data.persons[i].bodyParts
                for j in range(len(body_parts)):
                    body_part_name = self.body_part_names[j]
                    point_data = body_parts[j].point    #point data have 
                    #self.body_part[body_part_name] = np.array([point_data.x, point_data.y, point_data.z])
                    self.body_part[body_part_name] = list([point_data.x, point_data.y, point_data.z])
        #print("RAWdata",data)
    
    def GetUpperLimbVariables(self):
        self.body_part_names_of_interests
        self.upper_limb_data = dict()
        for i in range(len(self.body_part_names_of_interests)):
            self.upper_limb_data[self.body_part_names_of_interests[i]].append(self.body_part.get(self.body_part_names_of_interests[i]))
        ## Then now upper_limb_data looks = [Neck.point.x , Neck.point.y, Neck.point.z,RShoulder.point.x , RShoulder.point.y, RShoulder.point.z,..., MidHip.point.z ]
        pass

def talker():
    callback_time_duration = 1.0 / 60.0 # 4ms

    raw_data = RawDataGet()
    
    # Initial sample index
    raw_data.sample_index = 0
    raw_data.MakeBodyPartXYZ()
    

    time_stack = 0 #initial total time 0
    rate = rospy.Rate(60) # 500Hz
    time_limit = 1000
    print("Ready for the experiment")
    while not rospy.is_shutdown():
        
        #print("keep while loop")
        if(raw_data.record_data_flag == True):   ### Data Collection Start! --> from ros service "trigger_experiment_flag"
            raw_data.prev_time = rospy.Time.now
            if(time_stack < time_limit):
                try:
                    upper_limb_data = raw_data.GetUpperLimbVariables()
                    raw_data.upper_raw_list.append(upper_limb_data)  # then list have list
                    print("Doing Well")
                except:
                    upper_limb_data = prev_data
                    print("Fail use pervious data")
                prev_data = upper_limb_data
            else:  
                raw_data.record_data_flag = False
                output_df = pd.DataFrame(data=raw_data.upper_raw_list, columns = raw_data.list_bodypart_names_xyz)
                output_df['sampling time'] = raw_data.step_time_list
                output_df['task_label'] = raw_data.task_label_list
                csv_save_name = '\home\hshhln\experiment_data\human_upperlimb_pose\human_upper_limb_data_collection_' + raw_data.task_label_name + str(raw_data.sample_index)+ '.csv'
                output_df.to_csv(csv_save_name)
                print("Finished exporting csv named " + csv_save_name + ", safe to exit!")
                
            raw_data.cur_time = rospy.Time.now
            relative_time = raw_data.cur_time - raw_data.prev_time
            # Save the Time and Task Label
            raw_data.step_time_list.append(relative_time)
            raw_data.task_label_list.append(raw_data.task_label)
            time_stack = time_stack + relative_time 
        else:
            pass
        
        rate.sleep()
        #rospy.spin()



if __name__ == '__main__':
    talker()
