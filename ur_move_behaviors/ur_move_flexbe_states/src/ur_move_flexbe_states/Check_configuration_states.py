#! /usr/bin/env python3

from flexbe_core              import EventState
from tqdm                     import tqdm
import copy
import numpy as np


class CheckConfiguration(EventState):
    
    '''
        Goal:
            check if in the joint trajectory found, there are singularity points (if there are that configuration will be discarded). If choice_primary is True,
            the ik_choice can assume whatever configuration, otherwise it can assume only the one with highest priority
        Parameters:
            choice_primary: used for defining which robot can do the primary trajectory and with which configuration (bool)
        '''

    def __init__(self, prior_config = 0):
        super(CheckConfiguration, self).__init__(outcomes = ['done', 'failed'],input_keys = ['success', 'max_error', 'ik_solution'], output_keys = ['ik_choice'])
        self.prior_config = prior_config

    def execute(self, userdata):
        print(self.ik_solution[-1])
        boolean_list = list(np.array(self.max_error) <= 0.002)
        print(boolean_list)
        if (not True in boolean_list):    
            self.ik_choice = 0
            return 'failed'   
        print(boolean_list)
        matrix       = copy.deepcopy(self.ik_solution)
        
        for coloumn in tqdm(range(4)):
            if(self.success[coloumn] == False or boolean_list[coloumn] == False):
                boolean_list[coloumn] = False
            else:
                r = [True,True,True,True,True,True]
                for i,j,d in zip(matrix,matrix[1:],matrix[2:]):
                    sol1, sol2, sol3 = i[coloumn], j[coloumn], d[coloumn]
                    for z in range(6):
                        if (abs(sol1[z] - sol2[z]) > 0.30):
                            if(abs(sol1[z] - sol3[z]) > 0.30):
                                r[z] = False
                    if (False in r):
                        boolean_list[coloumn] = False
        print(boolean_list)

        self.ik_choice = self.priority(boolean_list)
        print(self.ik_choice)

        if (not True in boolean_list) or (self.ik_choice != self.prior_config):    
            self.ik_choice = 0
            return 'failed'   
                                         
        return 'done'  
     
    def on_enter(self, userdata):
        
        '''
            Input:
                ik_solution: list of matrices that contains the joints solution for each point of the trajectory (list(list(list)))
                success: list of booleans that indicates if there's a joints limit exceed (list(bool))
                max_error: indicates the maximum error between the desired and the obtained pose (list(float))
                robot: name of the robot (string)
            '''
            
        self.ik_solution = userdata.ik_solution
        self.success     = userdata.success
        self.max_error   = userdata.max_error

    def on_exit(self, userdata):
        
        '''
            Output:
                ik_choice: configuration chosen for the trajectory (int)
            '''
            
        userdata.ik_choice = self.ik_choice
       
    def priority(self, bool_list:list):

        config = []
        for sub,name in enumerate(bool_list):
            if (name == True):
                config.append(sub)         #salvo gli index della lista che tornano true = la configurazione che funziona 
        for val in config:
            if val == self.prior_config:
                res = val           #ritorna la configurazione a true con la più alta priorità
                break
            else:
                res = 10

        return res   
       
