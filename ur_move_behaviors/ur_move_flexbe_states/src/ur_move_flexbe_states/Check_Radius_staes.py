from flexbe_core               import EventState
import numpy as np

class CheckRadius(EventState):
    '''
        Goal:
            computation of the follower trajectory when the pirmary robot pose is fixed and we need to consider the presence of the cable between the two robots during every movement.

        '''

    def __init__(self):
        super(CheckRadius, self).__init__(outcomes = ['done'],input_keys = ['robot1_traj', 'robot2_traj'], output_keys = ['robot2_corrected_traj'])
        
        
    def execute(self, userdata):

        # homogeneus matrix
        homo_start = np.eye(4)
        homo_start[:3,3] = self.robot2_traj[0][:3]

        homo_first_p = np.eye(4)
        homo_first_p[:3,3] = self.robot1_traj[0][:3]

        first_point = np.dot(np.linalg.inv(homo_first_p), homo_start)

        pos_start = first_point[:3,3]

        radius_start = np.sqrt(pos_start[0]**2 + pos_start[1]**2 + pos_start[2]**2)


        for point1, point2 in zip(self.robot1_traj[1:], self.robot2_traj[1:]):
            if self.compute_radius(point1, point2) > radius_start:
                self.point_correction(point1, point2, radius_start)

        # Stampa le nuove traiettorie
        # print("Traiettoria 1:", self.robot1_traj)
        print("Traiettoria 2:", self.robot2_traj[-1])
                
        return 'done'

    def on_enter(self, userdata):
        '''
            Input:
                fix_primary_point: fixed pose of the primary robot (list)
                start_point_follower: starting pose of the follower robot (list)
                final_point_follower: final pose of the follower robot (list)
                radius: length of the cable (float)
            '''
        self.robot1_traj = userdata.robot1_traj
        self.robot2_traj = userdata.robot2_traj

    def on_exit(self, userdata):
        '''
            Output:
                follower_trajectory: cartesian follower trajectory (list(list))
                last_follower_point: last point of the follower trajectory (list)
            
            '''
        userdata.robot2_corrected_traj = self.robot2_traj
        print("exit ", self.robot2_traj[-1])
    
    def compute_radius(self, point1, point2):
        return np.linalg.norm(np.array(point1[:3]) - np.array(point2[:3]))

    def point_correction(self, point1, point2, max_radius):

        actual_radius = self.compute_radius(point1, point2)

        if actual_radius > max_radius:
            # Calcola il vettore differenza tra i due punti
            diff = np.array(point2[:3]) - np.array(point1[:3])
            # Normalizza il vettore differenza e moltiplica per il raggio massimo
            diff_normalizzato = diff / np.linalg.norm(diff) * max_radius
            # Aggiorna la posizione del secondo punto
            point2[:3] = point1[:3] + diff_normalizzato
