import os
import rclpy
import numpy as np
import numpy.matlib 
import quaternion
import time

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from stage_control_interfaces.action import MoveStage

from rclpy.node import Node
from ros2_igtl_bridge.msg import Transform
from numpy import asarray, savetxt, loadtxt

INIT_POINT_X = -25  #Initial robot X position at entry point
INIT_POINT_Z = 35   #Initial robot Z position at entry point
BUFF = 500          #Size of sensor buffer

class Registration(Node):

    def __init__(self):
        super().__init__('registration')

        #Topics from Aurora sensor node
        self.subscription_sensor = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.aurora_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Action client 
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.robot_idle = True                      # Stage move action status
        self.current_point = 0                      # Current registration point
        self.transform = np.empty(shape=[0,7])      # Registration transform (from aurora to stage)
        self.Z_sensor = np.empty(shape=[0,7])       # Tip sensor value in Aurora frame
        
        self.sensor_buffer = True                   # Buffer sensor data to extract mean value
        self.buffer_index = 0                       # Buffer index
        self.Z_buffer = np.empty(shape=[BUFF,7])    # Buffer of sensor data
        self.Z = np.empty(shape=[0,7])              # Filtered (mean) tip value in Aurora frame

        # Registration points A (aurora) and B (stage)
        ###############################################################################
        ### ATTENTION: Define B: Registration points in the stage frame             ###
        ### B = np.array([[x0, x1, ..., xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]) ###
        ###############################################################################
        #Registration points in aurora frame
        self.A = np.empty(shape=[3,0])          
        #Registration points in stage frame
        self.B = np.array([[-10, -15, -20, -25, -30, -35, -40, 
                            -40, -35, -30, -25, -20, -15, -10,
                            -10, -15, -20, -25, -30, -35, -40, 
                            -40, -35, -30, -25, -20, -15, -10,
                            -10, -15, -20, -25, -30, -35, -40, 
                            -40, -35, -30, -25, -20, -15, -10,
                            -10, -15, -20, -25, -30, -35, -40, 
                            -40, -35, -30, -25, -20, -15, -10,
                            -10, -15, -20, -25, -30, -35, -40
                            ],
                           [0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                           ], 
                           [15, 15, 15, 15, 15, 15, 15,
                            20, 20, 20, 20, 20, 20, 20, 
                            25, 25, 25, 25, 25, 25, 25, 
                            30, 30, 30, 30, 30, 30, 30, 
                            35, 35, 35, 35, 35, 35, 35, 
                            40, 40, 40, 40, 40, 40, 40, 
                            45, 45, 45, 45, 45, 45, 45, 
                            50, 50, 50, 50, 50, 50, 50, 
                            55, 55, 55, 55, 55, 55, 55
                           ]])   

    # Get current Aurora sensor measurements and filters
    def aurora_callback(self, msg_sensor):
        # Get needle shape from Aurora IGTL
        name = msg_sensor.name      
        if name=="NeedleToTracker": # Name is adjusted in Plus .xml
            # Get aurora new reading
            self.Z_sensor = np.array([[msg_sensor.transform.translation.x, msg_sensor.transform.translation.y, msg_sensor.transform.translation.z, \
                msg_sensor.transform.rotation.w, msg_sensor.transform.rotation.x, msg_sensor.transform.rotation.y, msg_sensor.transform.rotation.z]])
            # If sensor buffer is on (robot reached a registration point)
            if (self.sensor_buffer == True):
                # Store sensor reading in the buffer
                self.Z_buffer[self.buffer_index,:] = self.Z_sensor
                self.buffer_index = self.buffer_index+1
                # Completed buffer
                if (self.buffer_index >= BUFF):
                    self.Z = numpy.mean(self.Z_buffer, axis=0)
                    self.sensor_buffer = False

    def get_registration(self):  
        # Get points until A is same size as B
        if (self.A.shape[1] < self.B.shape[1]): 
            self.get_next_point()             
        else:
            # Calculate registration transform
            self.transform = np.array(find_registration(self.A, self.B))   #Store registration transform
            # Save matrix to file
            self.get_logger().debug('Save registration transform %s' %(self.transform))
            savetxt(os.path.join(os.getcwd(),'src','trajcontrol','files','registration.csv'), asarray(self.transform), delimiter=',')
            regitration_points = np.row_stack((self.A, self.B))
            savetxt(os.path.join(os.getcwd(),'src','trajcontrol','files','registration_points.csv'), asarray(regitration_points), delimiter=',')
            # Send robot back to home position
            self.get_logger().info('Returning robot to home position... %s' %(self.transform))
            self.robot_idle = False
            self.send_cmd(INIT_POINT_X, INIT_POINT_Z)

    def get_next_point(self):
        # Robot finished moving and sensor buffer already full
        if (self.robot_idle == True) and (self.sensor_buffer == False):
            # New point reached but not stored
            if (self.A.shape[1] < self.current_point):
                if (self.Z.size == 0):
                    self.get_logger().info('There is no sensor reading to store. Check Aurora')
                else:
                    P = self.Z[0:3] # Store next registration point
                    self.get_logger().info('Stored A Point #%i = (%f, %f, %f)' % (self.current_point, P[0], P[1], P[2]))
                    self.A = np.column_stack((self.A, P.T))
            # Select next point
            else:
                # Get next B point
                P = self.B[:, self.A.shape[1]]
                # Send robot to next point
                self.current_point = self.current_point +1
                self.send_cmd(P[0], P[2])          
                self.get_logger().info('Sending robot to B Point #%i = (%f, %f, %f)' % (self.current_point, P[0], P[1], P[2]))

    # Send MoveStage action to Stage
    def send_cmd(self, x, z):
        # Send command to stage (convert mm to m)
        self.robot_idle = False
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x*0.001)
        goal_msg.z = float(z*0.001)
        goal_msg.eps = 0.0001
        self.get_logger().info('Send goal request... Control u: x=%f, z=%f' % (x, z))

        # Wait for action server
        self.action_client.wait_for_server()
        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        goal_msg.x = goal_msg.x*1000
        goal_msg.z = goal_msg.z*1000
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # Check if MoveStage action was accepted 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Get MoveStage action finish message (Result)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.robot_idle = True
            self.sensor_buffer = True
            self.buffer_index = 0
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x*1000))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

                                 
########################################################################
### Auxiliar functions ###
########################################################################

# Function: find_registration
# DO: From two sets of N 3D points in two different reference frames, find the best fit
#       in the LS-sense for the transformation between them (translation and rotation in quaternion)
# Inputs: 
#   A: set of N 3D points in first frame (numpy array 3xN)
#   B: set of N 3D points in second frame (numpy array 3xN)
# Output:
#   x_reg: transformation from first to second frame (numpy array [x, y, z, qw, qx, qy, qz])
def find_registration(A, B):
    [d, n] = np.shape(A)

    #Mean Center Data
    Ac = np.mean(A.T,axis=0)
    Bc = np.mean(B.T,axis=0)
    A = A - np.matlib.repmat(Ac[:,None], 1, n)
    B = B - np.matlib.repmat(Bc[:,None], 1, n)

    #Calculate Optimal Rotation
    M = np.matmul(A, B.T)
    N = np.array([[M[0,0]+M[1,1]+M[2,2], M[1,2]-M[2,1],        M[2,0]-M[0,2],        M[0,1]-M[1,0]],\
                  [M[1,2]-M[2,1],        M[0,0]-M[1,1]-M[2,2], M[0,1]+M[1,0],        M[2,0]+M[0,2]],\
                  [M[2,0]-M[0,2],        M[0,1]+M[1,0],        M[1,1]-M[0,0]-M[2,2], M[1,2]+M[2,1]],\
                  [M[0,1]-M[1,0],        M[2,0]+M[0,2],        M[1,2]+M[2,1],        M[2,2]-M[0,0]-M[1,1]]])
    [w,v]=np.linalg.eig(N)
    ind=np.argmax(w)
    q = v[:,ind]                                                                #Rotation quaternion
    R = (q[0]**2-np.inner(q[1:4],q[1:4]))*np.eye(3) + 2*np.outer(q[1:4],q[1:4]) + \
        2*q[0]*np.array([[0,-q[3],q[2]],[q[3],0,-q[1]],[-q[2],q[1],0]])         #Rotation matrix

    #Calculate Optimal Translation
    t = Bc - np.matmul(R,Ac)

    x_reg = [t[0], t[1], t[2], q[0], q[1], q[2], q[3]] #final registration transform
    return x_reg

########################################################################

def main(args=None):
    rclpy.init(args=args)

    registration = Registration()

    # Initialize registration
    registration.get_logger().info('Registration Node' %  (registration.transform))
    while rclpy.ok():
        rclpy.spin_once(registration)
        if (registration.transform.size == 0): #No registration yet
            registration.get_registration()
        else:
            if (registration.robot_idle == True):
                registration.get_logger().info('Registration procedure successfully completed! Press Ctrl + C to quit' )
                input()
                break
   
    rclpy.spin(registration)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    registration.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
