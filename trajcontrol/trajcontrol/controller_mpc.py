import rclpy
import numpy as np
import time
import math
import os 
import quaternion

from std_msgs.msg import Int8

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped, Point, Quaternion, Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from smart_control_interfaces.srv import GetPoint, GetPose
from trajcontrol_interfaces.srv import GetJacobian
from smart_control_interfaces.action import MoveStage

from scipy.optimize import minimize
from scipy.io import savemat

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory
from functools import partial
from .utils import *

#########################################################################
#
# MPC Controller Node
#
# Description:
# This node implements the MPC control.
# It loads the planning (skin_entry and target). Then, at each keyboard SPACE input, it updates the stage and tip positions, 
# requests a Jacobian to the estimator, calculates the step_wise control and sends it to the robot action server
# TODO: Implement version with target update (for tracked target)
#
# Subscribes:   
# '/keyboard/key'           (std_msgs.msg.Int8)
#
# Service client:    
# '/planning/get_skin_entry'    (smart_control_interfaces.srv.GetPoint)  - robot frame
# '/planning/get_target'        (smart_control_interfaces.srv.GetPoint)  - robot frame
# '/stage/get_position'         (smart_control_interfaces.srv.GetPoint)  - robot frame
# '/needle/get_tip'             (smart_control_interfaces.srv.GetPose)   - robot frame
# '/estimator/get_jacobian'     (trajcontrol_interfaces.srv.GetJacobian) - robot frame
#
# Action client:
# 'stage/move'              (smart_control_interfaces.action.MoveStage) - robot frame
#
##########################################################################

SAFE_LIMIT = 6.0    # Maximum control output delta from entry point [mm]
GALIL_ERR = 0.5

class ControllerMPC(Node):

    def __init__(self):
        super().__init__('controller_mpc')

        #Declare node parameters
        self.declare_parameter('H', 4)                          # Horizon size
        self.declare_parameter('insertion_step', 10.0)          # Insertion step parameter
        self.declare_parameter('filename', 'mpc_data')    # Name of file where data values are saved

        
    #### Stored variables ###################################################
        self.skin_entry = np.empty(shape=[0,3])  # Skin entry position
        self.target = np.empty(shape=[0,3])      # Target [x, y, z]
        self.goal = np.empty(shape=[0,5])        # Control goal [target_x, target_y, _target_z, 0.0, 0.0]
        
        self.tip_pose = np.empty(shape=[0,7])    # Current tip pose   [x, y, z, qw, qx, qy, qz] in robot frame
        self.tip = np.empty(shape=[0,5])         # Current tip input  [x, y, z, angle_h, angle_v]
        self.stage = np.empty(shape=[0,3])       # Current base input [x, y, z]
        self.cmd = np.empty(shape=[0,3])         # Control output to the robot stage
        self.Jc = np.zeros((5,3))                # Jacobian matrix

        self.depth = 0.0                         # Current insertion depth = stage[1] - skin_entry[1]
        self.step = 0                            # Current insertion step      

        self.wait_stage = False       # Flag to wait for tip value
        self.wait_tip = False         # Flag to wait for base value
        self.wait_jacobian = False    # Flag to wait for jacobian value
        self.wait_finish = False      # Flag to wait for insertion to finish
        self.robot_idle = False       # Robot status

    #### Action/service clients ###################################################

        # /needle/get_tip and /stage/get_position should be in the same callbackgroup
        jacobian_inputs_callgroup = MutuallyExclusiveCallbackGroup() 
        # OBS: Node will not spin until servers are available
        # /planning/get_target Service client
        self.target_service_client = self.create_client(GetPoint, '/planning/get_target', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.target_service_client.service_is_ready():
            self.get_logger().warn('/planning/get_target service not available, waiting...')
        while not self.target_service_client.wait_for_service(timeout_sec=5.0):
            pass     
        # /planning/get_skin_entry Service client
        self.skin_entry_service_client = self.create_client(GetPoint, '/planning/get_skin_entry', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.skin_entry_service_client.service_is_ready():
            self.get_logger().warn('/planning/get_skin_entry service not available, waiting...')
        while not self.skin_entry_service_client.wait_for_service(timeout_sec=5.0):
            pass      
        # /stage/move Action client
        self.move_action_client = ActionClient(self, MoveStage, '/stage/move', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.move_action_client.server_is_ready():
            self.get_logger().warn('/stage/move action not available, waiting...')
        while not self.move_action_client.wait_for_server(timeout_sec=5.0):
            pass
        # /stage/get_position Service client
        self.stage_position_service_client = self.create_client(GetPoint, '/stage/get_position', callback_group=jacobian_inputs_callgroup)
        if not self.stage_position_service_client.service_is_ready():
            self.get_logger().warn('/stage/get_position service not available, waiting...')
        while not self.stage_position_service_client.wait_for_service(timeout_sec=5.0):
            pass          
        # /needle/get_tip Service client
        self.tip_service_client = self.create_client(GetPose, '/needle/get_tip', callback_group=jacobian_inputs_callgroup)
        if not self.tip_service_client.service_is_ready():
            self.get_logger().warn('/needle/get_tip service not available, waiting...')
        while not self.tip_service_client.wait_for_service(timeout_sec=5.0):
            pass  
        # /estimator/get_jacobian Service client
        self.jacobian_service_client = self.create_client(GetJacobian, '/estimator/get_jacobian', callback_group=jacobian_inputs_callgroup)
        if not self.jacobian_service_client.service_is_ready():
            self.get_logger().warn('/estimator/get_jacobian service not available, waiting...')
        while not self.jacobian_service_client.wait_for_service(timeout_sec=5.0):
            pass  
        self.get_logger().info('Action and services available')

    #### Initialize variables ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        self.insertion_step = self.get_parameter('insertion_step').get_parameter_value().double_value        
        self.H = self.get_parameter('H').get_parameter_value().integer_value

        # Set .mat filename and path
        trajcontrol_share_directory = get_package_share_directory('trajcontrol') 
        self.filename = os.path.join(trajcontrol_share_directory,'data',self.get_parameter('filename').get_parameter_value().string_value + '.mat') #String with full path to file
        self.get_logger().info('Saving experiment data to %s' %self.filename)

        # Request planning (initialized only once)
        self.target = self.get_target()
        self.skin_entry = self.get_skin_entry()
        self.goal = np.array([self.target[0], self.target[1], self.target[2], 0.0, 0.0])

        # Define number of control steps
        self.insertion_length = self.target[1] - self.skin_entry[1]
        self.ns = math.ceil(self.insertion_length/self.insertion_step)
        self.get_logger().info('\nMPC horizon: H = %i\nInsertion length: %.4fmm\nTotal insertion: %i steps' %(self.H, self.insertion_length, self.ns))

        self.u_pred = np.zeros((self.H, 2)) # MPC Prediction
        self.y_pred = np.zeros((self.H, 5)) # MPC Prediction

        # Define motion safety limits
        limit_x = (float(self.skin_entry[0])-SAFE_LIMIT, float(self.skin_entry[0])+SAFE_LIMIT)
        limit_z = (float(self.skin_entry[2])-SAFE_LIMIT, float(self.skin_entry[2])+SAFE_LIMIT)
        self.limit = [limit_x, limit_z]

        # Experiment data (save to .mat file)
        self.depth_data = np.zeros((1, 1, 0))       # Insertion depth before control action
        self.stage_data = np.zeros((1, 3, 0))       # Stage input before control action
        self.tip_data = np.zeros((1, 5, 0))         # Tip input before control action
        self.tip_pose_data = np.zeros((1, 7, 0))    # Tip pose before control action
        self.jacobian_data = np.zeros((5, 3, 0))    # Jacobian matrix
        self.cmd_data = np.zeros((1, 3, 0))         # Step control action
        self.u_pred_data = np.zeros((self.H, 2, 0)) # MPC Prediction
        self.y_pred_data = np.zeros((self.H, 5, 0)) # MPC Prediction

        # Set robot as ready
        self.robot_idle = True 


    #### Subscribed topics ###################################################

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

#### Listening callbacks ###################################################

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (msg.data == 32):
            if (self.robot_idle is True):
                self.get_logger().warn('Starting step #%i...' %(self.step+1))                
                self.next_step()
            else:
                self.get_logger().info('Motion not available')
        elif (msg.data == 10):
            if (self.wait_finish is True):
                self.update_tip()

#### Service client functions ###################################################

    # Get target (blocking service request)
    def get_target(self):
        request = GetPoint.Request()
        future = self.target_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        target = future.result()
        if target.valid is True:
            return np.array([target.x, target.y, target.z])
        else:
            self.get_logger().error('Invalid target')
            return None

    # Get skin_entry (blocking service request)
    def get_skin_entry(self):
        request = GetPoint.Request()
        future = self.skin_entry_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        skin_entry = future.result()
        if skin_entry.valid is True:
            return np.array([skin_entry.x, skin_entry.y, skin_entry.z])
        else:
            self.get_logger().error('Invalid skin_entry')
            return None

    # Update stage (non blocking service request)
    def update_stage(self):
        request = GetPoint.Request()
        future = self.stage_position_service_client.call_async(request)
        # When stage request done, do callback
        future.add_done_callback(partial(self.update_stage_callback))
        return future.result()

    # Update tip (non blocking service request)
    def update_tip(self):
        request = GetPose.Request()
        future = self.tip_service_client.call_async(request)
        # When tip request done, do callback
        future.add_done_callback(partial(self.update_tip_callback))
        return future.result()
    
    # Update jacobian (non blocking service request)
    def update_jacobian(self):
        request = GetJacobian.Request()
        request.base = Point(x=self.stage[0], y=self.stage[1], z=self.stage[2])
        Z_p = Point(x=self.tip[0], y=self.tip[1], z=self.tip[2])
        Z_q = Quaternion(w=self.tip[3], x=self.tip[4], y=0.0, z=0.0)
        request.tip = Pose(position=Z_p, orientation=Z_q)  
        future = self.jacobian_service_client.call_async(request)
        # When jacobian request done, do callback
        future.add_done_callback(partial(self.update_jacobian_callback))
        return future.result()

    # Update stage service response message (Response)
    def update_stage_callback(self, future):
        try:
            stage = future.result()
            if stage.valid is True:
                self.stage = np.array([stage.x, stage.y, stage.z])
                self.depth = self.stage[1] - self.skin_entry[1]
                self.wait_stage = False
            else:
                self.get_logger().error('Invalid stage position')
        except Exception as e:
            self.get_logger().error('Stage call failed: %r' %(e,))
        if (self.wait_stage is False) and (self.wait_tip is False) and (self.robot_idle is True):
            self.move_step()

    # Update tip service response message (Response)
    def update_tip_callback(self, future):
        try:
            tip = future.result()
            if tip.valid is True:
                q = np.array([tip.qw, tip.qx, tip.qy, tip.qz])
                angles = get_angles(q)
                self.tip = np.array([tip.x, tip.y, tip.z, angles[0],angles[1]])
                self.tip_pose = np.array([tip.x, tip.y, tip.z, tip.qw, tip.qx, tip.qy, tip.qz])
                self.wait_tip = False
            else:
                self.get_logger().error('Invalid tip pose')
        except Exception as e:
            self.get_logger().error('Tip call failed: %r' %(e,))
        if (self.wait_finish is True):
            self.finish_insertion()
        elif (self.wait_stage is False) and (self.wait_tip is False) and (self.robot_idle is True):
            self.move_step()

    # Update jacobian service response message (Response)
    def update_jacobian_callback(self, future):
        try:
            jacobian = future.result()
            self.Jc = np.asarray(CvBridge().imgmsg_to_cv2(jacobian.image_matrix))
            self.wait_jacobian = False
        except Exception as e:
            self.get_logger().error('Jacobian call failed: %r' %(e,))
        self.mpc_control()

#### Internal functions ###################################################

    def next_step(self):
        # Update jacobian request inputs (tip and base)
        # Responses trigger move_step
        self.wait_stage = True
        self.wait_tip = True
        self.update_stage()
        self.update_tip()

    def move_step(self):
        self.robot_idle = False
        self.wait_jacobian = True
        self.step += 1
        # Update Jacobian
        # Response triggers mpc_control
        self.update_jacobian()

    def finish_insertion(self):
        # Save last values into .mat file
        self.depth_data = np.dstack((self.depth_data , self.depth))
        self.stage_data = np.dstack((self.stage_data , self.stage))
        self.tip_data = np.dstack((self.tip_data , self.tip))
        self.tip_pose_data = np.dstack((self.tip_pose_data , self.tip_pose))
        error = self.tip - self.goal
        savemat(self.filename, {'depth':self.depth_data, 'target':self.target, 'skin_entry':self.skin_entry,\
                'stage': self.stage_data, 'tip': self.tip_data, 'jacobian': self.jacobian_data, 'tip_pose':self.tip_pose_data, \
                'u_pred':self.u_pred, 'y_pred':self.y_pred, 'cmd':self.cmd_data})
        # Print last values
        self.get_logger().info('\n****** FINAL ******\nTarget: (%f, %f, %f)\nTip: (%f, %f, %f) \
            \nError: (%f, %f, %f / %f (%f deg), %f (%f deg))\nStage: (%f, %f, %f)\n*********************' \
            %(self.target[0], self.target[1], self.target[2],\
            self.tip[0],self.tip[1], self.tip[2],\
            error[0], error[1], error[2], error[3], math.degrees(error[3]), error[4], math.degrees(error[4]),\
            self.stage[0], self.stage[1], self.stage[2])
        )
        self.get_logger().warn('Press Ctrl+C to finish.')

    def mpc_control(self):
        #### MPC functions ###################################################
        
        # Define process model
        # y = y0 + J(u-u0)
        # where y = tip, u = base
        def process_model(y0, u0, u, Jc):
            delta_u = np.array([float(u[0]-u0[0]), self.insertion_step, float(u[1]-u0[1])])
            y = y0 + np.matmul(Jc,delta_u)
            return y

        # Define objective function
        def objective(u_hat):
            H = math.floor(u_hat.size/2)                # How many steps to go
            u_hat = np.reshape(u_hat,(H,2), order='C')  # Reshape u_hat (minimize flattens it)
            y_hat = np.zeros((H,5))                     # Initialize y_hat for H next steps
            # Initialize prediction
            y_hat0 = np.copy(self.tip)
            u_hat0 = np.copy([self.stage[0],self.stage[2]])
            # Simulate prediction horizon
            for k in range(0,H):
                yp = process_model(y_hat0, u_hat0, u_hat[k], self.Jc)
                # Save predicted variable
                y_hat[k] = np.copy(yp)
                # Update initial condition for next prediction step
                y_hat0 = np.copy(y_hat[k])
                u_hat0 = np.copy(u_hat[k])
            ## Minimization objectives
            # Objective 1: Trajectory error (x/z position and angles)
            tg_xz = np.tile([self.goal[0],self.goal[2],self.goal[3],self.goal[4]],(H,1))    # Build target without depth and with angles
            y_hat_xz = np.array([y_hat[:,0],y_hat[:,2],y_hat[:,3],y_hat[:,4]]).T                    # Build tip prediction without depth
            err2 = np.dot(np.power(y_hat_xz-tg_xz,2), np.array([1.0, 1.0, 3.5, 3.5]))
            obj1 = np.matmul(np.ones(H),err2)
            # Objective 2: delta at the base
            u_temp = np.vstack((np.copy([self.stage[0],self.stage[2]]), u_hat))
            delta_u_hat = np.diff(u_temp, axis=0)
            obj2 = np.linalg.norm(delta_u_hat)          # Delta_u
            # Combine both objectives
            obj = 1.0*obj1 + 0.0*obj2
            return obj 

        # Calculates expected insertion final error (from prediction)
        def expected_error(u_hat):
            H = math.floor(u_hat.size/2)                # How many steps to go
            u_hat = np.reshape(u_hat,(H,2), order='C')  # Reshape u_hat (minimize flattens it)
            y_hat = np.zeros((H,5))                     # Initialize y_hat for H next steps
            # Initialize prediction
            y_hat0 = np.copy(self.tip)
            u_hat0 = np.copy([self.stage[0],self.stage[2]])
            # Simulate prediction horizon
            for k in range(0,H):
                yp = process_model(y_hat0, u_hat0, u_hat[k], self.Jc)
                # Save predicted variable
                y_hat[k] = np.copy(yp)
                # Update initial condition for next prediction step
                y_hat0 = np.copy(y_hat[k])
                u_hat0 = np.copy(u_hat[k])

            # Save prediction to array
            self.get_logger().info('=====> step = %i' % self.step)
            self.get_logger().info('=====> H = %i' % H)
            self.get_logger().info('=====> u_hat = %s' % u_hat)
            self.u_pred[0:H,:] = np.copy(u_hat)
            self.y_pred[0:H,:] = np.copy(y_hat)
            #This considers only final tip and target
            tg_xz = np.array([self.goal[0],self.goal[2],self.goal[3],self.goal[4]]) # Build target without depth
            y_hat_xz = np.array([y_hat[-1,0],y_hat[-1,2],y_hat[-1,3],y_hat[-1,4]])  # Build last tip prediction without depth
            err = y_hat_xz-tg_xz
            return err 

        ########################################################################

        # Calculate error to target
        error = self.tip - self.goal  
        self.get_logger().info('Error = %s' %error)
        # MPC Initialization
        H = min(self.H, self.ns - self.step)

        # First command is to make one push without moving the base  
        if (self.step == 1):
            self.get_logger().info('First step is a straight push')
            self.cmd = np.copy(self.skin_entry) + np.array([0, self.insertion_step, 0])  
            self.get_logger().info('Cmd = %s' %self.cmd) 
            u = np.array([[self.cmd[0], self.cmd[2]]])     
        elif (H > 0):         # Continue insertion steps
            u0 = np.array([self.cmd[0], self.cmd[2]])
            u_hat = np.tile(u0, (H,1))   # Initial control guess using last cmd value (vector with remaining horizon size)
            
            # Initial objective
            self.get_logger().info('Initial SSE Objective: %f' % (objective(u_hat)))  # calculate cost function with initial guess

            # MPC calculation
            start_time = time.time()
            solution = minimize(objective, u_hat.flatten(), method='SLSQP', bounds=self.limit*H)    # optimizes the objective function
            u = np.reshape(solution.x, (H,2), order='C')                                            # reshape solution (minimize flattens it)
            end_time = time.time()
            cost = objective(u)

            # Summarize the result
            self.get_logger().info('Success : %s' % solution['message'])
            self.get_logger().info('Status : %s' % solution['message'])
            self.get_logger().info('Total Evaluations: %d' % solution['nfev'])
            self.get_logger().info('Final SSE Objective: %f' % (cost)) # calculate cost function with optimization result
            self.get_logger().info('Elapsed time: %f' % (end_time-start_time))
            self.get_logger().info('Solution: %s' % (u)) # calculate cost function with optimization result

            # Calculate step goal
            step_depth = self.skin_entry[1] + self.insertion_step*self.step
            # Update controller output
            self.cmd[0] = u[0,0]
            self.cmd[1] = min(self.target[1], step_depth)
            self.cmd[2] = u[0,1]
            
            ## Keeping this just to be on the safe side (but should not be necessary)
            # Limit control output to maximum SAFE_LIMIT[mm] around skin entry
            self.cmd[0] = min(self.cmd[0], self.skin_entry[0]+SAFE_LIMIT)
            self.cmd[0] = max(self.cmd[0], self.skin_entry[0]-SAFE_LIMIT)
            self.cmd[2] = min(self.cmd[2], self.skin_entry[2]+SAFE_LIMIT)
            self.cmd[2] = max(self.cmd[2], self.skin_entry[2]-SAFE_LIMIT)

            # Expected final error
            exp_err = expected_error(u)
            self.get_logger().info('Expected final control error: (%f, %f, %f, %f) ' % (exp_err[0], exp_err[1], exp_err[2], exp_err[3]))

        else:   # Finished all MPC steps
            self.get_logger().warn('Last MPC step')
            step_depth = self.skin_entry[1] + self.insertion_step*self.step
            self.cmd[0] = self.stage[0]
            self.cmd[1] = min(self.target[1], step_depth)
            self.cmd[2] = self.stage[2]
            u = np.array([[self.stage[0], self.stage[2]]])

        # Print values
        self.get_logger().info('Applying trajectory compensation... \
            \nDeltaU: (%.4f, %.4f)'\
                %(u[0,0] - self.stage[0], u[0,1] - self.stage[2]))    
        self.move_stage(self.cmd[0], self.cmd[1], self.cmd[2])

        # Save .mat file
        self.depth_data = np.dstack((self.depth_data , self.depth))
        self.stage_data = np.dstack((self.stage_data , self.stage))
        self.tip_data = np.dstack((self.tip_data , self.tip))
        self.tip_pose_data = np.dstack((self.tip_pose_data , self.tip_pose))
        self.jacobian_data = np.dstack((self.jacobian_data, self.Jc))
        self.cmd_data = np.dstack((self.cmd_data , self.cmd))
        self.u_pred_data = np.dstack((self.u_pred_data, self.u_pred))
        self.y_pred_data = np.dstack((self.y_pred_data, self.y_pred))

        savemat(self.filename, {'depth':self.depth_data, 'target':self.target, 'skin_entry':self.skin_entry,\
                'stage': self.stage_data, 'tip': self.tip_data, 'jacobian': self.jacobian_data, 'tip_pose':self.tip_pose_data, \
                'u_pred':self.u_pred, 'y_pred':self.y_pred, 'cmd':self.cmd_data})


#### Action client functions ###################################################

    def move_stage(self, x, y, z):
        # Send command to stage (in mm)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        goal_msg.eps = GALIL_ERR # in mm
        self.get_logger().debug('Move request: %.4f, %.4f, %.4f' % (x, y, z))
        # Wait for action server
        self.move_action_client.wait_for_server()        
        self.send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.move_response_callback)

    # Check if MoveStage action was accepted 
    def move_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Move rejected :(')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_move_result_callback)

    # Get MoveStage action finish message (Result)
    def get_move_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            error = self.tip - self.goal
            # Print values
            self.get_logger().info('\n****** STEP #%i ******\nInsertion depth: %f\nTarget: (%f, %f, %f)\nTip: (%f, %f, %f)\
                \nError: (%f, %f, %f / %f (%f deg), %f (%f deg))\nStage: (%f, %f, %f)\nCmd: (%f, %f, %f)\
                \nReached: (%.4f, %.4f, %.4f)\n*********************' \
                %(self.step, self.depth, \
                self.target[0], self.target[1], self.target[2],\
                self.tip[0],self.tip[1], self.tip[2],\
                error[0], error[1], error[2], error[3], math.degrees(error[3]), error[4], math.degrees(error[4]),\
                self.stage[0], self.stage[1], self.stage[2],\
                self.cmd[0], self.cmd[1], self.cmd[2], \
                result.x, result.y, result.z)
            )
            # Update stage
            self.stage = np.array([result.x, result.y, result.z])
            self.depth = self.stage[1] - self.skin_entry[1]
            if ((self.target[1]-self.tip[1]) <= (self.insertion_step+GALIL_ERR)):
                self.robot_idle = False
                self.wait_finish = True
                self.get_logger().warn('End of insertion. Hit ENTER when needle is updated')
            else:
                self.robot_idle = True 
                self.get_logger().warn('Hit SPACE for next step')
        elif result.error_code == 1:
            self.get_logger().info('Move failed: TIMETOUT')


########################################################################

def main(args=None):
    rclpy.init(args=args)

    controller_mpc = ControllerMPC()

    executor = MultiThreadedExecutor()

    controller_mpc.get_logger().warn('***** Ready to INSERT - MPC Control *****')
    controller_mpc.get_logger().warn('Use SPACE to signal each insertion step')

    try:
        rclpy.spin(controller_mpc, executor=executor)
    except KeyboardInterrupt:
        controller_mpc.get_logger().info('Keyboard interrupt, shutting down.\n')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()