import rclpy
import numpy as np
import time
import math
import os 

from scipy.io import savemat

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage
from scipy.optimize import minimize
from trajcontrol.sensor_processing import INSERTION_STEP
from trajcontrol.estimator import get_angles


SAFE_LIMIT = 6.0    # Maximum control output delta from entry point [mm]
DEPTH_MARGIN = 1.5  # Final insertion length margin [mm]

class ControllerMPC(Node):

    def __init__(self):
        super().__init__('controller_mpc')

        #Declare node parameters
        self.declare_parameter('insertion_length', -100.0) #Insertion length parameter
        self.declare_parameter('H', 4) #Insertion length parameter
        self.declare_parameter('filename', 'my_data') #Name of file where data values are saved
        self.filename = os.path.join(os.getcwd(),'src','trajcontrol','data',self.get_parameter('filename').get_parameter_value().string_value + '_pred.mat') #String with full path to file

        #Topics from sensor processing node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning
        self.subscription_robot = self.create_subscription(PoseStamped, '/sensor/base', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from sensor processing node
        #Topics from UI (currently sensor processing node is doing the job)
        self.subscription_target = self.create_subscription(PointStamped, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Topics from estimator node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.jacobian_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Stored values
        self.tip = np.empty(shape=[0,5])            # Current needle tip x and z (from aurora)
        self.stage_initial = np.empty(shape=[0,3])  # Stage initial position
        self.stage = np.empty(shape=[0,3])          # Current stage position
        self.target = np.empty(shape=[0,5])         # Target 
        self.cmd = np.empty(shape=[0,3])            # Control output to the robot stage
        self.depth = 0.0                            # Current insertion depth
        self.robot_idle = True                      # Stage move action status
        self.Jc = np.zeros((5,3))

        self.insertion_length = self.get_parameter('insertion_length').get_parameter_value().double_value
        self.H = self.get_parameter('H').get_parameter_value().integer_value
        self.ns = math.floor(self.insertion_length/INSERTION_STEP)
        self.get_logger().info('MPC 3 horizon for this trial: H = %i \n Total insertion: %i steps' %(self.H, self.ns))

        # Prediction (save to mat file)
        self.u_pred = np.zeros((self.ns,self.H,2))
        self.y_pred = np.zeros((self.ns,self.H,5))

    # Get current base pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z])
        self.depth = robot.position.y
        if (self.stage_initial.size == 0):
            # Set stage initial position
            self.stage_initial = np.array([robot.position.x, robot.position.y, robot.position.z])
            self.cmd = np.copy(self.stage_initial) + np.array([0, INSERTION_STEP, 0]) #First command is to make one push without moving the base
            self.get_logger().info('Stage initial: (%f, %f, %f) ' % (self.stage_initial[0], self.stage_initial[1], self.stage_initial[2]))

            # Control output limits
            limit_x = (float(self.stage_initial[0])-SAFE_LIMIT, float(self.stage_initial[0])+SAFE_LIMIT)
            limit_z = (float(self.stage_initial[2])-SAFE_LIMIT, float(self.stage_initial[2])+SAFE_LIMIT)
            self.limit = [limit_x, limit_z]
            self.get_logger().info('Stage limit: %s'%(self.limit))

    # Get current target (only once)
    def target_callback(self, msg):
        if (self.target.size == 0):
            target = msg.point
            self.target = np.array([target.x, target.y, target.z, 0.0, 0.0])
            self.get_logger().info('Target: (%f, %f, %f, %f, %f) ' % (self.target[0], self.target[1], self.target[2], self.target[3], self.target[4]))

    # Get current needle tip from sensor processing node
    # tip = [x_tip, y_tip, z_tip, angle_horiz, angle_vert]  
    def tip_callback(self, msg):
        quat = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        angles = get_angles(quat)
        self.tip = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, angles[0], angles[1]])

    # Get current Jacobian matrix from Estimator node
    def jacobian_callback(self, msg):
        J = np.asarray(CvBridge().imgmsg_to_cv2(msg))
        self.Jc = J.copy()
        if (self.robot_idle == True) and (self.target.size != 0) and (self.tip.size != 0):
            self.send_cmd()

    def send_cmd(self):
    ########################################################################
                ## MPC Functions
    ########################################################################
        # Define process model
        # y = y0 + J(u-u0)
        # where y = tip, u = base
        def process_model(y0, u0, u, Jc):
            delta_u = np.array([float(u[0]-u0[0]), INSERTION_STEP, float(u[1]-u0[1])])
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
            tg_xz = np.tile([self.target[0],self.target[2],self.target[3],self.target[4]],(H,1))    # Build target without depth and with angles
            y_hat_xz = np.array([y_hat[:,0],y_hat[:,2],y_hat[:,3],y_hat[:,4]]).T                    # Build tip prediction without depth
            # kerr = np.array([1.0, 1.0, 0.2, 0.2])           # Balance weight of errors in x/z [mm] and angles [rad] (kx=kz=1, kav=kah=0.2)
            # err2 = np.dot(np.power(y_hat_xz-tg_xz,2), kerr) # Calculate err2 for each step (err2 = kx*Xerr^2 + kz*Zerr^e + kav*AVerr^2 + kah*AHerr^2)
            # kstep = np.arange(H,0,-1)                       # Decreasing weights for different steps (more importance to close future) 
            # obj1 = np.matmul(kstep,err2)                    # Weights each step
            
            err2 = np.dot(np.power(y_hat_xz-tg_xz,2), np.array([1.0, 1.0, 3.5, 3.5]))
            obj1 = np.matmul(np.ones(H),err2)

            ##This considers only final step error
            # tg_xz = np.array([self.target[0],self.target[2]])   # Build target without depth
            # y_hat_xz = np.array([y_hat[-1,0],y_hat[-1,2]])      # Build last tip prediction without depth

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

            # Save prediction to mat file
            step = math.floor(self.depth/INSERTION_STEP)
            self.get_logger().info('=====> step = %i' % step)
            self.get_logger().info('=====> H = %i' % H)
            self.get_logger().info('=====> u_hat = %s' % u_hat)
            self.u_pred[step-1,0:H,:] = np.copy(u_hat)
            self.y_pred[step-1,0:H,:] = np.copy(y_hat)
            savemat(self.filename, {'u_pred':self.u_pred, 'y_pred':self.y_pred})

            #This considers only final tip and target
            tg_xz = np.array([self.target[0],self.target[2],self.target[3],self.target[4]]) # Build target without depth
            y_hat_xz = np.array([y_hat[-1,0],y_hat[-1,2],y_hat[-1,3],y_hat[-1,4]])          # Build last tip prediction without depth

            err = y_hat_xz-tg_xz
            return err 

    ########################################################################

        # Calculate error to target
        error = self.tip - self.target  

        # MPC Initialization
        step = math.floor(self.depth/INSERTION_STEP)
        H = min(self.H, self.ns - step)
        
        if (H > 0):         # Continue insertion steps
            u0 = np.array([self.cmd[0], self.cmd[2]])
            u_hat = np.tile(u0, (H,1))   # Initial control guess using last cmd value (vector with remaining horizon size)

            # Initial objective
            # self.get_logger().info('Initial SSE Objective: %f' % (objective(u_hat)))  # calculate cost function with initial guess

            # MPC calculation
            start_time = time.time()
            solution = minimize(objective, u_hat, method='SLSQP', bounds=self.limit*H)    # optimizes the objective function
            u = np.reshape(solution.x,(H,2), order='C')                                 # reshape solution (minimize flattens it)
            end_time = time.time()
            cost = objective(u)

            # Summarize the result
            self.get_logger().info('Success : %s' % solution['message'])
            self.get_logger().info('Status : %s' % solution['message'])
            self.get_logger().info('Total Evaluations: %d' % solution['nfev'])
            self.get_logger().info('Final SSE Objective: %f' % (cost)) # calculate cost function with optimization result
            self.get_logger().info('Elapsed time: %f' % (end_time-start_time))
            self.get_logger().info('Solution: %s' % (u)) # calculate cost function with optimization result

            # Update controller output
            self.cmd[0] = u[0,0]
            self.cmd[1] = self.cmd[1]+INSERTION_STEP
            self.cmd[2] = u[0,1]

            ## Keeping this just to be on the safe side (but should not be necessary)
            # Limit control output to maximum SAFE_LIMIT[mm] around entry stage_initial
            self.cmd[0] = min(self.cmd[0], self.stage_initial[0]+SAFE_LIMIT)
            self.cmd[0] = max(self.cmd[0], self.stage_initial[0]-SAFE_LIMIT)
            self.cmd[2] = min(self.cmd[2], self.stage_initial[2]+SAFE_LIMIT)
            self.cmd[2] = max(self.cmd[2], self.stage_initial[2]-SAFE_LIMIT)

            # Test for stage limits
            self.cmd[0] = min(self.cmd[0], 0.0)
            self.cmd[0] = max(self.cmd[0], -95.0)
            self.cmd[2] = min(self.cmd[2], 95.0)
            self.cmd[2] = max(self.cmd[2], 0.0)

            # Expected final error
            exp_err = expected_error(u)
            self.get_logger().info('Expected final error: (%f, %f, %f, %f) ' % (exp_err[0], exp_err[1], exp_err[2], exp_err[3]))

        else:   # Finished all insertion steps
            self.cmd[0] = self.stage[0]
            self.cmd[2] = self.stage[2]
            u = np.array([[self.stage[0], self.stage[2]]])

        # # TO MAKE INSERTIONS WITHOUT COMPENSATION (DELETE AFTER)
        # self.cmd[0] = self.stage_initial[0]
        # self.cmd[2] = self.stage_initial[2]
    
        # Print values
        self.get_logger().info('Applying trajectory compensation... DO NOT insert the needle now\nTip: (%f, %f, %f) \
            \nTarget: (%f, %f, %f) \nError: (%f, %f, %f) \nDeltaU: (%f, %f)  \nCmd: (%f, %f) \nStage: (%f, %f)' % (self.tip[0],\
            self.tip[1], self.tip[2], self.target[0], self.target[1], self.target[2], error[0], error[1], error[2],\
            u[0,0] - self.stage[0], u[0,1] - self.stage[2], self.cmd[0], self.cmd[2], self.stage[0], self.stage[2]))    

        # Send command to stage
        self.robot_idle = False
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(self.cmd[0]*0.001)
        goal_msg.z = float(self.cmd[2]*0.001)
        goal_msg.eps = 0.0001

        self.action_client.wait_for_server() # Waiting for action server        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # Publish cmd
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"
        msg.point = Point(x=self.cmd[0], y=self.cmd[1], z=self.cmd[2])
        self.publisher_control.publish(msg)


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
            self.get_logger().info('Goal succeeded! Result: %f, %f' %(result.x*1000, result.z*1000))
            self.get_logger().info('Tip: (%f, %f, %f)'   % (self.tip[0], self.tip[1], self.tip[2]))
             # Check if max depth reached
            # if (abs(self.tip[1]-self.target[1]) <= DEPTH_MARGIN): 
            if (abs(self.depth) >= abs(self.target[1])):
                self.robot_idle = False
                self.get_logger().info('ATTENTION: Insertion depth reached! Please stop insertion')                
            else:
                self.robot_idle = True
                self.get_logger().info('Depth count: %.1fmm. Please insert %.1fmm more, then hit SPACE' % (self.stage[1], INSERTION_STEP))      
        else:
            self.get_logger().info('Goal failed with status: %s' %(result.status))


def main(args=None):
    rclpy.init(args=args)

    controller_mpc = ControllerMPC()

    # global P
    # global C
    # P = controller_mpc.get_parameter('P').get_parameter_value().integer_value
    # C = controller_mpc.get_parameter('C').get_parameter_value().integer_value

    rclpy.spin(controller_mpc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()