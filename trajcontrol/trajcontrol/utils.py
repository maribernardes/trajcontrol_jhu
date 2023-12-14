import rclpy
import numpy as np
import quaternion
import math




#             error = self.target - self.tip[0:3]
#             # Print values
#             self.get_logger().info('\n****** STEP #%i ******\nInsertion depth: %f\nTarget: (%f, %f, %f)\nTip: (%f, %f, %f)\
#                 \nError: (%f, %f, %f / %f (%f deg), %f (%f deg))\nJ: %s\nStage: (%f, %f, %f)\nCmd: (%f, %f, %f)\nReached: (%.4f, %.4f, %.4f)\n*********************' \
#                 %(self.step, self.depth, \
#                 self.target[0], self.target[1], self.target[2],\
#                 self.tip[0],self.tip[1], self.tip[2],\
#                 error[0], error[1], error[2], self.tip[3], math.degrees(self.tip[3]), self.tip[4], math.degrees(self.tip[4]),\
#                 self.Jc, \
#                 self.stage[0], self.stage[1], self.stage[2],\
#                 self.cmd[0], self.cmd[1], self.cmd[2], \
#                 result.x, result.y, result.z)
#             )


# #Array of data
# header = ['Timestamp sec', 'Timestamp nanosec', \
#     'Step', \
#     'Insertion_depth', \
#     'Skin_entry_r x', 'Skin_entry_r y', 'Skin_entry_r z', \
#     'Target_r x', 'Target_r y', 'Target_r z', \
#     'Tip_n x', 'Tip_n y', 'Tip_n z', 'Tip_n qw', 'Tip_n qx', 'Tip_n qy', 'Tip_n qz' \
#     'Tip_r x', 'Tip_r y', 'Tip_r z', 'Tip_r qw', 'Tip_r qx', 'Tip_r qy', 'Tip_r qz', 'Tip_r sec', 'Tip_r nanosec', \
#     'Err x', 'Err y', 'Err z', 'Err h', 'Err v', \
#     'Stage_r x', 'Stage_r y', 'Stage_r z', 'Stage_r qw', 'Stage_r qx', 'Stage_r qy', 'Stage_r qz', \
#     'J00', 'J01', 'J02', \
#     'J10', 'J11', 'J12', \
#     'J20', 'J21', 'J22', \
#     'J30', 'J31', 'J32', \
#     'J40', 'J41', 'J42', \
#     'Control x', 'Control y', 'Control z', \
# ]
        
#         with open(self.filename, 'w', newline='', encoding='UTF8') as f: # open the file in the write mode
#             writer = csv.writer(f)  # create the csv writer
#             writer.writerow(header) # write a row to the csv file

#         #Last data received
#         self.target_r = [0,0,0, 0,0]             #target point + sec nanosec (sensor_processing - UI)
#         self.skin_entry_r = [0,0,0, 0,0]        #skin entry point + sec nanosec (sensor_processing - UI)
#         self.tip_n = [0,0,0,0,0,0,0, 0,0]     #tip from /needle/state/current_shape (x, y, z, qw, qx, qy, qz) + sec nanosec (shape_sensing)
#         self.tip_r = [0,0,0,0,0,0,0, 0,0]        #/sensor/tip (x, y, z, qw, qx, qy, qz) + sec nanosec (sensor_processing)
#         self.base_n = [0,0,0,0,0,0,0, 0,0] #/stage/state/needle_pose (x, y(depth), z, 1, 0, 0, 0) + sec nanosec (sensor_processing)
#         self.base_r = [0,0,0,0,0,0,0, 0,0]       #/sensor/base (x, y(depth), z, 1, 0, 0, 0) + sec nanosec (sensor_processing)
#         self.J = [0,0,0, \
#                   0,0,0, \
#                   0,0,0, \
#                   0,0,0, \
#                   0,0,0]            # /needle/state/jacobian Matrix (estimator)
#         self.Jtime = [0,0]          # Jacobian sec nanosec
#         self.cmd = [0,0,0, 0,0]     # /stage/control/cmd (x,y,z) + sec nanosec (sensor_processing)
#         self.stage = [0,0, 0,0]     # stage/state/pose (x,z) + sec nanosec  (stage_control)
#         self.depth = [0, 0,0]       # /arduino/depth (z) + sec nanosec  (depth_measurement)
#         self.key = [0, 0,0]         # /keyboard/key (k) + sec nanosec  (keypress)
#         self.get_logger().info('Log data will be saved at %s' %(self.filename))   



# #Save data do file
# def write_file_callback(filename, clock, target, skin_entry, tip, stage, J, cmd):
#     now = self.get_clock().now().to_msg()
    
#     data = [now.sec, now.nanosec, \
#         self.target_r[0], self.target_r[1], self.target_r[2], self.target_r[3], self.target_r[4],\
#         self.skin_entry_r[0], self.skin_entry_r[1], self.skin_entry_r[2], self.skin_entry_r[3], self.skin_entry_r[4],\
#         self.tip_n[0], self.tip_n[1], self.tip_n[2], self.tip_n[3], self.tip_n[4], self.tip_n[5], self.tip_n[6], self.tip_n[7], self.tip_n[8], \
#         self.tip_r[0], self.tip_r[1], self.tip_r[2], self.tip_r[3], self.tip_r[4], self.tip_r[5], self.tip_r[6], self.tip_r[7], self.tip_r[8], \
#         self.base_n[0], self.base_n[1], self.base_n[2], self.base_n[3], self.base_n[4], self.base_n[5], self.base_n[6], self.base_n[7], self.base_n[8],\
#         self.base_r[0], self.base_r[1], self.base_r[2], self.base_r[3], self.base_r[4], self.base_r[5], self.base_r[6], self.base_r[7], self.base_r[8],\
#         self.J[0], self.J[1], self.J[2], \
#         self.J[3], self.J[4], self.J[5], \
#         self.J[6], self.J[7], self.J[8], \
#         self.J[9], self.J[10], self.J[11], \
#         self.J[12], self.J[13], self.J[14], self.Jtime[0] , self.Jtime[1], \
#         self.cmd[0], self.cmd[1], self.cmd[2], self.cmd[3], self.cmd[4], \
#         self.stage[0], self.stage[1], self.stage[2], self.stage[3], \
#         self.depth[0], self.depth[1], self.depth[2], \
#         self.key[0], self.key[1], self.key[2], \
#         ]
#     # Reset keyboard
#     self.key = [0, 0,0]

#     with open(self.filename, 'a', newline='', encoding='UTF8') as f: # open the file in append mode
#         writer = csv.writer(f) # create the csv writer
#         writer.writerow(data)  # append a new row to the existing csv file    


########################################################################

# Function: get_angles(q)
# DO: Get needle angles in horizontal and vertical plane
# Inputs: 
#   q: quaternion (numpy array [qw, qx, qy, qz])
# Output:
#   angles: angle vector (numpy array [horiz, vert])
def get_angles(q):

    #Define rotation quaternion
    q_tf= np.quaternion(q[0], q[1], q[2], q[3])
    #Set needle insertion axis (in needle frame)
    u_dir = np.quaternion(0, 0, 0, 1) # Needle insertion in Z-axis (needle frame)
    # Rotate direction vector with rotation quaternion
    v = q_tf*u_dir*q_tf.conj()
    #Angles are components in x (horizontal) and z(vertical)
    horiz = math.atan2(v.x, v.y)
    vert = math.atan2(v.z, math.sqrt(v.x**2+v.y**2))
    return np.array([horiz, vert])

########################################################################

# Function: pose_transform
# DO: Transform pose to new reference frame
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf*q_orig
    p_new = q_tf*p_orig*q_tf.conj() + p_tf

    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

# Function: pose_inv_transform
# DO: Transform pose to new reference frame with inverse transform 
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_inv_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf.conj()*q_orig
    p_new = q_tf.conj()*(p_orig-p_tf)*q_tf
    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################