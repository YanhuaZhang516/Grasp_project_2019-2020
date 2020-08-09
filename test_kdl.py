from urdf2casadi import converter
import casadi as cs

path_urdf = "/home/ren/mypy/RobotTaskSim/robosuite/models/assets/bullet_data/sawyer_description/urdf/sawyer_arm.urdf"

fk_dict = converter.from_file("right_arm_base_link", "right_l4", path_urdf)
print(fk_dict.keys())

q = fk_dict["q"]
# Upper limits of the joint values
q_upper = fk_dict["upper"]
# Lower limits of the joint values
q_lower = fk_dict["lower"]
# Joint names
joint_names = fk_dict["joint_names"]

print("Number of joints:", q.size()[0])
print("Upper limits:", q_upper)
print("Lower limits:", q_lower)
print("Joint names:", joint_names)

# should give ['q', 'upper', 'lower', 'dual_quaternion_fk', 'joint_names', 'T_fk', 'joint_list', 'quaternion_fk']
forward_kinematics = fk_dict["T_fk"]
print(forward_kinematics([0.3, 0.3, 0.3, 0., 0.3]))
