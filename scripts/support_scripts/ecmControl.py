# Import the Client from ambf_client package
from PyKDL import Vector, Rotation, Frame, dot
from ambf_client import Client
import time
import PyKDL
import math
import re
import pytest

def get_handler(_client, rigid_body_names, rigid_body_name):
	res = [x for x in rigid_body_names if re.search(rigid_body_name, x)]
	if len(res) == 0:
		return None

	return _client.get_obj_handle(rigid_body_name)

def activate_handler(handler, rigid_body_name):
	if handler is None:
		print('handler not found for ', rigid_body_name)
		return

	if handler.get_num_joints() == 0:
		print('handler has not joints ', rigid_body_name)
		return

	print('activating handler for ', rigid_body_name)
	handler.set_joint_pos(0, 0)
	time.sleep(0.2)

def get_all_joint_pos_wrapper(handler):
	if handler is None:
		return []

	return handler.get_all_joint_pos()

def set_joint_pos_wrapper(handler, joint_name, joint_pos):
	if handler is None:
		return
	
	handler.set_joint_pos(joint_name, joint_pos)
	time.sleep(0.2)


def get_joint_names_wrapper(handler):
	if handler is None:
		return []

	return handler.get_joint_names()

def vectors_to_dict(res, joints, angles):
	# res = {joints[i]: angles[i] for i in range(len(joints))}

	for key in list(joints):
		for value in list(angles):
			res[key] = value
			angles.remove(value)
			break 

	return res

def get_frame(name, handler):
	if handler is None:
		return
	
	Q = handler.get_rot()

	R_n_w = PyKDL.Rotation.Quaternion(Q.x, Q.y, Q.z, Q.w)
	
	P = handler.get_pos()
	P_n_w = Vector(P.x, P.y, P.z)
	T_n_w = Frame(R_n_w, P_n_w)
	return T_n_w

def validate_frame(T, p_ref):
	if T == None:
		return

	# m = T.M
	p = T.p

	assert pytest.approx(p.x(), 0.1) == p_ref.x()
	assert pytest.approx(p.y(), 0.1) == p_ref.y()
	assert pytest.approx(p.z(), 0.1) == p_ref.z()

def print_frame(name, T_n_w):
	print(name)
	print(T_n_w)
	print("---------------\n")

# def print_frame(name, handler):
# 	if handler is None:
# 		return
	
# 	Q = handler.get_rot()

# 	R_n_w = PyKDL.Rotation.Quaternion(Q.x, Q.y, Q.z, Q.w)
	
# 	P = handler.get_pos()
# 	P_n_w = Vector(P.x, P.y, P.z)
# 	T_n_w = Frame(R_n_w, P_n_w)

# 	print(name)
# 	print(Q)
# 	print(T_n_w)
# 	print("---------------\n")

print("_RandomPose_V1")
# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

# print('\n\n----')
# input("We can see what objects the client has found. Press Enter to continue...")
# You can print the names of objects found. We should see all the links found
# print(_client.get_obj_names())
rigid_body_names = _client.get_obj_names()

# Lets get a handle to PSM and ECM, as we can see in the printed
# object names, 'ecm/baselink' and 'psm/baselink' should exist
# print(_client.get_obj_names())
# object_names = ['World', '/ambf/env/Plane', '/ambf/env/lights/light1', '/ambf/env/lights/light2', '/ambf/env/cameras/default_camera', '/ambf/env/ecm/baselink', '/ambf/env/ecm/pitchbacklink', '/ambf/env/ecm/toollink', '/ambf/env/ecm/pitchtoplink', '/ambf/env/ecm/target_fk', '/ambf/env/ecm/target_ik']

# subs = 'ecm/baselink'
# to get string with substring 
# res = [x for x in object_names if re.search(subs, x)]
# print(res)
baselink = 'ecm/baselink'
yawlink = 'ecm/yawlink'
pitchbacklink = 'ecm/pitchbacklink'
pitchbottomlink = 'ecm/pitchbottomlink'
pitchendlink = 'ecm/pitchendlink'
pitchfrontlink = 'ecm/pitchfrontlink'
pitchtoplink = 'ecm/pitchtoplink'
maininsertionlink = 'ecm/maininsertionlink'
toollink = 'ecm/toollink'

baselink_handler 					= get_handler(_client, rigid_body_names, baselink)
yawlink_handler 					= get_handler(_client, rigid_body_names, yawlink)
pitchbacklink_handler 		= get_handler(_client, rigid_body_names, pitchbacklink)
pitchbottomlink_handler 	= get_handler(_client, rigid_body_names, pitchbottomlink)
pitchendlink_handler 			= get_handler(_client, rigid_body_names, pitchendlink)
pitchfrontlink_handler 		= get_handler(_client, rigid_body_names, pitchfrontlink)
pitchtoplink_handler 			= get_handler(_client, rigid_body_names, pitchtoplink)
maininsertionlink_handler = get_handler(_client, rigid_body_names, maininsertionlink)
toollink_handler 					= get_handler(_client, rigid_body_names, toollink)

activate_handler(					baselink_handler, baselink)
activate_handler(					 yawlink_handler, yawlink)
activate_handler(		 pitchbacklink_handler, pitchbacklink)
activate_handler(	 pitchbottomlink_handler, pitchbottomlink)
activate_handler(			pitchendlink_handler, pitchendlink)
activate_handler(		pitchfrontlink_handler, pitchfrontlink)
activate_handler(			pitchtoplink_handler, pitchtoplink)
activate_handler(maininsertionlink_handler, maininsertionlink)
activate_handler(					toollink_handler, toollink)


for i in range(15):
	if i < 10:
# for i in range(1):
# 	if i < 1:

		baselink_joint_angles 					= get_all_joint_pos_wrapper(baselink_handler)
		# yawlink_joint_angles  					= get_all_joint_pos_wrapper(yawlink_handler)
		# pitchbacklink_joint_angles  		= get_all_joint_pos_wrapper(pitchbacklink_handler)
		# pitchbottomlink_joint_angles  	= get_all_joint_pos_wrapper(pitchbottomlink_handler)
		# pitchendlink_joint_angles  			= get_all_joint_pos_wrapper(pitchendlink_handler)
		pitchfrontlink_joint_angles  		= get_all_joint_pos_wrapper(pitchfrontlink_handler)
		pitchtoplink_joint_angles  			= get_all_joint_pos_wrapper(pitchtoplink_handler)
		# maininsertionlink_joint_angles  = get_all_joint_pos_wrapper(maininsertionlink_handler)
		toollink_joint_angles  					= get_all_joint_pos_wrapper(toollink_handler)

	set_joint_pos_wrapper(baselink_handler, 		 				  "baselink-yawlink", math.pi / 4.0)
	set_joint_pos_wrapper(baselink_handler, 				 "yawlink-pitchbacklink", math.pi / 4.0)
	# set_joint_pos_wrapper(baselink_handler, 				 "yawlink-pitchfrontlink", math.pi / 4.0)
	set_joint_pos_wrapper(baselink_handler, "pitchendlink-maininsertionlink", 0.1)
	set_joint_pos_wrapper(baselink_handler, 		"maininsertionlink-toollink", math.pi / 4.0)

joint_name_angles = {}

joint_name_angles = vectors_to_dict(joint_name_angles, 
	get_joint_names_wrapper(baselink_handler), baselink_joint_angles)

# joint_name_angles = vectors_to_dict(joint_name_angles, 
# 	get_joint_names_wrapper(yawlink_handler), yawlink_joint_angles)

# joint_name_angles = vectors_to_dict(joint_name_angles, 
# 	get_joint_names_wrapper(pitchbacklink_handler), pitchbacklink_joint_angles)

# joint_name_angles = vectors_to_dict(joint_name_angles, 
# 	get_joint_names_wrapper(pitchbottomlink_handler), pitchbottomlink_joint_angles)

# joint_name_angles = vectors_to_dict(joint_name_angles, 
# 	get_joint_names_wrapper(pitchendlink_handler), pitchendlink_joint_angles)

joint_name_angles = vectors_to_dict(joint_name_angles, 
	get_joint_names_wrapper(pitchfrontlink_handler), pitchfrontlink_joint_angles)

joint_name_angles = vectors_to_dict(joint_name_angles, 
	get_joint_names_wrapper(pitchtoplink_handler), pitchtoplink_joint_angles)

# joint_name_angles = vectors_to_dict(joint_name_angles, 
# 	get_joint_names_wrapper(maininsertionlink_handler), maininsertionlink_joint_angles)

joint_name_angles = vectors_to_dict(joint_name_angles, 
	get_joint_names_wrapper(toollink_handler), toollink_joint_angles)

print(len(joint_name_angles))


joint_order_rbdl = ["baselink-yawlink", "yawlink-pitchbacklink", "pitchbacklink-pitchbottomlink", 
"pitchbottomlink-pitchendlink", "yawlink-pitchfrontlink", "pitchfrontlink-pitchbottomlink", 
"pitchfrontlink-pitchtoplink", "pitchtoplink-pitchendlink", 
"pitchendlink-maininsertionlink", "maininsertionlink-toollink" ]

for body_id in range(len(joint_order_rbdl)):
	rbdl_joint_name = joint_order_rbdl[body_id]
	if rbdl_joint_name in joint_name_angles:
		joint_angle = joint_name_angles[rbdl_joint_name]
		# print(body_id, f'{rbdl_joint_name:>30}', joint_angle)
		print(f"// {body_id}, {rbdl_joint_name:>30}, {joint_angle}")

print("\n")

for body_id in range(len(joint_order_rbdl)):
	rbdl_joint_name = joint_order_rbdl[body_id]
	if rbdl_joint_name in joint_name_angles:
		joint_angle = joint_name_angles[rbdl_joint_name]
		# print("Q[", body_id, "] = ", joint_angle)
		# print(f"The band for {name} is {band} out of 10")
		print(f"Q[{body_id}] = {joint_angle};")

		# for name in joint_name_angles:
			# print(f'{name:>30}', joint_name_angles[name])

# print_frame(				 baselink, baselink_handler)
# print_frame(	     	  yawlink, yawlink_handler)
# print_frame(		pitchbacklink, pitchbacklink_handler)
# print_frame(  pitchbottomlink, pitchbottomlink_handler)
# print_frame(		 pitchendlink, pitchendlink_handler)
# print_frame(	 pitchfrontlink, pitchfrontlink_handler)
# print_frame(		 pitchtoplink, pitchtoplink_handler)
# print_frame(maininsertionlink, maininsertionlink_handler)
# print_frame(				 toollink, toollink_handler)

T_w_baselink 					= get_frame(				 baselink, baselink_handler)
T_w_yawlink 					= get_frame(	     	  yawlink, yawlink_handler)
T_w_pitchbacklink 		= get_frame(		pitchbacklink, pitchbacklink_handler)
T_w_pitchbottomlink 	= get_frame(  pitchbottomlink, pitchbottomlink_handler)
T_w_pitchendlink 			= get_frame(		 pitchendlink, pitchendlink_handler)
T_w_pitchfrontlink 		= get_frame(	 pitchfrontlink, pitchfrontlink_handler)
T_w_pitchtoplink 			= get_frame(		 pitchtoplink, pitchtoplink_handler)
T_w_maininsertionlink = get_frame(maininsertionlink, maininsertionlink_handler)
T_w_toollink 					= get_frame(				 toollink, toollink_handler)

# print("yawlink-pitchbacklink", math.pi / 4.0)
# validate_frame(T_w_baselink, 			Vector(0.4999,     -0.3901,      -0.599))
# validate_frame(T_w_pitchbacklink, Vector(0.499999,   -0.774282,   -0.599737))
# validate_frame(T_w_pitchtoplink, 	Vector(0.499917,   -0.583113,    -0.29479))
# validate_frame(T_w_toollink, 			Vector(0.499971,   -0.134195,   -0.342508))

print_frame(				 baselink, T_w_baselink)
print_frame(	     	  yawlink, T_w_yawlink)
print_frame(		pitchbacklink, T_w_pitchbacklink)
print_frame(  pitchbottomlink, T_w_pitchbottomlink)
print_frame(		 pitchendlink, T_w_pitchendlink)
print_frame(	 pitchfrontlink, T_w_pitchfrontlink)
print_frame(		 pitchtoplink, T_w_pitchtoplink)
print_frame(maininsertionlink, T_w_maininsertionlink)
print_frame(				 toollink, T_w_toollink)