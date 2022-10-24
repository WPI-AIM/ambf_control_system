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
base = 'base'
link1 = 'link1'
link2 = 'link2'
link3 = 'link3'
link4 = 'link4'
link5 = 'link5'
link6 = 'link6'
link7 = 'link7'

base_handler  = get_handler(_client, rigid_body_names, base)
link1_handler = get_handler(_client, rigid_body_names, link1)
link2_handler = get_handler(_client, rigid_body_names, link2)
link3_handler = get_handler(_client, rigid_body_names, link3)
link4_handler = get_handler(_client, rigid_body_names, link4)
link5_handler = get_handler(_client, rigid_body_names, link5)
link6_handler = get_handler(_client, rigid_body_names, link6)
link7_handler = get_handler(_client, rigid_body_names, link7)


activate_handler( base_handler, base)
activate_handler(link1_handler, link1)
activate_handler(link2_handler, link2)
activate_handler(link3_handler, link3)
activate_handler(link4_handler, link4)
activate_handler(link5_handler, link5)
activate_handler(link6_handler, link6)
activate_handler(link7_handler, link7)


for i in range(15):
	if i < 10:
# for i in range(1):
# 	if i < 1:

		base_joint_angles = get_all_joint_pos_wrapper(base_handler)
		# yawlink_joint_angles  					= get_all_joint_pos_wrapper(yawlink_handler)
		# pitchbacklink_joint_angles  		= get_all_joint_pos_wrapper(pitchbacklink_handler)
		# pitchbottomlink_joint_angles  	= get_all_joint_pos_wrapper(pitchbottomlink_handler)
		# pitchendlink_joint_angles  			= get_all_joint_pos_wrapper(pitchendlink_handler)
		# pitchfrontlink_joint_angles  		= get_all_joint_pos_wrapper(pitchfrontlink_handler)
		# pitchtoplink_joint_angles  			= get_all_joint_pos_wrapper(pitchtoplink_handler)
		# maininsertionlink_joint_angles  = get_all_joint_pos_wrapper(maininsertionlink_handler)
		# toollink_joint_angles  					= get_all_joint_pos_wrapper(toollink_handler)
	set_joint_pos_wrapper(base_handler,  "base-link1", 0)
	set_joint_pos_wrapper(base_handler, "link1-link2", 0)
	set_joint_pos_wrapper(base_handler, "link2-link3", 0)
	set_joint_pos_wrapper(base_handler, "link3-link4", 0)
	set_joint_pos_wrapper(base_handler, "link4-link5", 0)
	set_joint_pos_wrapper(base_handler, "link5-link6", 0)
	set_joint_pos_wrapper(base_handler, "link6-link7", 0)

joint_name_angles = {}

joint_name_angles = vectors_to_dict(joint_name_angles, 
	get_joint_names_wrapper(base_handler), base_joint_angles)

# # joint_name_angles = vectors_to_dict(joint_name_angles, 
# # 	get_joint_names_wrapper(yawlink_handler), yawlink_joint_angles)

# # joint_name_angles = vectors_to_dict(joint_name_angles, 
# # 	get_joint_names_wrapper(pitchbacklink_handler), pitchbacklink_joint_angles)

# # joint_name_angles = vectors_to_dict(joint_name_angles, 
# # 	get_joint_names_wrapper(pitchbottomlink_handler), pitchbottomlink_joint_angles)

# # joint_name_angles = vectors_to_dict(joint_name_angles, 
# # 	get_joint_names_wrapper(pitchendlink_handler), pitchendlink_joint_angles)

# # joint_name_angles = vectors_to_dict(joint_name_angles, 
# # 	get_joint_names_wrapper(pitchfrontlink_handler), pitchfrontlink_joint_angles)

# # joint_name_angles = vectors_to_dict(joint_name_angles, 
# # 	get_joint_names_wrapper(pitchtoplink_handler), pitchtoplink_joint_angles)

# # joint_name_angles = vectors_to_dict(joint_name_angles, 
# # 	get_joint_names_wrapper(maininsertionlink_handler), maininsertionlink_joint_angles)

# # joint_name_angles = vectors_to_dict(joint_name_angles, 
# # 	get_joint_names_wrapper(toollink_handler), toollink_joint_angles)

print(len(joint_name_angles))
for name in joint_name_angles:
	print(f'{name:>30}', joint_name_angles[name])



T_w_base 	= get_frame( base, base_handler)
T_w_link1 = get_frame(link1, link1_handler)
T_w_link2 = get_frame(link2, link2_handler)
T_w_link3 = get_frame(link3, link3_handler)
T_w_link4 = get_frame(link4, link4_handler)
T_w_link5 = get_frame(link5, link5_handler)
T_w_link6 = get_frame(link6, link6_handler)
T_w_link7 = get_frame(link7, link7_handler)

# # print(T_w_pitchtoplink)
# validate_frame(T_w_baselink, 			Vector(0.4999,     -0.3901,      -0.599))
# validate_frame(T_w_pitchbacklink, Vector(0.499999,   -0.764536,   -0.608716))
# validate_frame(T_w_pitchtoplink, 	Vector(0.499325,   -0.666444,   -0.262765))
# validate_frame(T_w_toollink, 			Vector(0.500146,   -0.209318,   -0.276304))

print_frame( base, T_w_base)
print_frame(link1, T_w_link1)
print_frame(link2, T_w_link2)
print_frame(link3, T_w_link3)
print_frame(link4, T_w_link4)
print_frame(link5, T_w_link5)
print_frame(link6, T_w_link6)
print_frame(link7, T_w_link7)