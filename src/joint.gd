@tool
extends Node3D

@export var constrain_position = true
@export var joint_S_Rot = [[1,0,0], [0,0,1]] :
	get:
		return joint_S_Rot
	set(value):
		joint_S_Rot = []
		
		for row in value:
			if row == null:
				row = [0,0,0]
			joint_S_Rot.append(row)


func get_joint_S():
	var rot_dim = len(joint_S_Rot)
	var lin_dim = 0 if constrain_position else 3
	var col_count = rot_dim + lin_dim
	
	var joint_S_ = []
	joint_S_.resize(6*col_count)
	joint_S_.fill(0)
	
	for row in range(3):
		for col in range(rot_dim):
			joint_S_[row * col_count + col] = joint_S_Rot[col][row]
	
	if not constrain_position:
		for i in range(3):
			var row = 3 + i
			var col = rot_dim + i
			joint_S_[row * col_count + col] = 1
	
	return DenseMatrix.from_packed_array(joint_S_, 6, col_count)
	
