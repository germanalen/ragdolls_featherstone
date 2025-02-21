@tool
extends Node3D

@export var ground_mesh: Node3D

const EPS = 0.00001
var CAPSULE_MASS = 1
var CAPSULE_RADIUS = 0.4
var CAPSULE_HEIGHT = 2
var b0_f = []
var b0_b = []


@onready var bodies = []
@onready var joints = []
var parent = []
var joint_S = []
var q = []
var q_dot = []

var default_body_transforms = []

var default_joint_transforms = []


func init_tree():
	bodies = [$body0]
	joints = [null]
	parent = [null]
	init_subtree(0)
	

func init_subtree(parent_i):
	var p: Node3D = bodies[parent_i]
	var p_joints = p.get_children()
	
	for i in range(len(p_joints)):
		var joint = p_joints[i]
		var child_body = joint.get_child(0)
		
		bodies.append(child_body)
		joints.append(joint)
		parent.append(parent_i)
		
		init_subtree(len(bodies)-1)

func _ready() -> void:
	init_tree()
	
	default_body_transforms.resize(len(bodies))
	default_joint_transforms.resize(len(bodies))
	joint_S.resize(len(bodies))
	q.resize(len(bodies))
	q_dot.resize(len(bodies))
	b0_f.resize(len(bodies))
	b0_b.resize(len(bodies))
	
	for i in range(len(bodies)):
		default_body_transforms[i] = bodies[i].global_transform
		
		if joints[i]:
			default_joint_transforms[i] = joints[i].global_transform
			
			joint_S[i] = joints[i].call('get_joint_S')
			
			
			var q_zeros_ = [0,0,0, 0,0,0]
			q_zeros_.resize(joint_S[i].get_dimensions()[1])
			q[i] = matrix_vector_from_arr(q_zeros_)
			q_dot[i] = matrix_vector_from_arr(q_zeros_)
			
		
		b0_f[i] = matrix_vector_from_arr([0,0,0, 0,0,0])
		b0_b[i] = matrix_vector_from_arr([0,0,0, 0,0,0])

func sort_contacts_by_depth(a, b):
	return a.depth > b.depth


func _process(_delta: float) -> void:
	var _config = DebugDraw3D.new_scoped_config().set_no_depth_test(true)
	
	
	
	if Engine.is_editor_hint(): return
	
	var step_all = Input.is_action_just_pressed("ui_accept") || Input.is_key_pressed(KEY_P)
	
	var delta = 0.016
	
	if step_all:
		var aba_results = articulated_body_algorithm(delta)
		
		var contacts = generate_contacts(delta)
		contacts.sort_custom(sort_contacts_by_depth)
		
		for contact in contacts:
			DebugDraw3D.draw_sphere(contact.point, 0.01, Color.RED)
		
		#for interp_pass in range(1):
			#for contact in contacts:
				#var interp_q_dot = []
				#interp_q_dot.resize(len(q_dot))
				#for k in range(len(q_dot)):
					#if q_dot[k]:
						#interp_q_dot[k] = mscale(q_dot[k], 0)
				#
				#
				#if Input.is_action_just_pressed("KEY_B") or step_all or Input.is_action_just_pressed("KEY_N"):
					#interpenetration_resolution(contact, aba_results)
				#if Input.is_action_just_pressed("KEY_N") or step_all or Input.is_action_just_pressed("KEY_B"):
					#apply_impulses(aba_results, interp_q_dot, contacts)
					#
					#var fake_interp_resolution_delta = 1 # see the comment for interpenetration_resolution() definition
					#update_transforms(fake_interp_resolution_delta, interp_q_dot)
					##aba_results = articulated_body_algorithm(0)
		
		for contact in contacts:
			collision_response(contact, aba_results)
			apply_impulses(aba_results, q_dot, contacts)

		
		update_transforms(delta, q_dot)

# a contact will be added to the list if it's predicted to interpenetrate after delta time
# check for depth > 0 to see if it's an interpenetration
func generate_contacts(delta):
	var contacts = []
	
	# collision between bodies
	for body_i in range(1, len(bodies) - 1):
		for body_j in range(body_i + 1, len(bodies)):
			var dont_collide = (
					parent[body_i] == body_j or
					parent[body_j] == body_i or
					default_joint_transforms[body_i].origin.distance_to(default_joint_transforms[body_j].origin) < 0.01
				)
			if dont_collide:
				continue
			
			var world_T_i = bodies[body_i].global_transform
			var world_T_j = bodies[body_j].global_transform
			var contact = calculate_capsule_capsule_contact(world_T_i, CAPSULE_RADIUS, CAPSULE_HEIGHT, world_T_j, CAPSULE_RADIUS, CAPSULE_HEIGHT)
			
			contact.body1 = body_i
			contact.body2 = body_j
			
			var world_v_i = calculate_world_vel(body_i, q_dot)
			var world_v_j = calculate_world_vel(body_j, q_dot)
			var world_v_ij = msub(world_v_i, world_v_j)
			
			var world_T_point = Transform3D(create_basis_from_normal(contact.normal), contact.point)
			var point_X_world = spatial_coordinate_transform_motion(Transform3D.IDENTITY, world_T_point)
			var point_v_ij = mmul(point_X_world, world_v_ij)
			var point_v_ij_y = point_v_ij.to_packed_array()[4]
			
			if contact.depth + point_v_ij_y * delta > 0:
				#DebugDraw3D.draw_sphere(contact.point, 0.01, Color.PURPLE, 0.5)
				contacts.append(contact)
	
	
	# collision with ground
	for body_i in range(len(bodies)):
		var center = bodies[body_i].global_transform.origin
		var top = center + bodies[body_i].global_transform.basis.y * (CAPSULE_HEIGHT * 0.5 - CAPSULE_RADIUS)
		var bottom = center - bodies[body_i].global_transform.basis.y * (CAPSULE_HEIGHT * 0.5 - CAPSULE_RADIUS)
		#DebugDraw3D.draw_line(bottom, top, Color.RED)
		top.y -= CAPSULE_RADIUS
		bottom.y -= CAPSULE_RADIUS
		
		for point in [bottom, top]:
			var point_v_y
			if true:
				var world_v_c = calculate_world_vel(body_i, q_dot)
				
				var world_T_point = Transform3D(Basis.IDENTITY, point)
				
				var point_X_world = spatial_coordinate_transform_motion(Transform3D.IDENTITY, world_T_point)
				var point_v_c = mmul(point_X_world, world_v_c)
				var point_v_c_ = point_v_c.to_packed_array()
				point_v_y = point_v_c_[4]
			
			#DebugDraw3D.draw_line(contact_point, contact_point + v, Color.BLUE if v.y < 0 else Color.PURPLE, 1)
			
			#DebugDraw3D.draw_line(Vector3.ZERO, vo, Color.MIDNIGHT_BLUE)
			#DebugDraw3D.draw_line(Vector3.ZERO, w, Color.BLUE_VIOLET)
			var ground_y = ground_mesh.global_transform.origin.y
			if point.y + point_v_y * delta < ground_y:
				#DebugDraw3D.draw_sphere(point, 0.01, Color.PURPLE, 0.5)
				
				var contact = {}
				contact.point = point
				contact.body1 = body_i
				contact.body2 = -1
				contact.normal = Vector3(0, -1, 0)
				contact.depth = ground_y - point.y
				
				contacts.append(contact)
	
	return contacts


# creates a fake impulse to that would move the body back by interpenetration depth over 1 sec
func interpenetration_resolution(contact, aba_results):
	var contact_point: Vector3 = contact.point
	var body_a: int = contact.body1
	var body_b: int = contact.body2
	var normal: Vector3 = contact.normal
	var depth: float = contact.depth
	
	if depth > 0:
		var world_T_b0: Transform3D = bodies[0].global_transform
		var world_T_point = Transform3D(create_basis_from_normal(normal), contact_point)
		
		var p_I_A_c_inv_44_arr = []
		var p_I_A_c_arr_debug = []
		
		for body_i in [body_a, body_b]:
			var p_I_A_c_inv_44 = 0
			var p_I_A_c_debug
			if body_i != -1:
				var world_T_c = bodies[body_i].global_transform
				
				var c_I_A_c = aba_results.body_inertia[body_i]
				
				var c_X_p = spatial_coordinate_transform_motion(world_T_point, world_T_c)
				var p_X_c_force = spatial_coordinate_transform_force(world_T_c, world_T_point)
				
				var p_I_A_c = mmul(p_X_c_force, c_I_A_c, c_X_p)
				var p_I_A_c_inv = minv(p_I_A_c)
				var p_I_A_c_inv_ = p_I_A_c_inv.to_packed_array()
				p_I_A_c_inv_44 = p_I_A_c_inv_[6*4 + 4]
				p_I_A_c_debug = p_I_A_c
			p_I_A_c_inv_44_arr.append(p_I_A_c_inv_44)
			p_I_A_c_arr_debug.append(p_I_A_c_debug)
		
		var p_I_A_a_inv_44 = p_I_A_c_inv_44_arr[0]
		var p_I_A_b_inv_44 = p_I_A_c_inv_44_arr[1]
		
		#var desired_displacement = matrix_vector_from_arr([0,0,0, 0,-depth/(p_I_A_a_inv_44 + p_I_A_b_inv_44), 0])
		var p_impulse = matrix_vector_from_arr([0,0,0, 0,-depth/(p_I_A_a_inv_44 + p_I_A_b_inv_44), 0])
		var b0_X_p_force = spatial_coordinate_transform_force(world_T_point, world_T_b0)
		var b0_impulse = mmul(b0_X_p_force, p_impulse)
		b0_b[body_a] = b0_impulse
			
			
		if body_b != -1:
			b0_b[body_b] = mscale(b0_impulse,-1)

#TODO: precompute this
func calculate_world_vel(body_i, q_dot_in):
	var world_v_c = matrix_vector_from_arr([0,0,0, 0,0,0])
	
	while body_i > 0:
		var world_X_j = spatial_coordinate_transform_motion(joints[body_i].global_transform, Transform3D.IDENTITY)
		var j_v_bp = mmul(joint_S[body_i], q_dot_in[body_i])
		var world_v_bp = mmul(world_X_j, j_v_bp)
		world_v_c = madd(world_v_c, world_v_bp)
		body_i = parent[body_i]
	
	return world_v_c

func collision_response(contact, aba_results):
	
	var contact_point: Vector3 = contact.point
	var body_a: int = contact.body1
	var body_b: int = contact.body2
	var normal: Vector3 = contact.normal
	#var depth: float = contact.depth
	
	var world_v_ab = calculate_world_vel(body_a, q_dot)
	if body_b != -1:
		world_v_ab = msub(world_v_ab, calculate_world_vel(body_b, q_dot))
	
	var world_T_point = Transform3D(create_basis_from_normal(normal), contact_point)
	
	var point_X_world = spatial_coordinate_transform_motion(Transform3D.IDENTITY, world_T_point)
	
	var point_v_ab = mmul(point_X_world, world_v_ab)
	var point_v_ab_ = point_v_ab.to_packed_array()
	var point_v_ab_y = point_v_ab_[4]
	
	
	if point_v_ab_y > 0:
		var world_T_b0: Transform3D = bodies[0].global_transform
		
		var p_I_A_c_inv_44_arr = []
		
		for body_i in [body_a, body_b]:
			var p_I_A_c_inv_44 = 0
			if body_i != -1:
				var world_T_c = bodies[body_i].global_transform
				
				var c_I_A_c = aba_results.body_inertia[body_i]
				
				var c_X_p = spatial_coordinate_transform_motion(world_T_point, world_T_c)
				var p_X_c_force = spatial_coordinate_transform_force(world_T_c, world_T_point)
				
				var p_I_A_c = mmul(p_X_c_force, c_I_A_c, c_X_p)
				var p_I_A_c_inv = minv(p_I_A_c)
				var p_I_A_c_inv_ = p_I_A_c_inv.to_packed_array()
				p_I_A_c_inv_44 = p_I_A_c_inv_[6*4 + 4]
			p_I_A_c_inv_44_arr.append(p_I_A_c_inv_44)
		
		var p_I_A_a_inv_44 = p_I_A_c_inv_44_arr[0]
		var p_I_A_b_inv_44 = p_I_A_c_inv_44_arr[1]
		
		
		var restitution = 0.4
		var vel_normal_scale = 1.0 + restitution
		var p_impulse = matrix_vector_from_arr([0,0,0, 0,-point_v_ab_y/(p_I_A_a_inv_44 + p_I_A_b_inv_44) * vel_normal_scale, 0])
		var b0_X_p_force = spatial_coordinate_transform_force(world_T_point, world_T_b0)
		var b0_impulse = mmul(b0_X_p_force, p_impulse)
		b0_b[body_a] = b0_impulse
		
		
		if body_b != -1:
			b0_b[body_b] = mscale(b0_impulse,-1)
		

func articulated_body_algorithm(delta):
	
	var body_v_body = [] # body_v_body[i] is the spatial velocity of body i in frame of body i
	body_v_body.resize(len(bodies))
	body_v_body[0] = matrix_vector_from_arr([0,0,0, 0,0,0])
	var body_c = []
	body_c.resize(len(bodies))
	
	var cylinder_inertia3 = cylinder_inertia(CAPSULE_MASS, CAPSULE_HEIGHT, CAPSULE_RADIUS)
	var cm_inertia = spatial_inertia(cylinder_inertia3, CAPSULE_MASS)
	
	var body_inertia = []
	body_inertia.resize(len(bodies))
	
	var p = []
	p.resize(len(bodies))
	
	
	var world_T_0: Transform3D = bodies[0].global_transform
		
	for i in range(1, len(bodies)):
		var parent_i = parent[i]
		var world_T_parent: Transform3D = bodies[parent_i].global_transform
		var world_T_joint: Transform3D = joints[i].global_transform
		var world_T_child: Transform3D = bodies[i].global_transform
		
		var jXp = spatial_coordinate_transform_motion(world_T_parent, world_T_joint)
		var cXj = spatial_coordinate_transform_motion(world_T_joint, world_T_child)
		
		var j_v_cj = mmul(joint_S[i], q_dot[i])
		var c_v_cj = mmul(cXj, j_v_cj) # v_Ji in the book
		
		var p_v_p = body_v_body[parent_i]
		var p_v_j = p_v_p # joint is rigidly attached to parent => same spatial velocity
		var j_v_j = mmul(jXp, p_v_j)
		
		var c_v_j = mmul(cXj, j_v_j)
		var c_v_c = madd(c_v_j, c_v_cj)
		body_v_body[i] = c_v_c
		
		var j_v_j_crossm = spatial_cross_motion_matrix(j_v_j)
		var j_c = mmul(j_v_j_crossm, j_v_cj)
		var c_c = mmul(cXj, j_c)
		body_c[i] = c_c
		
		body_inertia[i] = cm_inertia
		
		
		var cX0_f = spatial_coordinate_transform_force(world_T_0, world_T_child)
		
		var c_v_c_crossf = spatial_cross_force_matrix(c_v_c)
		var pi = msub(mmul(c_v_c_crossf, body_inertia[i], c_v_c), mmul(cX0_f, b0_f[i]))
		p[i] = pi
	
	var D = []
	D.resize(len(bodies))
	var U = []
	U.resize(len(bodies))
	var u = []
	u.resize(len(bodies))
	
	for i in range(len(bodies)-1, 0, -1): # excluding body 0 on purpose
		var parent_i = parent[i]
		var world_T_parent: Transform3D = bodies[parent_i].global_transform
		var world_T_joint: Transform3D = joints[i].global_transform
		var world_T_child: Transform3D = bodies[i].global_transform
		
		var cXj = spatial_coordinate_transform_motion(world_T_joint, world_T_child)
		var cXp = spatial_coordinate_transform_motion(world_T_parent, world_T_child)
		
		var c_S = mmul(cXj, joint_S[i])
		
		var Ui = mmul(body_inertia[i], c_S)
		U[i] = Ui
		var Di = mmul(mT(c_S), Ui)
		D[i] = Di
		
		var ti_ = []
		ti_.resize(joint_S[i].get_dimensions()[1])
		ti_.fill(0)
		var ti = matrix_vector_from_arr(ti_) # no torques on joint "motors"
		var ui = msub(ti, mmul(mT(c_S), p[i]))
		u[i] = ui
		
		
		if parent_i != 0:
			var c_I_A_c = body_inertia[i]
			var c_I_a_c = msub(c_I_A_c, mmul(Ui, minv(Di), mT(Ui)))
			
			var c_c = body_c[i]
			var c_p_A_c = p[i]
			var c_p_a_c = madd(c_p_A_c, mmul(c_I_a_c, c_c), mmul(Ui, minv(Di), ui))
			
			var pXc_f = spatial_coordinate_transform_force(world_T_child, world_T_parent)
			var p_I_A_p = body_inertia[parent_i]
			p_I_A_p = madd(p_I_A_p, mmul(pXc_f, c_I_a_c, cXp))
			body_inertia[parent_i] = p_I_A_p
			
			var p_p_A_p = p[parent_i] # pen pineapple Apple pen
			p_p_A_p = madd(p_p_A_p, mmul(pXc_f, c_p_a_c))
			p[parent_i] = p_p_A_p
	
	
	if delta > EPS:
		var body_a_body = []
		body_a_body.resize(len(bodies))
		
		# fictitious upward acceleration on body 0
		# see p. 130
		body_a_body[0] = matrix_vector_from_arr([0,0,0, 0,10,0])
		
		
		for i in range(1, len(bodies)):
			var parent_i = parent[i]
			var world_T_parent: Transform3D = bodies[parent_i].global_transform
			var world_T_joint: Transform3D = joints[i].global_transform
			var world_T_child: Transform3D = bodies[i].global_transform
			
			var cXp = spatial_coordinate_transform_motion(world_T_parent, world_T_child)
			var cXj = spatial_coordinate_transform_motion(world_T_joint, world_T_child)
			
			var p_a_p = body_a_body[parent_i]
			var c_a_p = mmul(cXp, p_a_p)
			var c_c = body_c[i]
			var c_a_tick = madd(c_a_p, c_c) # a' in the book
			
			var c_S = mmul(cXj, joint_S[i])
			var Di = D[i]
			var Ui = U[i]
			var ui = u[i]
			
			
			var q_dot_dot = mmul(minv(Di), msub(ui, mmul(mT(Ui), c_a_tick)))
			
			var c_a_c = madd(c_a_tick, mmul(c_S, q_dot_dot))
			body_a_body[i] = c_a_c
			
			
			q_dot[i] = madd(q_dot[i], mscale(q_dot_dot, delta))
			
			# damping
			q_dot[i] = mscale(q_dot[i], 0.99)
			#var q_dot_len = sqrt(mmul(mT(q_dot[i]), q_dot[i]).to_packed_array()[0])
			#if q_dot_len > 10:
			#	q_dot[i] = mscale(q_dot[i], 10/q_dot_len)
		
	
	var aba_results = {}
	aba_results.body_inertia = body_inertia
	aba_results.U = U
	aba_results.D = D
	
	return aba_results
	
	

# adds the change in q dot due to impulse into dq_dot
func apply_impulses(aba_results, dq_dot, _debug_contacts):
	var U = aba_results.U
	var D = aba_results.D
	
	var world_T_b0: Transform3D = bodies[0].global_transform
	
	var b = [] # bias impulses
	b.resize(len(bodies))
	
	for i in range(1, len(bodies)):
		var world_T_child: Transform3D = bodies[i].global_transform
		
		var cX0_f = spatial_coordinate_transform_force(world_T_b0, world_T_child)
				
		var bi = mscale(mmul(cX0_f, b0_b[i]), -1)
		b[i] = bi
		# zero out the impulse after we took it into account
		b0_b[i] = mscale(b0_b[i], 0)
	
	for i in range(len(bodies)-1, 0, -1): # excluding body 0 on purpose
		var parent_i = parent[i]
		var world_T_parent: Transform3D = bodies[parent_i].global_transform
		var world_T_joint: Transform3D = joints[i].global_transform
		var world_T_child: Transform3D = bodies[i].global_transform
		
		var cXj = spatial_coordinate_transform_motion(world_T_joint, world_T_child)
		
		var c_S = mmul(cXj, joint_S[i])
		
		
		if parent_i != 0:
			var pXc_f = spatial_coordinate_transform_force(world_T_child, world_T_parent)
			
			# propagate impulse to parent
			var c_b_A_c = b[i]
			var c_b_a_c = msub(c_b_A_c, mmul(U[i], minv(D[i]), mT(c_S), c_b_A_c))
			var p_b_A_p = b[parent_i]
			p_b_A_p = madd(p_b_A_p, mmul(pXc_f, c_b_a_c))
			b[parent_i] = p_b_A_p
	
	
	var body_dv_impulse_body = [] # instantanious change in velocity due to impulse
	body_dv_impulse_body.resize(len(bodies))
	body_dv_impulse_body[0] = matrix_vector_from_arr([0,0,0, 0,0,0])
	
	for i in range(1, len(bodies)):
		var parent_i = parent[i]
		var world_T_parent: Transform3D = bodies[parent_i].global_transform
		var world_T_joint: Transform3D = joints[i].global_transform
		var world_T_child: Transform3D = bodies[i].global_transform
		
		var cXp = spatial_coordinate_transform_motion(world_T_parent, world_T_child)
		var cXj = spatial_coordinate_transform_motion(world_T_joint, world_T_child)
		
		var c_S = mmul(cXj, joint_S[i])
		var Di = D[i]
		var Ui = U[i]
		
		
		
		var p_dv_impulse_p = body_dv_impulse_body[parent_i]
		var c_dv_impulse_p = mmul(cXp, p_dv_impulse_p)
		
		var c_b_A_c = b[i]
		
		var c_dqdot_impulse_c = mmul(
			minv(Di),
			mscale(madd(
				mmul(mT(Ui), c_dv_impulse_p),
				mmul(mT(c_S), c_b_A_c)
			), -1)
		)
		
		var c_dv_impulse_c = madd(c_dv_impulse_p, mmul(c_S, c_dqdot_impulse_c))
		body_dv_impulse_body[i] = c_dv_impulse_c
		
		
		dq_dot[i] = madd(dq_dot[i], c_dqdot_impulse_c)
	
	

# q_dot_in: q_dot to use to update q and the resulting transforms
# pass "the q_dot" for integrating velocities
# pass interp_q_dot to resolve interpetetrations
func update_transforms(delta: float, q_dot_in):
	var joint_T_child_arr = []
	joint_T_child_arr.resize(len(bodies))
	
	for i in range(1, len(bodies)):
		var world_T_joint: Transform3D = joints[i].global_transform
		var world_T_child: Transform3D = bodies[i].global_transform
		var joint_T_child = world_T_joint.inverse() * world_T_child
		joint_T_child_arr[i] = joint_T_child
	
	for i in range(1, len(bodies)):
		var parent_i = parent[i]
		var default_world_T_parent: Transform3D = default_body_transforms[parent_i]
		var default_world_T_joint: Transform3D = default_joint_transforms[i]
		var default_world_T_child: Transform3D = default_body_transforms[i]
		
		# default transform of joint in parent space
		var default_parent_T_joint = default_world_T_parent.inverse() * default_world_T_joint
		
		
		# update joint transform
		var world_T_parent: Transform3D = bodies[parent_i].global_transform
		var world_T_joint: Transform3D = world_T_parent * default_parent_T_joint
		joints[i].global_transform = world_T_joint
		
		
		# update child transform
		var world_T_child: Transform3D = bodies[i].global_transform
		if true:
			var default_joint_T_child = default_world_T_joint.inverse() * default_world_T_child
			var default_jRc = default_joint_T_child.basis
			var default_j_r_cj = default_joint_T_child.origin
			
			var joint_T_child = joint_T_child_arr[i]
			var jTc = joint_T_child
			var jRc = jTc.basis
			var j_r_cj = jTc.origin
			
			var qi_dot = q_dot_in[i]
			var joint_Sq_dot = mmul(joint_S[i],qi_dot) # joint space
			var joint_Sq_dot_ = joint_Sq_dot.to_packed_array()
			var w = Vector3(joint_Sq_dot_[0], joint_Sq_dot_[1], joint_Sq_dot_[2])
			var v = Vector3(joint_Sq_dot_[3], joint_Sq_dot_[4], joint_Sq_dot_[5])
			
			# update jRc
			var angle = w.length() * delta
			var axis = w.normalized()
			if w.length() > 0.01 and axis.is_normalized():
				var delta_rot = Basis(axis, angle)
				jRc = delta_rot * jRc
			jRc = jRc.orthonormalized()
			
			var constrain_position = joints[i].get('constrain_position')
			if constrain_position:
				j_r_cj = jRc * default_jRc.transposed() * default_j_r_cj
			else:
				j_r_cj += (w.cross(j_r_cj) + v) * delta
			
			# update jTc
			jTc.basis = jRc
			jTc.origin = j_r_cj
			joint_T_child = jTc
			
			world_T_child = world_T_joint * joint_T_child
			bodies[i].global_transform = world_T_child




static func cylinder_inertia(mass, height, radius):
	var xz = (1.0/12.0) * (height**2 + 3 * radius**2) * mass
	var y = (1.0/2.0) * mass * radius**2
	
	var arr = [
		xz,0,0,
		0,y,0,
		0,0,xz
	]
	
	var inertia_c = DenseMatrix.from_packed_array(arr, 3, 3)
	
	return inertia_c

static func matrix_from_arr(arr : PackedFloat64Array, rows, cols) -> DenseMatrix:
	return DenseMatrix.from_packed_array(arr, rows, cols)

static func matrix_vector_from_arr(arr : PackedFloat64Array) -> DenseMatrix:
	return DenseMatrix.from_packed_array(arr, len(arr), 1)

static func matrix_from_vector3(v : Vector3) -> DenseMatrix:
	return DenseMatrix.from_packed_array([v[0],v[1],v[2]],3,1)

static func matrix_from_basis(b : Basis) -> DenseMatrix:
	var arr = [
		b[0][0], b[0][1], b[0][2],
		b[1][0], b[1][1], b[1][2],
		b[2][0], b[2][1], b[2][2]
	]
	
	# transposed because Godot internally stores basis in row major order
	return DenseMatrix.from_packed_array(arr, 3, 3).transposed()

static func vector3_from_matrix(m : DenseMatrix) -> Vector3:
	var dim = m.get_dimensions()
	assert(dim == Vector2i(3,1))
	var m_ = m.to_packed_array()
	return Vector3(m_[0],m_[1],m_[2])

static func skewsym_matrix(v : DenseMatrix) -> DenseMatrix:
	assert(v.get_dimensions() == Vector2i(3,1))
	
	var v_ = vector3_from_matrix(v)
	
	var arr = [
			0,-v_.z, v_.y,
		 v_.z,    0,-v_.x,
		-v_.y, v_.x,    0
	]
	
	return DenseMatrix.from_packed_array(arr, 3, 3)

static func spatial_cross_motion_matrix(v : DenseMatrix) -> DenseMatrix:
	assert(v.get_dimensions() == Vector2i(6,1))
	
	var v_ = v.to_packed_array()
	var wx = v_[0]
	var wy = v_[1]
	var wz = v_[2]
	var vox = v_[3]
	var voy = v_[4]
	var voz = v_[5]
	
	var arr = [
		   0, -wz,  wy,    0,   0,   0,
		  wz,   0, -wx,    0,   0,   0,
		 -wy,  wx,   0,    0,   0,   0,
		
		   0,-voz, voy,    0, -wz,  wy,
		 voz,   0,-vox,   wz,   0, -wx,
		-voy, vox,   0,  -wy,  wx,   0,
	]
	
	return DenseMatrix.from_packed_array(arr, 6, 6)


static func spatial_cross_force_matrix(v : DenseMatrix) -> DenseMatrix:
	var m = spatial_cross_motion_matrix(v).transposed()
	m.multiply_scaler_in_place(-1)
	return m




static func spatial_inertia(inertia_c : DenseMatrix, mass : float) -> DenseMatrix:
	assert(inertia_c.get_dimensions() == Vector2i(3,3))
	
	var I00 = inertia_c
	var I00_ = I00.to_packed_array()
	
	var arr = [
		I00_[0],I00_[1],I00_[2],        0,      0,      0,
		I00_[3],I00_[4],I00_[5],        0,      0,      0,
		I00_[6],I00_[7],I00_[8],        0,      0,      0,
		
			  0,      0,      0,     mass,      0,      0,
			  0,      0,      0,        0,   mass,      0,
			  0,      0,      0,        0,      0,   mass,
	]
	
	return DenseMatrix.from_packed_array(arr, 6, 6)


# transforms from A to B
# E is the basis of A in B
# r is (B origin - A origin) in A space
static func spatial_coordinate_transform_motion_Er(E : DenseMatrix, r : DenseMatrix) -> DenseMatrix:
	assert(E.get_dimensions() == Vector2i(3,3))
	assert(r.get_dimensions() == Vector2i(3,1))
	
	var neg_r_cross = skewsym_matrix(r)
	neg_r_cross.multiply_scaler_in_place(-1)
	
	var E_ = E.to_packed_array()
	var nrx_ = neg_r_cross.to_packed_array()
	
	var left_ = [
		E_[0],E_[1],E_[2],       0,    0,    0,
		E_[3],E_[4],E_[5],       0,    0,    0,
		E_[6],E_[7],E_[8],       0,    0,    0,
		
			0,    0,    0,   E_[0],E_[1],E_[2],
			0,    0,    0,   E_[3],E_[4],E_[5],
			0,    0,    0,   E_[6],E_[7],E_[8],
	]
	
	var right_ = [
			  1,      0,      0,   0,0,0,
			  0,      1,      0,   0,0,0,
			  0,      0,      1,   0,0,0,
		
		nrx_[0],nrx_[1],nrx_[2],   1,0,0,
		nrx_[3],nrx_[4],nrx_[5],   0,1,0,
		nrx_[6],nrx_[7],nrx_[8],   0,0,1
	]
	
	var left = DenseMatrix.from_packed_array(left_, 6, 6)
	var right = DenseMatrix.from_packed_array(right_, 6, 6)
	return left.multiply_dense(right)

# transforms from A to B
static func spatial_coordinate_transform_motion(frameA : Transform3D, frameB : Transform3D) -> DenseMatrix:
	# E is the basis of A in B
	# r is (B origin - A origin) in A space
	
	var E = matrix_from_basis(frameB.basis.inverse() * frameA.basis)
	var r = matrix_from_vector3(frameA.basis.inverse() * (frameB.origin - frameA.origin))
	return spatial_coordinate_transform_motion_Er(E,r)

# transforms from A to B
static func spatial_coordinate_transform_force(frameA : Transform3D, frameB : Transform3D) -> DenseMatrix:
	return spatial_coordinate_transform_motion(frameA, frameB).transposed().inverse()




static func matrix_scale(m : DenseMatrix, s : float) -> DenseMatrix:
	var res = m.clone()
	res.multiply_scaler_in_place(s)
	return res

static func mscale(m : DenseMatrix, s : float) -> DenseMatrix:
	return matrix_scale(m, s)

static func madd(m1 : DenseMatrix, m2 : DenseMatrix, 
	m3 : DenseMatrix = null, m4 : DenseMatrix = null, m5 : DenseMatrix = null, m6 : DenseMatrix = null, 
	m7 : DenseMatrix = null, m8 : DenseMatrix = null, m9 : DenseMatrix = null, m10 : DenseMatrix = null) -> DenseMatrix:
	
	var res = m1.add_dense(m2)
	
	if m3: res.add_dense_in_place(m3)
	if m4: res.add_dense_in_place(m4)
	if m5: res.add_dense_in_place(m5)
	if m6: res.add_dense_in_place(m6)
	if m7: res.add_dense_in_place(m7)
	if m8: res.add_dense_in_place(m8)
	if m9: res.add_dense_in_place(m9)
	if m10: res.add_dense_in_place(m10)
	
	return res

static func msub(m1 : DenseMatrix, m2 : DenseMatrix) -> DenseMatrix:
	return m1.subtract_dense(m2)

static func mmul(m1 : DenseMatrix, m2 : DenseMatrix, 
	m3 : DenseMatrix = null, m4 : DenseMatrix = null, m5 : DenseMatrix = null, m6 : DenseMatrix = null, 
	m7 : DenseMatrix = null, m8 : DenseMatrix = null, m9 : DenseMatrix = null, m10 : DenseMatrix = null) -> DenseMatrix:
	
	var res = m1.multiply_dense(m2)
	
	if m3: res = res.multiply_dense(m3)
	if m4: res = res.multiply_dense(m4)
	if m5: res = res.multiply_dense(m5)
	if m6: res = res.multiply_dense(m6)
	if m7: res = res.multiply_dense(m7)
	if m8: res = res.multiply_dense(m8)
	if m9: res = res.multiply_dense(m9)
	if m10: res = res.multiply_dense(m10)
	
	return res
	
static func mT(m : DenseMatrix) -> DenseMatrix:
	return m.transposed()
	
static func minv(m : DenseMatrix) -> DenseMatrix:
	return m.inverse()

static func mcrossm(v : DenseMatrix, u : DenseMatrix) -> DenseMatrix:
	assert(v.get_dimensions() == Vector2i(6,1))
	
	var v_cross = spatial_cross_motion_matrix(v)
	return v_cross.multiply_dense(u)
	
static func mcrossf(v : DenseMatrix, u : DenseMatrix) -> DenseMatrix:
	assert(v.get_dimensions() == Vector2i(6,1))
	
	var v_cross = spatial_cross_force_matrix(v)
	return v_cross.multiply_dense(u)

static func string_from_matrix(m : DenseMatrix) -> String:
	var res = ''
	var m_ = m.to_packed_array()
	var dim = m.get_dimensions()
	
	for r in range(dim[0]):
		for c in range(dim[1]):
			res += '%.5f,\t' % (m_[r * dim[0] + c])
		res += '\n'
	
	return res


static func calculate_capsule_capsule_contact(xform1, r1, h1, xform2, r2, h2):
	var cyl_h1 = h1 - 2 * r1
	var cyl_h2 = h2 - 2 * r2
	var p1: Vector3 = xform1.origin - xform1.basis.y * cyl_h1 * 0.5
	var q1: Vector3 = xform1.origin + xform1.basis.y * cyl_h1 * 0.5
	var p2: Vector3 = xform2.origin - xform2.basis.y * cyl_h2 * 0.5
	var q2: Vector3 = xform2.origin + xform2.basis.y * cyl_h2 * 0.5
	
	var c1c2 = closest_point_segment_segment(p1,q1,p2,q2)
	var c1: Vector3 = c1c2[0]
	var c2: Vector3 = c1c2[1]
	
	var diff = c2 - c1
	
	var contact_point = (c1 + c2) * 0.5
	var depth = r1 + r2 - diff.length()
	var normal = Vector3(1,0,0)
	if diff.length_squared() > EPS:
		normal = diff.normalized()
	
	var contact = {}
	contact.point = contact_point
	contact.depth = depth
	contact.normal = normal
	return contact

# copied from Real Time Collision Detection p. 149-150
static func closest_point_segment_segment(p1, q1, p2, q2):
	var d1 = q1 - p1
	var d2 = q2 - p2
	var r = p1 - p2
	var a = d1.dot(d1)
	var e = d2.dot(d2)
	var f = d2.dot(r)
	var c = d1.dot(r)
	
	var b = d1.dot(d2)
	var denom = a*e - b*b
	
	
	var s: float = 0
	if abs(denom ) > EPS:
		s = clamp((b*f - c*e)/denom, 0, 1)
	
	var t = (b*s + f)/e
	
	if t < 0:
		t = 0
		s = clamp(-c/a, 0, 1)
	elif t > 1:
		t = 1
		s = clamp((b - c) / a, 0, 1)
	
	var c1 = p1 + d1 * s
	var c2 = p2 + d2 * t
	
	return [c1,c2]

# creates a basis with normal as the y axis
static func create_basis_from_normal(normal: Vector3):
	assert(normal.is_normalized())
	var y = normal
	
	var x = Vector3(1,0,0)
	if abs(x.dot(y)) > 0.9:
		x = Vector3(0,1,0)
	
	var z = x.cross(y).normalized()
	x = y.cross(z)
	
	return Basis(x,y,z)
