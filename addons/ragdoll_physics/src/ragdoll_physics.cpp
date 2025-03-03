#include "ragdoll_physics.h"
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/Input.hpp>
#include <godot_cpp/core/math.hpp>

#include <algorithm>
#include <cassert>
#include <sstream>

#include "Eigen/LU" // required for ::inverse()


using namespace godot;


const float DELTA = 0.008;
const float EPS = 0.00001;
const float CAPSULE_MASS = 1;
const float CAPSULE_RADIUS = 0.4;
const float CAPSULE_HEIGHT = 2;


void print_matrix(const MatrixMN& m) {
    std::stringstream ss;
    ss << m;
    UtilityFunctions::print("[---------");
    UtilityFunctions::print(ss.str().c_str());
    UtilityFunctions::print("---------]");
}

Basis skewsym_matrix(const Vector3& v) {
    return Basis(
           0,-v.z, v.y,
         v.z,   0,-v.x,
        -v.y, v.x,    0
    );
}


Basis cylinder_inertia(float mass, float height, float radius) {
    float xz = (1.0/12.0) * (height*height + 3 * radius*radius) * mass;
    float y = (1.0/2.0) * mass * radius*radius;

    return Basis(
        xz,0,0,
        0,y,0,
        0,0,xz
    );
}

Matrix66 spatial_inertia(const Basis& inertia_c, float mass) {

    const Basis& I = inertia_c;

    Matrix66 m;
    m <<
        I[0][0],I[0][1],I[0][2],   0,   0,   0,
        I[1][0],I[1][1],I[1][2],   0,   0,   0,
        I[2][0],I[2][1],I[2][2],   0,   0,   0,
              0,      0,      0,mass,   0,   0,
              0,      0,      0,   0,mass,   0,
              0,      0,      0,   0,   0,mass
    ;
    return m;
}


Matrix66 spatial_cross_motion_matrix(const Vector6& v) {
    float  wx = v[0];
    float  wy = v[1];
    float  wz = v[2];
    float vox = v[3];
    float voy = v[4];
    float voz = v[5];

    Matrix66 m;
    m <<
           0, -wz,  wy,    0,   0,   0,
          wz,   0, -wx,    0,   0,   0,
         -wy,  wx,   0,    0,   0,   0,

           0,-voz, voy,    0, -wz,  wy,
         voz,   0,-vox,   wz,   0, -wx,
        -voy, vox,   0,  -wy,  wx,   0;

    return m;
}


Matrix66 spatial_cross_force_matrix(const Vector6& v) {
    Matrix66 m = spatial_cross_motion_matrix(v);
    m.transposeInPlace();
    m *= -1;
    return m;
}



// transforms from A to B
Matrix66 spatial_coordinate_transform_motion(const Transform3D& frameA, const Transform3D& frameB) {
    // E is the basis of A in B
    // r is (B origin - A origin) in A space

    Basis E = frameB.basis.transpose_xform(frameA.basis);
    Vector3 r = frameA.basis.transposed().xform(frameB.origin - frameA.origin);

    Basis n_rx = skewsym_matrix(-r);

    Basis n_E_rx = E * n_rx;

    Matrix66 m;
    m <<
             E[0][0],     E[0][1],     E[0][2],           0,      0,      0,
             E[1][0],     E[1][1],     E[1][2],           0,      0,      0,
             E[2][0],     E[2][1],     E[2][2],           0,      0,      0,

        n_E_rx[0][0],n_E_rx[0][1],n_E_rx[0][2],     E[0][0],E[0][1],E[0][2],
        n_E_rx[1][0],n_E_rx[1][1],n_E_rx[1][2],     E[1][0],E[1][1],E[1][2],
        n_E_rx[2][0],n_E_rx[2][1],n_E_rx[2][2],     E[2][0],E[2][1],E[2][2];

    return m;
}

// transforms from A to B
Matrix66 spatial_coordinate_transform_force(const Transform3D& frameA, const Transform3D& frameB) {
    // E is the basis of A in B
    // r is (B origin - A origin) in A space

    Basis E = frameB.basis.transpose_xform(frameA.basis);
    Vector3 r = frameA.basis.transposed().xform(frameB.origin - frameA.origin);

    Basis n_rx = skewsym_matrix(-r);

    Basis n_E_rx = E * n_rx;

    Matrix66 m;
    m <<
        E[0][0],E[0][1],E[0][2],   n_E_rx[0][0],n_E_rx[0][1],n_E_rx[0][2],
        E[1][0],E[1][1],E[1][2],   n_E_rx[1][0],n_E_rx[1][1],n_E_rx[1][2],
        E[2][0],E[2][1],E[2][2],   n_E_rx[2][0],n_E_rx[2][1],n_E_rx[2][2],

              0,      0,      0,        E[0][0],     E[0][1],     E[0][2],
              0,      0,      0,        E[1][0],     E[1][1],     E[1][2],
              0,      0,      0,        E[2][0],     E[2][1],     E[2][2];

    return m;
}




void RagdollPhysics::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_ground_mesh"), &RagdollPhysics::get_ground_mesh);
    ClassDB::bind_method(D_METHOD("set_ground_mesh", "path"), &RagdollPhysics::set_ground_mesh);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "ground_mesh", PROPERTY_HINT_NODE_TYPE, "Node3D"), "set_ground_mesh", "get_ground_mesh");

}

void RagdollPhysics::init_tree() {
    if (get_child_count() != 1) {
        godot::UtilityFunctions::push_warning("RagdollPhysics must have 1 child");
        return;
    }

    Node3D* body0 = Object::cast_to<Node3D>(get_child(0));
    bodies.push_back(body0);
    joints.push_back(nullptr);
    parent.push_back(-1);

    init_subtree(0);
}

void RagdollPhysics::init_subtree(int parent_i) {

    Node3D* p = bodies[parent_i];
    int joint_count = p->get_child_count();
    for (int i = 0; i < joint_count; ++i) {
        Node3D* joint = Object::cast_to<Node3D>(p->get_child(i));
        Node3D* child_body = Object::cast_to<Node3D>(joint->get_child(0));

        bodies.push_back(child_body);
        joints.push_back(joint);
        parent.push_back(parent_i);

        init_subtree(bodies.size() - 1);
    }

}

void RagdollPhysics::articulated_body_algorithm(float delta, ABA_Results& aba_results){
    const int num_bodies = bodies.size();
    std::vector<Vector6> body_v_body(num_bodies); // body_v_body[i] is the spatial velocity of body i in frame of body i
    body_v_body[0] = Vector6::Zero();

    std::vector<Vector6> body_c(num_bodies);

    std::vector<Matrix66>& body_inertia = aba_results.body_inertia;
    body_inertia.clear();
    body_inertia.resize(num_bodies);
    body_inertia[0] = Matrix66::Zero();

    std::vector<Vector6> p(num_bodies);
    p[0] = Vector6::Zero();

    Transform3D world_T_0 = bodies[0]->get_global_transform();

    for (int i = 1; i < num_bodies; ++i) {
        const int parent_i = parent[i];
        const Transform3D world_T_parent = bodies[parent_i]->get_global_transform();
        const Transform3D world_T_joint = joints[i]->get_global_transform();
        const Transform3D world_T_child = bodies[i]->get_global_transform();

        Matrix66 jXp = spatial_coordinate_transform_motion(world_T_parent, world_T_joint);
        Matrix66 cXj = spatial_coordinate_transform_motion(world_T_joint, world_T_child);

        Vector6 j_v_cj = joint_S[i] * q_dot[i];
        Vector6 c_v_cj = cXj * j_v_cj;// v_Ji in the book

        const Vector6 p_v_p = body_v_body[parent_i];
        const Vector6 p_v_j = p_v_p; // joint is rigidly attached to parent => same spatial velocity
        Vector6 j_v_j = jXp * p_v_j;

        Vector6 c_v_j = cXj * j_v_j;
        Vector6 c_v_c = c_v_j + c_v_cj;
        body_v_body[i] = c_v_c;

        Matrix66 j_v_j_crossm = spatial_cross_motion_matrix(j_v_j);
        Vector6 j_c = j_v_j_crossm * j_v_cj;
        Vector6 c_c = cXj * j_c;
        body_c[i] = c_c;

        body_inertia[i] = cm_inertia;

        Matrix66 cX0_f = spatial_coordinate_transform_force(world_T_0, world_T_child);

        Matrix66 c_v_c_crossf = spatial_cross_force_matrix(c_v_c);
        Vector6 pi = (c_v_c_crossf * body_inertia[i] * c_v_c) - (cX0_f * b0_f[i]);



        p[i] = pi;
    }

    std::vector<MatrixMN>& D_inv = aba_results.D_inv;
    D_inv.clear();
    D_inv.resize(num_bodies);

    std::vector<Matrix6N>& U = aba_results.U;
    U.clear();
    U.resize(num_bodies);

    std::vector<Matrix6N>& body_S = aba_results.body_S;
    body_S.clear();
    body_S.resize(num_bodies);

    std::vector<VectorN> u(num_bodies);

    for (int i = num_bodies-1; i > 0; --i) {// excluding body 0 on purpose
        const int parent_i = parent[i];
        Transform3D world_T_parent = bodies[parent_i]->get_global_transform();
        Transform3D world_T_joint = joints[i]->get_global_transform();
        Transform3D world_T_child = bodies[i]->get_global_transform();

        Matrix66 cXj = spatial_coordinate_transform_motion(world_T_joint, world_T_child);
        Matrix66 cXp = spatial_coordinate_transform_motion(world_T_parent, world_T_child);

        Matrix6N c_S = cXj * joint_S[i];
        body_S[i] = c_S;

        Matrix6N Ui = body_inertia[i] * c_S;
        U[i] = Ui;
        MatrixMN Di = c_S.transpose() * Ui;
        D_inv[i] = Di.inverse();


        VectorN ti = VectorN::Zero(joint_S[i].cols()); // no torques on joint "motors"
        VectorN ui = ti - c_S.transpose() * p[i];
        u[i] = ui;


        if (parent_i != 0) {
            Matrix66 c_I_A_c = body_inertia[i];
            Matrix66 c_I_a_c = c_I_A_c - Ui * D_inv[i] * Ui.transpose();

            Vector6 c_c = body_c[i];
            Vector6 c_p_A_c = p[i];
            Vector6 c_p_a_c = c_p_A_c + c_I_a_c * c_c + Ui * (D_inv[i] * ui);

            Matrix66 pXc_f = spatial_coordinate_transform_force(world_T_child, world_T_parent);
            Matrix66 p_I_A_p = body_inertia[parent_i];
            p_I_A_p = p_I_A_p + pXc_f * c_I_a_c * cXp;
            body_inertia[parent_i] = p_I_A_p;

            Vector6 p_p_A_p = p[parent_i]; // pen pineapple Apple pen
            p_p_A_p = p_p_A_p + pXc_f * c_p_a_c;
            p[parent_i] = p_p_A_p;
        }
    }

    if (delta > EPS) {
        std::vector<Vector6> body_a_body(num_bodies, Vector6::Zero());

        // fictitious upward acceleration on body 0
        // see p. 130
        body_a_body[0] << 0,0,0, 0,10,0;

        for (int i = 1; i < num_bodies; ++i) {
            const int parent_i = parent[i];
            Transform3D world_T_parent = bodies[parent_i]->get_global_transform();
            Transform3D world_T_joint = joints[i]->get_global_transform();
            Transform3D world_T_child = bodies[i]->get_global_transform();

            Matrix66 cXp = spatial_coordinate_transform_motion(world_T_parent, world_T_child);

            Vector6 p_a_p = body_a_body[parent_i];
            Vector6 c_a_p = cXp * p_a_p;
            Vector6 c_c = body_c[i];
            Vector6 c_a_tick = c_a_p + c_c; // a' in the book

            Matrix6N c_S = body_S[i];
            const MatrixMN& Di_inv = D_inv[i];
            const Matrix6N& Ui = U[i];
            const VectorN& ui = u[i];


            VectorN q_dot_dot = Di_inv * (ui - Ui.transpose() * c_a_tick);

            Vector6 c_a_c = c_a_tick + c_S * q_dot_dot;
            body_a_body[i] = c_a_c;


            q_dot[i] = q_dot[i] + q_dot_dot * delta;

            // damping
            float damping_per_sec = 0.3;
            float damping_per_frame = pow(damping_per_sec, DELTA);
            q_dot[i] = q_dot[i] * damping_per_frame;

            //var q_dot_len = sqrt(mmul(mT(q_dot[i]), q_dot[i]).to_packed_array()[0])
            //if q_dot_len > 10:
                //q_dot[i] = mscale(q_dot[i], 10/q_dot_len)
        }
    }
}


// q_dot_in: q_dot to use to update q and the resulting transforms
void RagdollPhysics::update_transforms(float delta, const std::vector<VectorN> &q_dot_in) {
    const int num_bodies = bodies.size();

    std::vector<Transform3D> joint_T_child_arr(num_bodies);

    for (int i = 1; i < num_bodies; ++i) {
        Transform3D world_T_joint = joints[i]->get_global_transform();
        Transform3D world_T_child = bodies[i]->get_global_transform();
        Transform3D joint_T_child = world_T_joint.inverse() * world_T_child;
        joint_T_child_arr[i] = joint_T_child;
    }

    for (int i = 1; i < num_bodies; ++i) {
        const int parent_i = parent[i];
        Transform3D default_world_T_parent = default_body_transforms[parent_i];
        Transform3D default_world_T_joint = default_joint_transforms[i];
        Transform3D default_world_T_child = default_body_transforms[i];

        // default transform of joint in parent space
        Transform3D default_parent_T_joint = default_world_T_parent.inverse() * default_world_T_joint;


        // update joint transform
        Transform3D world_T_parent = bodies[parent_i]->get_global_transform();
        Transform3D world_T_joint = world_T_parent * default_parent_T_joint;
        joints[i]->set_global_transform(world_T_joint);


        // update child transform
        Transform3D world_T_child = bodies[i]->get_global_transform();
        {
            Transform3D default_joint_T_child = default_world_T_joint.inverse() * default_world_T_child;
            Basis default_jRc = default_joint_T_child.basis;
            Vector3 default_j_r_cj = default_joint_T_child.origin;

            Transform3D joint_T_child = joint_T_child_arr[i];
            Transform3D& jTc = joint_T_child;
            Basis& jRc = jTc.basis;
            Vector3& j_r_cj = jTc.origin;

            VectorN qi_dot = q_dot_in[i];
            Vector6 joint_Sq_dot = joint_S[i] * qi_dot; // joint space

            Vector3 w(joint_Sq_dot(0,0), joint_Sq_dot(1,0), joint_Sq_dot(2,0));
            Vector3 v(joint_Sq_dot(3,0), joint_Sq_dot(4,0), joint_Sq_dot(5,0));

            // update jRc
            //TODO: update quaternion instead
            float angle = w.length() * delta;
            Vector3 axis = w.normalized();
            if (w.length() > 0.01 && axis.is_normalized()) {
                Basis delta_rot(axis, angle);
                jRc = delta_rot * jRc;
            }
            jRc.orthonormalize();

            bool constrain_position = joints[i]->get("constrain_position");
            if (constrain_position)
                j_r_cj = jRc.xform(default_jRc.transposed().xform(default_j_r_cj));
            else
                j_r_cj += (w.cross(j_r_cj) + v) * delta;


            world_T_child = world_T_joint * joint_T_child;
            bodies[i]->set_global_transform(world_T_child);
        }
    }
}

Variant RagdollPhysics::get_debug_draw() {
    Variant debug_draw = call("get_debug_draw");
    return debug_draw;
}

void closest_point_segment_segment(
    const Vector3 &p1, const Vector3 &q1,
    const Vector3 &p2, const Vector3 &q2,
    Vector3& c1, Vector3& c2) {

    Vector3 d1 = q1 - p1;
    Vector3 d2 = q2 - p2;

    Vector3 r = p1 - p2;

    float a = d1.dot(d1);
    float e = d2.dot(d2);
    float f = d2.dot(r);
    float c = d1.dot(r);

    float b = d1.dot(d2);
    float denom = a * e - b * b;

    float s = 0.0f;

    if (abs(denom) > EPS) {
        s = CLAMP((b * f - c * e) / denom, 0.0f, 1.0f);
    }

    float t = (b * s + f) / e;

    if (t < 0.0f) {
        t = 0.0f;
        s = CLAMP(-c / a, 0.0f, 1.0f);
    } else if (t > 1.0f) {
        t = 1.0f;
        s = CLAMP((b - c) / a, 0.0f, 1.0f);
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}


Basis create_basis_from_normal(const Vector3 &normal) {
    assert(normal.is_normalized());

    Vector3 y = normal;
    Vector3 x(1, 0, 0);
    if (std::abs(x.dot(y)) > 0.9f) {
        x = Vector3(0, 1, 0);
    }

    Vector3 z = x.cross(y).normalized();
    x = y.cross(z);

    return Basis(x, y, z);
}

RagdollPhysics::Contact calculate_capsule_capsule_contact(
    const Transform3D &xform1, float r1, float h1,
    const Transform3D &xform2, float r2, float h2) {

    float cyl_h1 = h1 - 2 * r1;
    float cyl_h2 = h2 - 2 * r2;
    Vector3 p1 = xform1.origin - xform1.basis.get_column(1) * cyl_h1 * 0.5f;
    Vector3 q1 = xform1.origin + xform1.basis.get_column(1) * cyl_h1 * 0.5f;
    Vector3 p2 = xform2.origin - xform2.basis.get_column(1) * cyl_h2 * 0.5f;
    Vector3 q2 = xform2.origin + xform2.basis.get_column(1) * cyl_h2 * 0.5f;

    Vector3 c1, c2;
    closest_point_segment_segment(p1, q1, p2, q2, c1, c2);

    Vector3 diff = c2 - c1;

    Vector3 contact_point = (c1 + c2) * 0.5f;
    float depth = r1 + r2 - diff.length();
    Vector3 normal = Vector3(1, 0, 0);

    if (diff.length_squared() > EPS)
        normal = diff.normalized();

    RagdollPhysics::Contact contact;
    contact.point = contact_point;
    contact.depth = depth;
    contact.normal = normal;
    return contact;
}


// a contact will be added to the list if it's predicted to interpenetrate after delta time
// check for depth > 0 to see if it's an interpenetration
void RagdollPhysics::generate_contacts(float delta, std::vector<Contact>& contacts) {
    contacts.clear();

    // collision between bodies
    for (int body_i = 1; body_i < bodies.size() - 1; ++body_i) {
        for (int body_j = body_i + 1; body_j < bodies.size(); ++body_j) {
            bool dont_collide = (
                parent[body_i] == body_j ||
                parent[body_j] == body_i ||
                default_joint_transforms[body_i].origin.distance_to(default_joint_transforms[body_j].origin) < 0.01f
               );

            if (dont_collide)
                continue;

            Transform3D world_T_i = bodies[body_i]->get_global_transform();
            Transform3D world_T_j = bodies[body_j]->get_global_transform();

            Contact contact = calculate_capsule_capsule_contact(world_T_i, CAPSULE_RADIUS, CAPSULE_HEIGHT, world_T_j, CAPSULE_RADIUS, CAPSULE_HEIGHT);
            contact.body1 = body_i;
            contact.body2 = body_j;

            Vector6 world_v_i = world_v[body_i];
            Vector6 world_v_j = world_v[body_j];
            Vector6 world_v_ij = world_v_i - world_v_j;

            Transform3D world_T_point(create_basis_from_normal(contact.normal), contact.point);
            Matrix66 point_X_world = spatial_coordinate_transform_motion(Transform3D(), world_T_point);

            Vector6 point_v_ij = point_X_world * world_v_ij;
            float point_v_ij_y = point_v_ij(4,0);

            if (contact.depth + point_v_ij_y * delta > 0.0f) {
                //debug_draw.call("draw_sphere", contact.point, 0.01, Color::named("PURPLE"));
                contacts.push_back(contact);
            }
        }
    }

    // collision with ground
    for (int body_i = 1; body_i < bodies.size(); ++body_i) {
        Vector3 center = bodies[body_i]->get_global_transform().origin;

        Vector3 y_axis = bodies[body_i]->get_global_basis().get_column(1);

        Vector3 top = center + y_axis * (CAPSULE_HEIGHT * 0.5f - CAPSULE_RADIUS);
        Vector3 bottom = center - y_axis * (CAPSULE_HEIGHT * 0.5f - CAPSULE_RADIUS);
        //DebugDraw3D.draw_line(bottom, top, Color.RED)
        top.y -= CAPSULE_RADIUS;
        bottom.y -= CAPSULE_RADIUS;

        for (const Vector3 &point : { bottom, top }) {
            float point_v_y = 0.0f;
            {
                Vector6 world_v_c = world_v[body_i];

                Transform3D world_T_point(Basis(), point);

                Matrix66 point_X_world = spatial_coordinate_transform_motion(Transform3D(), world_T_point);

                Vector6 point_v_c = point_X_world * world_v_c;
                point_v_y = point_v_c(4,0);
            }

            // DebugDraw3D.draw_line(contact_point, contact_point + v, Color.BLUE if v.y < 0 else Color.PURPLE, 1)
            // DebugDraw3D.draw_line(Vector3.ZERO, vo, Color.MIDNIGHT_BLUE)
            // DebugDraw3D.draw_line(Vector3.ZERO, w, Color.BLUE_VIOLET)
            if (ground_mesh == nullptr) {
                UtilityFunctions::printerr("ground_mesh is null");
                return;
            }

            float ground_y = ground_mesh->get_global_transform().origin.y;

            if (point.y + point_v_y * delta < ground_y) {
                // debug_draw.call("draw_sphere", point, 0.01, Color(0.0,1.0,0.5));
                Contact contact;
                contact.point = point;
                contact.body1 = body_i;
                contact.body2 = -1;
                contact.normal = Vector3(0, -1, 0);
                contact.depth = ground_y - point.y;

                contacts.push_back(contact);
            }
        }
    }
}

void RagdollPhysics::interpenetration_penalty_and_friction(const std::vector<Contact>& contacts) {
    Variant debug_draw = get_debug_draw();

    Vector6 b0_force_total = Vector6::Zero();
    for (const Contact &contact : contacts) {
        debug_draw.call("draw_line", contact.point, contact.point + contact.normal * contact.depth, Color::named("RED"));

        int body_a = contact.body1;
        int body_b = contact.body2;
        Vector3 normal = contact.normal;
        float depth = contact.depth;

        if (depth > 0) {
            Transform3D world_T_b0 = bodies[0]->get_global_transform();
            Transform3D world_T_point(create_basis_from_normal(normal), contact.point);

            Matrix66 b0_X_point_f = spatial_coordinate_transform_force(world_T_point, world_T_b0);

            Vector6 point_v_ab;
            {
                Vector6 world_v_ab = world_v[body_a];

                if (body_b != -1) {
                    world_v_ab = world_v_ab - world_v[body_b];
                }

                Matrix66 point_X_world = spatial_coordinate_transform_motion(Transform3D(), world_T_point);
                point_v_ab = point_X_world * world_v_ab;
            }

            float penalty_magnitude = depth * 50.0f;// + point_v_ab[4] * 5.0f;
            Vector6 point_f;
            point_f << 0, 0, 0, 0, -penalty_magnitude, 0;

            Vector6 b0_force_penalty = b0_X_point_f * point_f;

            Vector6 b0_force_friction;
            {
                float friction_coeff = -depth * 20.0f;
                Vector3 point_v_tangential(point_v_ab[3], 0, point_v_ab[5]);
                Vector6 point_force_friction;
                point_force_friction << 0, 0, 0, friction_coeff * point_v_tangential.x, 0, friction_coeff * point_v_tangential.z;
                b0_force_friction = b0_X_point_f * point_force_friction;
            }
            Vector6 b0_force = b0_force_penalty + b0_force_friction;
            b0_f[body_a] = b0_force;
            b0_force_total += b0_force;

            if (body_b != -1) {
                b0_f[body_b] = -b0_force;
                b0_force_total -= b0_force;
            }
        }
    }


    if (b0_force_total.norm() > EPS) {
        for (int i = 1; i < bodies.size(); ++i) {
            Vector6 b0_force_total_weighted = b0_force_total * (b0_f[i].norm() / b0_force_total.norm());
            //b0_f[i] -= b0_force_total_weighted * 0.01;
        }
    }
    //b0_f[1] = -b0_force_total;
}

void RagdollPhysics::collision_response(const Contact &contact, const ABA_Results &aba_results) {
    int body_a = contact.body1;
    int body_b = contact.body2;
    Vector3 normal = contact.normal;

    Vector6 world_v_ab = world_v[body_a];
    if (body_b != -1) {
        world_v_ab = world_v_ab - world_v[body_b];
    }

    Transform3D world_T_point(create_basis_from_normal(normal), contact.point);
    Matrix66 point_X_world = spatial_coordinate_transform_motion(Transform3D(), world_T_point);
    Vector6 point_v_ab = point_X_world * world_v_ab;

    float point_v_ab_y = point_v_ab[4];

    if (point_v_ab_y > 0) {
        Transform3D world_T_b0 = bodies[0]->get_global_transform();

        std::vector<float> p_I_A_c_inv_44_arr;

        for (int body_i : {body_a, body_b}) {
            float p_I_A_c_inv_44 = 0.0f;

            if (body_i != -1) {
                Transform3D world_T_c = bodies[body_i]->get_global_transform();
                Matrix66 c_I_A_c = aba_results.body_inertia[body_i];
                Matrix66 c_X_p = spatial_coordinate_transform_motion(world_T_point, world_T_c);
                Matrix66 p_X_c_force = spatial_coordinate_transform_force(world_T_c, world_T_point);
                Matrix66 p_I_A_c = p_X_c_force * c_I_A_c * c_X_p;

                std::vector<int> cofactor_indices_44 = {0,1,2,3,5};
                Eigen::Matrix<float,5,5> cofactor_matrix_44 = p_I_A_c(cofactor_indices_44, cofactor_indices_44);
                float cofactor44 = cofactor_matrix_44.determinant();

                // equivalent to p_I_A_c_inv_44 = p_I_A_c.inverse()(4,4);
                p_I_A_c_inv_44 = cofactor44/p_I_A_c.determinant();
            }
            p_I_A_c_inv_44_arr.push_back(p_I_A_c_inv_44);
        }

        float p_I_A_a_inv_44 = p_I_A_c_inv_44_arr[0];
        float p_I_A_b_inv_44 = p_I_A_c_inv_44_arr[1];

        float restitution = 0.4f;
        float vel_normal_scale = 1.0f + restitution;
        Vector6 p_impulse;
        p_impulse << 0, 0, 0, 0, -point_v_ab_y / (p_I_A_a_inv_44 + p_I_A_b_inv_44) * vel_normal_scale, 0;
        Matrix66 b0_X_p_force = spatial_coordinate_transform_force(world_T_point, world_T_b0);
        Vector6 b0_impulse = b0_X_p_force * p_impulse;
        b0_b[body_a] = b0_b[body_a] + b0_impulse;

        if (body_b != -1) {
            b0_b[body_b] = b0_b[body_b] - b0_impulse;
        }
    }
}


void RagdollPhysics::apply_impulses(const ABA_Results &aba_results, std::vector<VectorN> &dq_dot, const std::vector<Contact> &_debug_contacts) {
    const std::vector<Matrix6N>& U = aba_results.U;
    const std::vector<MatrixMN>& D_inv = aba_results.D_inv;
    const std::vector<Matrix6N>& body_S = aba_results.body_S;

    Transform3D world_T_b0 = bodies[0]->get_global_transform();

    const int num_bodies = bodies.size();

    std::vector<Vector6> b(num_bodies, Vector6::Zero()); // bias impulses

    for (int i = 1; i < num_bodies; ++i) {
        Transform3D world_T_child = bodies[i]->get_global_transform();

        Matrix66 cX0_f = spatial_coordinate_transform_force(world_T_b0, world_T_child);

        Vector6 bi = -(cX0_f * b0_b[i]);
        b[i] = bi;

        // zero out the impulse after we took it into account
        b0_b[i] = b0_b[i] * 0;
    }

    for (int i = num_bodies - 1; i > 0; --i) {  //excluding body 0 on purpose
        int parent_i = parent[i];

        if (parent_i != 0) {
            Transform3D world_T_parent = bodies[parent_i]->get_global_transform();
            Transform3D world_T_child = bodies[i]->get_global_transform();

            Matrix6N c_S = body_S[i];

            Matrix66 pXc_f = spatial_coordinate_transform_force(world_T_child, world_T_parent);

            Vector6 c_b_A_c = b[i];
            Vector6 c_b_a_c = c_b_A_c - U[i] * (D_inv[i] * (c_S.transpose() * c_b_A_c));
            Vector6 p_b_A_p = b[parent_i];
            p_b_A_p = p_b_A_p + (pXc_f * c_b_a_c);
            b[parent_i] = p_b_A_p;
        }
    }

    std::vector<Vector6> body_dv_impulse_body(num_bodies); // instantaneous change in velocity due to impulse
    body_dv_impulse_body[0] << 0,0,0, 0,0,0;

    for (int i = 1; i < num_bodies; ++i) {
        int parent_i = parent[i];

        Transform3D world_T_parent = bodies[parent_i]->get_global_transform();
        Transform3D world_T_joint = joints[i]->get_global_transform();
        Transform3D world_T_child = bodies[i]->get_global_transform();

        Matrix66 cXp = spatial_coordinate_transform_motion(world_T_parent, world_T_child);

        Matrix6N c_S = body_S[i];
        const MatrixMN& Di_inv = D_inv[i];
        const Matrix6N& Ui = U[i];

        Vector6 p_dv_impulse_p = body_dv_impulse_body[parent_i];
        Vector6 c_dv_impulse_p = cXp * p_dv_impulse_p;

        const Vector6& c_b_A_c = b[i];

        VectorN c_dqdot_impulse_c = -Di_inv * (Ui.transpose() * c_dv_impulse_p + c_S.transpose() * c_b_A_c);

        Vector6 c_dv_impulse_c = c_dv_impulse_p + (c_S * c_dqdot_impulse_c);
        body_dv_impulse_body[i] = c_dv_impulse_c;

        dq_dot[i] = dq_dot[i] + c_dqdot_impulse_c;
    }
}

void RagdollPhysics::apply_impulse2(int body_a, int body_b, const ABA_Results &aba_results, std::vector<VectorN> &dq_dot, const std::vector<Contact> &_debug_contacts) {
    const std::vector<Matrix6N>& U = aba_results.U;
    const std::vector<MatrixMN>& D_inv = aba_results.D_inv;
    const std::vector<Matrix6N>& body_S = aba_results.body_S;

    Transform3D world_T_b0 = bodies[0]->get_global_transform();

    const int num_bodies = bodies.size();

    std::vector<Vector6> b(num_bodies, Vector6::Zero()); // bias impulses

    for (int i : {body_a, body_b}) {
        if (i != -1) {
            Transform3D world_T_child = bodies[i]->get_global_transform();

            Matrix66 cX0_f = spatial_coordinate_transform_force(world_T_b0, world_T_child);

            Vector6 bi = -(cX0_f * b0_b[i]);
            b[i] = bi;

            // zero out the impulse after we took it into account
            b0_b[i] = b0_b[i] * 0;
        }
    }

    for (int i = MAX(body_a, body_b); i > 0; --i) {  //excluding body 0 on purpose
        int parent_i = parent[i];

        if (parent_i != 0) {
            Transform3D world_T_parent = bodies[parent_i]->get_global_transform();
            Transform3D world_T_child = bodies[i]->get_global_transform();

            Matrix6N c_S = body_S[i];

            Matrix66 pXc_f = spatial_coordinate_transform_force(world_T_child, world_T_parent);

            Vector6 c_b_A_c = b[i];
            Vector6 c_b_a_c = c_b_A_c - U[i] * (D_inv[i] * (c_S.transpose() * c_b_A_c));
            Vector6 p_b_A_p = b[parent_i];
            p_b_A_p = p_b_A_p + (pXc_f * c_b_a_c);
            b[parent_i] = p_b_A_p;
        }
    }

    std::vector<Vector6> body_dv_impulse_body(num_bodies); // instantaneous change in velocity due to impulse
    body_dv_impulse_body[0] << 0,0,0, 0,0,0;

    for (int i = 1; i < num_bodies; ++i) {
        int parent_i = parent[i];

        Transform3D world_T_parent = bodies[parent_i]->get_global_transform();
        Transform3D world_T_joint = joints[i]->get_global_transform();
        Transform3D world_T_child = bodies[i]->get_global_transform();

        Matrix66 cXp = spatial_coordinate_transform_motion(world_T_parent, world_T_child);

        Matrix6N c_S = body_S[i];
        const MatrixMN& Di_inv = D_inv[i];
        const Matrix6N& Ui = U[i];

        Vector6 p_dv_impulse_p = body_dv_impulse_body[parent_i];
        Vector6 c_dv_impulse_p = cXp * p_dv_impulse_p;

        const Vector6& c_b_A_c = b[i];

        VectorN c_dqdot_impulse_c = -Di_inv * (Ui.transpose() * c_dv_impulse_p + c_S.transpose() * c_b_A_c);

        Vector6 c_dv_impulse_c = c_dv_impulse_p + (c_S * c_dqdot_impulse_c);
        body_dv_impulse_body[i] = c_dv_impulse_c;

        dq_dot[i] = dq_dot[i] + c_dqdot_impulse_c;
    }
}

void godot::RagdollPhysics::calculate_world_velocities(const std::vector<VectorN> &q_dot_in)
{
    world_v.clear();
    world_v.resize(bodies.size());
    world_v[0] = Vector6::Zero();

    for (int i = 1; i < bodies.size(); ++i) {
        Matrix66 world_X_j = spatial_coordinate_transform_motion(joints[i]->get_global_transform(), Transform3D());

        Vector6 j_v_cp = joint_S[i] * q_dot_in[i];

        Vector6 world_v_cp = world_X_j * j_v_cp;

        world_v[i] = world_v[parent[i]] + world_v_cp;
    }
}

RagdollPhysics::RagdollPhysics() {

}


RagdollPhysics::~RagdollPhysics() {

}


void RagdollPhysics::_ready() {
    init_tree();

    const int num_bodies = bodies.size();

    default_body_transforms.resize(num_bodies);
    default_joint_transforms.resize(num_bodies);
    joint_S.resize(num_bodies);
    q.resize(num_bodies);
    q_dot.resize(num_bodies);
    b0_f.resize(num_bodies);
    b0_b.resize(num_bodies);

    for (int i = 0; i < num_bodies; ++i) {
        default_body_transforms[i] = bodies[i]->get_global_transform();

        if (joints[i]) {
            default_joint_transforms[i] = joints[i]->get_global_transform();

            {
                Array joint_S_i_arr(joints[i]->call("get_joint_S"));
                int num_rows = joint_S_i_arr.size();
                int num_cols = Array(joint_S_i_arr[0]).size();
                assert(num_rows == 6);

                joint_S[i].resize(6, num_cols);

                for (int r = 0; r < num_rows; ++r) {
                    Array row(joint_S_i_arr[r]);
                    for (int c = 0; c < num_cols; ++c) {
                        joint_S[i](r,c) = row[c];
                    }
                }

                // std::stringstream sout;
                // sout << joint_S[i];
                // UtilityFunctions::print("--");
                // UtilityFunctions::print(sout.str().c_str());
                // UtilityFunctions::print("--");

            }

            VectorN zeros = VectorN::Zero(joint_S[i].cols());
            q[i] = zeros;
            q_dot[i] = zeros;
        }

        b0_f[i] = Vector6::Zero();
        b0_b[i] = Vector6::Zero();
    }


    Basis cylinder_inertia3 = cylinder_inertia(CAPSULE_MASS, CAPSULE_HEIGHT, CAPSULE_RADIUS);
    cm_inertia = spatial_inertia(cylinder_inertia3, CAPSULE_MASS);
}


void RagdollPhysics::_process(double delta) {
    //var _config = DebugDraw3D.new_scoped_config().set_no_depth_test(true)

    Input* InputServer = Input::get_singleton();

    bool step_all = InputServer->is_action_just_pressed("ui_accept") || InputServer->is_key_pressed(KEY_P);

    Variant debug_draw = get_debug_draw();
    Variant debug_draw_scoped_config = debug_draw.call("new_scoped_config");
    debug_draw_scoped_config.call("set_no_depth_test", true);


    if (step_all) {

        ABA_Results aba_results;

        articulated_body_algorithm(DELTA, aba_results);

        calculate_world_velocities(q_dot);

        std::vector<Contact> contacts;
        generate_contacts(DELTA, contacts);

        for (const Contact& contact : contacts) {
            debug_draw.call("draw_line", contact.point, contact.point + contact.normal * contact.depth, Color::named("RED"));
        }


        //#contacts.sort_custom(sort_contacts_by_depth)
        //std::shuffle(contacts.begin(), contacts.end(), random_generator);

        interpenetration_penalty_and_friction(contacts);



        for (const Contact& contact : contacts) {
            for (int i = 1; i < bodies.size(); ++i)
                b0_b[i] = b0_b[i] * 0.0f;
            collision_response(contact, aba_results);
            apply_impulse2(contact.body1, contact.body2, aba_results, q_dot, contacts);

            calculate_world_velocities(q_dot);

            //UtilityFunctions::print("apply_impulse2");
            //print_matrix(q_dot[1]);
        }

        update_transforms(DELTA, q_dot);
    }
}

void RagdollPhysics::set_ground_mesh(Node3D* path) {
    ground_mesh = path;
}

Node3D* RagdollPhysics::get_ground_mesh() const {
    return ground_mesh;
}
