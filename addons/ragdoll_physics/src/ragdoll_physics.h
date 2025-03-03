#ifndef RAGDOLL_PHYSICS_H
#define RAGDOLL_PHYSICS_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <vector>

#include "Eigen/Core"

#include <random>

namespace godot {

    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixMN;
    typedef Eigen::Matrix<float, 6, Eigen::Dynamic> Matrix6N;
    typedef Eigen::Matrix<float, 6, 6> Matrix66;
    typedef Eigen::Matrix<float, 6, 1> Vector6;
    typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorN;

    class RagdollPhysics : public Node3D {
        GDCLASS(RagdollPhysics, Node3D)
    public:
        struct Contact;

    protected:
        static void _bind_methods();

    private:
        struct ABA_Results {
            std::vector<Matrix66> body_inertia;
            std::vector<Matrix6N> U;
            std::vector<MatrixMN> D_inv;

            std::vector<Matrix6N> body_S;
        };

        std::vector<Vector6> world_v;


        void init_tree();
        void init_subtree(int parent_i);
        void articulated_body_algorithm(float delta, ABA_Results& aba_results);
        void update_transforms(float delta, const std::vector<VectorN>& q_dot_in);
        void generate_contacts(float delta, std::vector<Contact>& contacts);
        void interpenetration_penalty_and_friction(const std::vector<Contact>& contacts);
        void collision_response(const Contact &contact, const ABA_Results &aba_results);
        void apply_impulses(const ABA_Results &aba_results, std::vector<VectorN> &dq_dot, const std::vector<Contact> &_debug_contacts);

        void apply_impulse2(int body_a, int body_b, const ABA_Results &aba_results, std::vector<VectorN> &dq_dot, const std::vector<Contact> &_debug_contacts);

        void calculate_world_velocities(const std::vector<VectorN>& q_dot_in);

        Variant get_debug_draw();

    public:

        RagdollPhysics();
        ~RagdollPhysics();
    
        virtual void _ready() override;
        virtual void _process(double delta) override;

        void set_ground_mesh(Node3D* node);
        Node3D* get_ground_mesh() const;

        
        struct Contact {
            int body1 = -1;
            int body2 = -1;
            Vector3 point;
            Vector3 normal;
            float depth = 0.0f;
        };

        std::vector<Node3D*> bodies;
        std::vector<Node3D*> joints;
        std::vector<int> parent;

        std::vector<Matrix6N> joint_S;
        std::vector<VectorN> q;
        std::vector<VectorN> q_dot;

        std::vector<Transform3D> default_body_transforms;
        std::vector<Transform3D> default_joint_transforms;

        std::vector<Vector6> b0_f;
        std::vector<Vector6> b0_b;

        Matrix66 cm_inertia;

        Node3D* ground_mesh = nullptr;


        std::default_random_engine random_generator;

    };

}

#endif