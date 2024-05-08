#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class MoveInCirclePlugin : public ModelPlugin
    {
    public:
        // Constructor
        MoveInCirclePlugin() : radius(5.0), angularVelocity(0.1) {}

        // Load the plugin
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            this->model = _parent;
            // Connection to the world update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&MoveInCirclePlugin::OnUpdate, this));
        }

        // Called on each simulation step
        void OnUpdate()
        {
            double time = this->model->GetWorld()->SimTime().Double();
            double x = radius * cos(this->angularVelocity * time);
            double y = radius * sin(this->angularVelocity * time);

            // Update the model pose
            ignition::math::Pose3d newPose(x, y, 0, 0, 0, 0);
            this->model->SetWorldPose(newPose);
        }

    private:
        physics::ModelPtr model;               // Pointer to the model
        event::ConnectionPtr updateConnection; // Pointer to the connection
        double radius;                         // Radius of the circle
        double angularVelocity;                // Angular velocity in radians per second
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MoveInCirclePlugin)
}
