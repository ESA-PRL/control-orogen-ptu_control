/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace ptu_control;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    /***************************/
    /** Output port variables **/
    /***************************/
    mast2ptuRbs.invalidate();

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /************************/
    /** Read configuration **/
    /************************/
    ptuReadNames = _ptu_reading_names.value();
    ptuCommandNames = _ptu_commanding_names.value();

    /** PTU joints **/
    ptu_joints_commands_out.resize(ptuCommandNames.size());
    ptu_joints_commands_out.names = ptuCommandNames;

    /** Set the initial mast to ptu frame transform (transformation for the transformer) **/
    mast2ptuRbs.invalidate();
    mast2ptuRbs.sourceFrame = _ptu_source_frame.get();
    mast2ptuRbs.targetFrame = _ptu_target_frame.get();


    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    /** Set New Joints commands. RigidBodyState has priority against the joints commands in input ports **/
    if (_ptu_rbs_commands.read(ptu_rbs_commands) == RTT::NewData)
    {
        /** In case it has position then command the PTU in position **/
        if (base::isnotnan(ptu_rbs_commands.orientation.toRotationMatrix()))
        {
            double pan_position = ptu_rbs_commands.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
            double tilt_position = ptu_rbs_commands.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch

            ptu_joints_commands_out[ptuCommandNames[0]].position = pan_position;
            ptu_joints_commands_out[ptuCommandNames[1]].position = tilt_position;

            /** Set the velocities to NaN **/
            ptu_joints_commands_out[ptuCommandNames[0]].speed = base::NaN<float>();
            ptu_joints_commands_out[ptuCommandNames[1]].speed = base::NaN<float>();

        }
        else if (base::isnotnan(ptu_rbs_commands.angular_velocity))
        {
            /** Command in velocity in other cases **/
            double pan_velocity = ptu_rbs_commands.angular_velocity[2];
            double tilt_velocity = ptu_rbs_commands.angular_velocity[1];

            ptu_joints_commands_out[ptuCommandNames[0]].speed = pan_velocity;
            ptu_joints_commands_out[ptuCommandNames[1]].speed = tilt_velocity;

            /** Set the position to NaN **/
            ptu_joints_commands_out[ptuCommandNames[0]].position = base::NaN<float>();
            ptu_joints_commands_out[ptuCommandNames[1]].position = base::NaN<float>();

        }
        _ptu_commands_out.write(ptu_joints_commands_out);
    }
    else if (_ptu_joints_commands.read(ptu_joints_commands_in) == RTT::NewData)
    {
        if (differentJointsInOut (ptu_joints_commands_in, ptu_joints_commands_out))
        {
            ptu_joints_commands_out = ptu_joints_commands_in;
            _ptu_commands_out.write(ptu_joints_commands_out);
        }
    }


    base::samples::Joints ptu_samples;

    /** Compute the new transformation for the transformer **/
    if (_ptu_samples.read(ptu_samples) == RTT::NewData)
    {
        _ptu_samples_out.write(ptu_samples);

        /** The transformation for the transformer **/
        Eigen::Affine3d tf; tf.setIdentity();
        double pan_value = ptu_samples.getElementByName(ptuReadNames[0]).position;
        double tilt_value = ptu_samples.getElementByName(ptuReadNames[1]).position;

        tf = Eigen::Quaternion <double>(Eigen::AngleAxisd(pan_value, Eigen::Vector3d::UnitZ()));
        tf.translation() = Eigen::Vector3d (0.00, 0.00, _mast2tilt_link.value());
        tf = tf * Eigen::Affine3d (Eigen::AngleAxisd(tilt_value, Eigen::Vector3d::UnitY()));

        mast2ptuRbs.time = ptu_samples.time;
        mast2ptuRbs.setTransform(tf);

        /** Write the PTU transformation into the port **/
        _mast_to_ptu_out.write(mast2ptuRbs);
    }

}
void Task::errorHook()
{
    TaskBase::errorHook();

    RTT::log(RTT::Warning)<<"[PTU_CONTROL] Error task. Sending Zero PTU position."<<RTT::endlog();

    /** Safe error by setting the PTU back to zero pan and tilt **/
    ptu_joints_commands_out[ptuCommandNames[0]].position = 0.00;
    ptu_joints_commands_out[ptuCommandNames[1]].position = 0.00;
    _ptu_commands_out.write(ptu_joints_commands_out);

}
void Task::stopHook()
{
    TaskBase::stopHook();

    RTT::log(RTT::Warning)<<"[PTU_CONTROL] Stopping task. Sending Zero PTU position."<<RTT::endlog();

    /** Safe error by setting the PTU back to zero pan and tilt **/
    ptu_joints_commands_out[ptuCommandNames[0]].position = 0.00;
    ptu_joints_commands_out[ptuCommandNames[1]].position = 0.00;
    ptu_joints_commands_out[ptuCommandNames[0]].speed = base::NaN<float>();
    ptu_joints_commands_out[ptuCommandNames[1]].speed = base::NaN<float>();
    _ptu_commands_out.write(ptu_joints_commands_out);


}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    RTT::log(RTT::Fatal)<<"[PTU_CONTROL] Cleaning up task. Sending Zero PTU position."<<RTT::endlog();

    /** Safe error by setting the PTU back to zero pan and tilt **/
    ptu_joints_commands_out[ptuCommandNames[0]].position = 0.00;
    ptu_joints_commands_out[ptuCommandNames[1]].position = 0.00;
    ptu_joints_commands_out[ptuCommandNames[0]].speed = base::NaN<float>();
    ptu_joints_commands_out[ptuCommandNames[1]].speed = base::NaN<float>();

    _ptu_commands_out.write(ptu_joints_commands_out);

}

bool Task::differentJointsInOut (const base::commands::Joints &in_joints, const base::commands::Joints &out_joints)
{
    bool are_different = false;
    for (std::vector<std::string>::iterator it = ptuCommandNames.begin(); it != ptuCommandNames.end(); ++it)
    {
        if ((in_joints[*it].speed != out_joints[*it].speed) || (in_joints[*it].position != out_joints[*it].position))
        {
            are_different = true;
            break;
        }
    }

    return are_different;
}

