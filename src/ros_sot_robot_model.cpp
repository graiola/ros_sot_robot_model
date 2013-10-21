/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Gennaro Raiola
 *   inspired on the RosRobotModel class written by Thomas Moulard, available here http://wiki.ros.org/dynamic_graph_bridge#RosRobotModel.
 */

# include <ros/ros.h>
# include <dynamic-graph/all-commands.h>
# include <dynamic-graph/factory.h>
# include "dynamic_graph_bridge/ros_init.hh"
# include <ros_sot_robot_model/ros_sot_robot_model.hh>

namespace dynamicgraph
{

RosSotRobotModel::RosSotRobotModel(const std::string& name)
    : Dynamic(name,false),
      pn_("jrl_joints_list"),
      ns_("sot_controller")
{
    std::string docstring;

    docstring =
            "\n"
            "  Load the robot model from the parameter server.\n"
            "\n"
            "  This is the recommended method.\n"
            "\n";
    addCommand("loadFromParameterServer", command::makeCommandVoid0(*this,&RosSotRobotModel::loadFromParameterServer,docstring));

    docstring =
            "\n"
            "  Load the robot model from an URDF file.\n"
            "\n";
    addCommand("loadUrdf", command::makeCommandVoid1(*this,&RosSotRobotModel::loadUrdf,docstring));

    docstring =
            "\n"
            "  Set the namespace."
            "\n";
    addCommand("setNamespace", command::makeCommandVoid1(*this,&RosSotRobotModel::setNamespace,docstring));

    docstring =
            "\n"
            "  Set the parameter name."
            "\n";
    addCommand("setParameterName", command::makeCommandVoid1(*this,&RosSotRobotModel::setNamespace,docstring));

    docstring =
            "\n"
            "  Get current configuration of the robot.\n"
            "\n";
    addCommand ("curConf", new command::Getter<RosSotRobotModel,Vector> (*this,&RosSotRobotModel::curConf,docstring));
}

RosSotRobotModel::~RosSotRobotModel()
{}

void RosSotRobotModel::loadUrdf (const std::string& filename)
{

    m_HDR = parse(filename);

    publishJointNames();
}

void RosSotRobotModel::setNamespace (const std::string& ns)
{
    ns_ = ns;
}

void RosSotRobotModel::setParameterName (const std::string& pn)
{
    pn_ = pn;
}

void RosSotRobotModel::loadFromParameterServer()
{
    rosInit (false);
    std::string robotDescription;
    ros::param::param<std::string> (ns_ + "/robot_description", robotDescription, "");

    if (robotDescription.empty ())
        throw std::runtime_error("No model available as ROS parameter. Fail.");

    m_HDR = parseStream(robotDescription);

    publishJointNames();

}

void RosSotRobotModel::publishJointNames(){

    // Load a list of joints ordered by rank
    std::vector<CjrlJoint*> tmp_jv = m_HDR->jointVector();
    std::vector<CjrlJoint*> actJointsVect = actuatedJoints();
    for (int i=0;i<tmp_jv.size();i++)
        if (std::find(actJointsVect.begin(), actJointsVect.end(),tmp_jv[i])!=actJointsVect.end())
            jointNames_[tmp_jv[i]->rankInConfiguration()-6] = tmp_jv[i]->getName();

    ros::NodeHandle nh(ns_);
    nh.setParam(pn_, jointNames_);

}

namespace
{

vectorN convertVector(const ml::Vector& v)
{
    vectorN res (v.size());
    for (unsigned i = 0; i < v.size(); ++i)
        res[i] = v(i);
    return res;
}

ml::Vector convertVector(const vectorN& v)
{
    ml::Vector res;
    res.resize(v.size());
    for (unsigned i = 0; i < v.size(); ++i)
        res(i) = v[i];
    return res;
}

} // end of anonymous namespace.

Vector RosSotRobotModel::curConf() const
{

    // The first 6 dofs are associated to the Freeflyer frame
    // Freeflyer reference frame should be the same as global
    // frame so that operational point positions correspond to
    // position in freeflyer frame.

    XmlRpc::XmlRpcValue ffpose;
    ros::NodeHandle nh(ns_);
    std::string param_name = "ffpose";
    if (nh.hasParam(param_name)){
        nh.getParam(param_name, ffpose);
        ROS_ASSERT(ffpose.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(ffpose.size() == 6);
        for (int32_t i = 0; i < ffpose.size(); ++i)
        {
            ROS_ASSERT(ffpose[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        }
    }
    else
    {
        ffpose.setSize(6);
        for (int32_t i = 0; i < ffpose.size(); ++i)
            ffpose[i] = 0.0;
    }

    if (!m_HDR )
        throw std::runtime_error ("no robot loaded");
    else {
        vectorN currConf = m_HDR->currentConfiguration();
        Vector res;
        res = convertVector(currConf);

        for (int32_t i = 0; i < ffpose.size(); ++i)
            res(i) = static_cast<double>(ffpose[i]);

        return res;
    }
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosSotRobotModel, "RosSotRobotModel");
} // end of namespace dynamicgraph.
