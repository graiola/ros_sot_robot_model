#include <limits>

#include <boost/bind.hpp>

#include <ros/ros.h>

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <jrl/dynamics/urdf/parser.hh>

#include "dynamic_graph_bridge/ros_init.hh"
#include <ros_sot_robot_model/ros_sot_robot_model.hh>

namespace dynamicgraph
{

RosSotRobotModel::RosSotRobotModel(const std::string& name)
    : Dynamic(name,false),
      parameterName_ ("jrl_map"),
      ns_ ("sot_controller"),
      lastComputation_ (std::numeric_limits<int>::min())
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
            "  Set the controller namespace."
            "\n";
    addCommand("setNamespace", command::makeCommandVoid1(*this,&RosSotRobotModel::setNamespace,docstring));

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
    jrl::dynamics::urdf::Parser parser;

    m_HDR = parser.parse(filename);

    ros::NodeHandle nh(ns_);

    nh.setParam(parameterName_, parser.JointsNamesByRank_);
}

void RosSotRobotModel::setNamespace (const std::string& ns)
{
    ns_ = ns;
}

void RosSotRobotModel::loadFromParameterServer()
{
    jrl::dynamics::urdf::Parser parser;

    rosInit (false);
    std::string robotDescription;
    ros::param::param<std::string> (ns_ + "/robot_description", robotDescription, "");

    if (robotDescription.empty ())
        throw std::runtime_error("No model available as ROS parameter. Fail.");

    m_HDR = parser.parseStream (robotDescription);


    // HACK
    /*
    std::vector<CjrlJoint*> vect = m_HDR->jointVector();
    std::cout<<"NAME: "<<vect[6]->getName()<<std::endl;
    m_HDR->rootJoint(*vect[6]);
    */

    ros::NodeHandle nh(ns_);

    nh.setParam(parameterName_, parser.JointsNamesByRank_);

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
