#ifndef ROS_SOT_ROBOT_MODEL_HH
# define ROS_SOT_ROBOT_MODEL_HH
# include <list>
# include <string>

//# include <jrl/mal/boost.hh>
# include "jrl/mal/matrixabstractlayer.hh"

namespace ml = maal::boost;

# include <jrl/dynamics/dynamicsfactory.hh>

//# include <sot/core/flags.hh>
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
//# include <dynamic-graph/pool.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>
//# include <sot/core/exception-dynamic.hh>
# include <sot/core/matrix-homogeneous.hh>
# include <sot-dynamic/dynamic.h>
# include "XmlRpcValue.h"

namespace dynamicgraph
{
  class RosSotRobotModel;

  namespace command
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    /// \brief Load from the robot_description parameter of the
    /// parameter server.
    ///
    /// This is the recommended method as it ensures model consistency
    /// between the control and the other nodes.
    class LoadFromParameterServer : public Command
    {
    public:
      explicit LoadFromParameterServer(RosSotRobotModel& entity,
				       const std::string& docstring);
      Value doExecute();
    };

    /// \brief Load model from an URDF file.
    class LoadUrdf : public Command
    {
    public:
      explicit LoadUrdf(RosSotRobotModel& entity, const std::string& docstring);
      Value doExecute();
    };
  } // end of namespace command.

  /// \brief This entity load either the current model available in
  /// the robot_description parameter or a specified file and provides
  /// various data such as body positions, jacobians, etc.
  ///
  /// This relies on jrl_dynamics_urdf to load the model and jrl-dynamics
  /// to realize the computation.
  class RosSotRobotModel : public sot::Dynamic
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
  public:
    typedef ::dynamicgraph::sot::MatrixHomogeneous MatrixHomogeneous;
    RosSotRobotModel(const std::string& n);
    virtual ~RosSotRobotModel();

    void loadUrdf(const std::string& filename);
    void loadFromParameterServer();
    Vector curConf() const;

  protected:

    unsigned getDimension () const
    {
      if (!m_HDR)
	throw std::runtime_error ("no robot loaded");
      return m_HDR->numberDof();
    }

  private:

    /// \brief Name of the parameter where the joints list will be published
    std::string parameterName_;

    /// \brief When did the last computation occur?
    int lastComputation_;

  };
} // end of namespace dynamicgraph.

#endif //! ROS_SOT_ROBOT_MODEL_HH
