#include "rclcpp/rclcpp.hpp"

#include "padflies_cpp/pad_control.hpp"
#include "padflies_cpp/padflie_tf.hpp"

class PadConnection
{
public:
  PadConnection(
      const std::string & prefix,
      const std::string & pad_frame,
      std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
  : m_pad_frame(pad_frame)
  , m_pad_control(std::make_unique<PadControl>())
  , m_padflie_tf(std::make_shared<PadflieTF>())
  {
      m_pad_control->create_connection("megapad");
  }

  void choose_a_pad()
  {


  }

  bool get_pose_world(Eigen::Affine3d & pose)
  {
    return m_padflie_tf->get_world_affine3d(m_pad_frame, pose);  
  }


  
private: 
    std::string m_pad_frame;

    std::unique_ptr<PadControl> m_pad_control;
    std::shared_ptr<PadflieTF> m_padflie_tf;
};