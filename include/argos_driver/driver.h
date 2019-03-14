#ifndef _ARGOS_DRIVER_H_
#define _ARGOS_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

namespace argos_driver
{
class ArgosDriver
{
public:
  ArgosDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
                 
  ~ArgosDriver() {}

  bool poll(void);

private:

  /// Callback for diagnostics update for lost communication with vlp
  void diagTimerCallback(const ros::TimerEvent&event);

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
    int cut_angle;                   ///< cutting angle in 1/100°
  } config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;

  /** diagnostics updater */
  ros::Timer diag_timer_;
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

} // namespace argos_driver

#endif // _ARGOS_DRIVER_H_
