#include <gtest/gtest.h>

#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(Plugin, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  std::shared_ptr<hardware_interface::SystemInterface> hw;
  ASSERT_NO_THROW(hw = loader.createSharedInstance("so101_hardware/SO101SystemHardware"));
  EXPECT_NE(hw, nullptr);
}
