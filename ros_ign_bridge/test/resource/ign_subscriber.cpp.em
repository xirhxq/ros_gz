// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <gtest/gtest.h>
#include <ignition/transport.hh>

#include <chrono>
#include <memory>
#include <string>

#include "utils/test_utils.hpp"
#include "utils/ign_test_msg.hpp"

//////////////////////////////////////////////////
/// \brief A class for testing Ignition Transport topic subscription.
template<typename IGN_T>
class MyTestClass
{
/// \brief Class constructor.
/// \param[in] _topic Topic to subscribe.
public: explicit MyTestClass(const std::string & _topic)
{
  EXPECT_TRUE(this->node.Subscribe(_topic, &MyTestClass::Cb, this));
}

/// \brief Member function called each time a topic update is received.
/// \param[in] _msg Ignition message to be validated
public: void Cb(const IGN_T & _msg)
{
  ros_ign_bridge::testing::compareTestMsg(std::make_shared<IGN_T>(_msg));
  this->callbackExecuted = true;
}

/// \brief Member variables that flag when the actions are executed.
public: bool callbackExecuted = false;

/// \brief Transport node;
private: ignition::transport::Node node;
};

@[for m in mappings]@
/////////////////////////////////////////////////
TEST(IgnSubscriberTest, @(m.unique()))
{
  MyTestClass<@(m.ign_type())> client("@(m.unique())");

  using namespace std::chrono_literals;
  ros_ign_bridge::testing::waitUntilBoolVar(
    client.callbackExecuted, 100ms, 200);

  EXPECT_TRUE(client.callbackExecuted);
}

@[end for]@
/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
