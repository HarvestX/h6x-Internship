// Copyright 2022 HarvestX Inc.
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

#include "game_master.hpp"

GameMaster::GameMaster()
: Node("game_master")
{
  score_ = 0;
  past_time_ = 0;
  charge_score_ = 0;

  game_status_ = READY;

  tmp_courseout_count_ = 0;
  courseout_count_ = 0;

  gameover_was_sent_ = false;
  tmp_charge_flag_ = false;

  this->declare_parameter("initial_score", GAMEOVER_SCORE);
  gameover_socre_ = this->get_parameter("initial_score").as_int();

  sub_status_ =
    this->create_subscription<std_msgs::msg::Int32>(
    "/judge_status", 10,
    std::bind(&GameMaster::status_callback, this, std::placeholders::_1));
  sub_courseout_count_ = this->create_subscription<std_msgs::msg::Int32>(
    "/courseout_count", 10, std::bind(
      &GameMaster::courseout_count_callback, this,
      std::placeholders::_1));

  pub_log_ = this->create_publisher<rcl_interfaces::msg::Log>("/log", 10);

  pub_score_ = this->create_publisher<std_msgs::msg::Int32>("/score", 10);
  pub_time_ = this->create_publisher<std_msgs::msg::Int32>("/time", 10);
  pub_speed_limit_ = this->create_publisher<std_msgs::msg::Float32>("/speed_limit", 10);
  timer_ =
    this->create_wall_timer(
    std::chrono::seconds(1), std::bind(
      &GameMaster::timer_count_update,
      this));
}

void GameMaster::status_callback(const std_msgs::msg::Int32::SharedPtr ptr)
{
  game_status_ = ptr->data;
}
void GameMaster::courseout_count_callback(const std_msgs::msg::Int32::SharedPtr ptr)
{
  courseout_count_ = ptr->data;
}

void GameMaster::timer_count_update()
{
  if (game_status_ == GOAL || score_ >= gameover_socre_) {
  } else if (game_status_ == START) {
    // count timer
    past_time_++;
  } else if (game_status_ == READY) {
    // not started_
    past_time_ = 0;
  } else {
    // warn
    RCLCPP_WARN(this->get_logger(), "unknown game status");
    past_time_ = 0;
  }

  std_msgs::msg::Int32 msg;
  msg.data = (int)past_time_;
  pub_time_->publish(msg);

  score_update_and_publish();
  publish_viecle_status();

  // show log
  game_master_status();
  game_master_score();
  game_master_time();
}

void GameMaster::score_update_and_publish()
{
  if (game_status_ == GOAL || score_ >= gameover_socre_) {
    return;
  }
  std_msgs::msg::Int32 msg;
  score_ = past_time_ + courseout_count_ - charge_score_;

  msg.data = (int)score_;
  pub_score_->publish(msg);
}

void GameMaster::publish_viecle_status()
{
  bool stop = false;
  std_msgs::msg::Float32 msg;

  if (game_status_ == GOAL) {
    stop = true;
  }

  if (score_ > gameover_socre_) {
    stop = true;
    // send error log
    if (!gameover_was_sent_) {
      gameover_was_sent_ = true;
    }
  }

  // publish speed limit
  if (stop) {
    msg.data = 0.0;
  } else {
    msg.data = SPEED_LIMIT_NORMAL;
  }

  pub_speed_limit_->publish(msg);
}


// logging --------------------------------------------------
void GameMaster::game_master_status()
{
  rcl_interfaces::msg::Log _log;
  std::string _data;

  _data = "===========================";
  _log.level = INFO;
  _log.msg = _data;
  pub_log_->publish(_log);

  _data.clear();
  _data = "STATUS: ";
  _log.level = INFO;

  // status
  if (score_ >= gameover_socre_) {
    _log.level = ERROR;
    _data += "GAMEOVER";
  } else {
    switch (game_status_) {
      case READY:
        _data += "READY";
        break;
      case START:
        _data += "START";
        break;
      case GOAL:
        _data += "GOAL";
        break;
      default:
        _data += "UNKNOWN";
        break;
    }
  }

  // if over line --------------------------------------------------
  if (courseout_count_ > tmp_courseout_count_) {
    _log.level = WARN;
    _data += " (COUSEOUT)";
    tmp_courseout_count_ = courseout_count_;
  }

  _log.msg = _data;
  pub_log_->publish(_log);
}

void GameMaster::game_master_time()
{
  rcl_interfaces::msg::Log _log;
  std::string _data;

  // time ---------------------------------------------------------------
  _log.level = INFO;
  _data.clear();
  _data = " TIME: " + std::to_string(past_time_);

  _log.msg = _data;
  pub_log_->publish(_log);
}

void GameMaster::game_master_score()
{
  rcl_interfaces::msg::Log _log;
  std::string _data;

  // score ---------------------------------------------------------------
  _log.level = INFO;
  if (score_ >= gameover_socre_) {
    _log.level = ERROR;
  }

  int64_t total_score = gameover_socre_ - score_;
  if (total_score < 0) {
    total_score = 0;
  }

  _data = " SCORE: " + std::to_string(total_score);
  if (game_status_ == GOAL) {
    int64_t rank = (int64_t)(total_score * 100 / gameover_socre_);
    std::cout << "rank: " << rank << std::endl;
    if (rank >= 80) {
      _data += " RANK: S";
    } else if (rank >= 70) {
      _data += " RANK: A+";
    } else if (rank >= 60) {
      _data += " RANK: A";
    } else if (rank >= 50) {
      _data += " RANK: B+";
    } else if (rank >= 40) {
      _data += " RANK: B";
    } else if (rank >= 30) {
      _data += " RANK: C+";
    } else if (rank >= 20) {
      _data += " RANK: C";
    } else if (rank >= 10) {
      _data += " RANK: D+";
    } else if (rank > 0) {
      _data += " RANK: D";
    } else {
      _data += " RANK: F";
    }
  }

  _log.msg = _data;
  pub_log_->publish(_log);
}

// --------------------------------------------------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GameMaster>());
  rclcpp::shutdown();
  return 0;
}
