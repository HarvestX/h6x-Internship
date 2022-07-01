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
  this->score_ = 0;
  this->past_time_ = 0;
  this->charge_score_ = 0;

  this->game_status_ = READY;

  this->tmp_deviation_count_ = 0;
  this->deviation_count_ = 0;

  this->game_over_was_sent_ = false;
  tmp_charge_flag_ = false;

  this->declare_parameter("initial_score", GAME_OVER_SCORE);
  this->game_over_score_ = this->get_parameter("initial_score").as_int();

  this->sub_status_ =
    this->create_subscription<std_msgs::msg::Int32>(
    "/judge_status", 10,
    std::bind(&GameMaster::onStatus, this, std::placeholders::_1));
  this->sub_deviation_count_ = this->create_subscription<std_msgs::msg::Int32>(
    "/deviation_count", 10, std::bind(
      &GameMaster::onDeviationCount, this,
      std::placeholders::_1));

  this->pub_log_ = this->create_publisher<rcl_interfaces::msg::Log>("/log", 10);

  this->pub_score_ =
    this->create_publisher<std_msgs::msg::Int32>("/score", 10);
  this->pub_time_ =
    this->create_publisher<std_msgs::msg::Int32>("/time", 10);
  this->pub_speed_limit_ =
    this->create_publisher<std_msgs::msg::Float32>("/speed_limit", 10);
  this->timer_ =
    this->create_wall_timer(
    std::chrono::seconds(1), std::bind(
      &GameMaster::timerCountUpdate,
      this));
}

void GameMaster::onStatus(const std_msgs::msg::Int32::SharedPtr ptr)
{
  this->game_status_ = ptr->data;
}
void GameMaster::onDeviationCount(
  const std_msgs::msg::Int32::SharedPtr ptr)
{
  this->deviation_count_ = ptr->data;
}

void GameMaster::timerCountUpdate()
{
  if (this->game_status_ == GOAL || this->score_ >= this->game_over_score_) {
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
  msg.data = static_cast<int>(past_time_);
  pub_time_->publish(msg);

  scoreUpdateAndPublish();
  publishVehicleStatus();

  // show log
  gameMasterStatus();
  gameMasterScore();
  gameMasterTime();
}

void GameMaster::scoreUpdateAndPublish()
{
  if (this->game_status_ == GOAL || this->score_ >= this->game_over_score_) {
    return;
  }
  std_msgs::msg::Int32 msg;
  score_ = this->past_time_ + this->deviation_count_ - this->charge_score_;

  msg.data = static_cast<int>(this->score_);
  pub_score_->publish(msg);
}

void GameMaster::publishVehicleStatus()
{
  bool stop = false;
  std_msgs::msg::Float32 msg;

  if (this->game_status_ == GOAL) {
    stop = true;
  }

  if (this->score_ > this->game_over_score_) {
    stop = true;
    // send error log
    if (!this->game_over_was_sent_) {
      this->game_over_was_sent_ = true;
    }
  }

  // publish speed limit
  if (stop) {
    msg.data = 0.0;
  } else {
    msg.data = SPEED_LIMIT_NORMAL;
  }

  this->pub_speed_limit_->publish(msg);
}


// logging --------------------------------------------------
void GameMaster::gameMasterStatus()
{
  rcl_interfaces::msg::Log _log;
  std::string _data;

  _data = "===========================";
  _log.level = INFO;
  _log.msg = _data;
  this->pub_log_->publish(_log);

  _data.clear();
  _data = "STATUS: ";
  _log.level = INFO;

  // status
  if (score_ >= this->game_over_score_) {
    _log.level = ERROR;
    _data += "GAME OVER";
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
  if (this->deviation_count_ > this->tmp_deviation_count_) {
    _log.level = WARN;
    _data += " (DEVIATION)";
    this->tmp_deviation_count_ = this->deviation_count_;
  }

  _log.msg = _data;
  this->pub_log_->publish(_log);
}

void GameMaster::gameMasterTime()
{
  rcl_interfaces::msg::Log _log;
  std::string _data;

  // time ---------------------------------------------------------------
  _log.level = INFO;
  _data.clear();
  _data = " TIME: " + std::to_string(this->past_time_);

  _log.msg = _data;
  this->pub_log_->publish(_log);
}

void GameMaster::gameMasterScore()
{
  rcl_interfaces::msg::Log _log;
  std::string _data;

  // score ---------------------------------------------------------------
  _log.level = INFO;
  if (score_ >= this->game_over_score_) {
    _log.level = ERROR;
  }

  int64_t total_score = this->game_over_score_ - score_;
  if (total_score < 0) {
    total_score = 0;
  }

  _data = " SCORE: " + std::to_string(total_score);
  if (game_status_ == GOAL) {
    int64_t rank = static_cast<int64_t>(total_score * 100.0 / this->game_over_score_);
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
