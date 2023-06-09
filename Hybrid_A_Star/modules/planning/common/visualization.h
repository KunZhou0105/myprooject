/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

namespace apollo {
namespace common {

struct tstTime {
  uint32_t u32_sec;
  uint32_t u32_nsec;
};

struct tstHeader {
  std::string s_frame_id;
  tstTime st_stamp;
};

struct tstPosition {
    float f_x;
    float f_y;
    float f_heading;
};

struct tstOccupancyGrid {
  tstHeader st_header;
  tstPosition st_origin;
  float f_width;
  float f_height;
  float f_length;
  float f_resolution;
  std::vector<uint8_t> vt_data;
};

}  // namespace common
}  // namespace apollo
