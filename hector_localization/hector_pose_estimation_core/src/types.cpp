//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_pose_estimation/types.h>

namespace hector_pose_estimation {

std::string getSystemStatusString(const SystemStatus& status, const SystemStatus& asterisk_status) {
  std::string result;
  static const char* const str[] = {
    "ALIGNMENT", "DEGRADED", "READY", 0,
    "ROLLPITCH", "YAW", "PSEUDO_ROLLPITCH", "PSEUDO_YAW",
    "RATE_XY", "RATE_Z", "PSEUDO_RATE_XY", "PSEUDO_RATE_Z",
    "VELOCITY_XY", "VELOCITY_Z", "PSEUDO_VELOCITY_XY", "PSEUDO_VELOCITY_Z",
    "POSITION_XY", "POSITION_Z", "PSEUDO_POSITION_XY", "PSEUDO_POSITION_Z",
  };

  if (asterisk_status) {
      for(unsigned int i = 0; i < sizeof(str)/sizeof(*str); ++i) {
      if (asterisk_status & (1 << i)) {
        result += "*" + std::string(str[i]) + " ";
      }
    }

    if (asterisk_status != status) result += "(";
  }

  for(unsigned int i = 0; i < sizeof(str)/sizeof(*str); ++i) {
    if (status & (1 << i)) {
      if (asterisk_status & (1 << i)) continue;
      result += std::string(str[i]) + " ";
    }
  }
  if (result.size() > 0) result.resize(result.size() - 1);

  if (asterisk_status && (asterisk_status != status)) {
    result += ")";
  }

  return result;
}

} // namespace hector_pose_estimation
