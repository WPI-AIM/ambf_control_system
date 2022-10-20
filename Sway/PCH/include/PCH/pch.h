#include <iostream>
#include <stdio.h>
#include <utility>
#include <algorithm>
#include <functional>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <cassert>
#include <exception>
#include <iomanip>
#include <limits>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "eigen3/Eigen/Dense"
#include "catch2/catch.hpp"

#include <rbdl/Body.h>
#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl.h>
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Body.h"
#include "rbdl/Logging.h"
#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Constraints.h"
#include "rbdl/rbdl_errors.h"
#include "rbdl/rbdl_math.h"
#include "rbdl/rbdl_utils.h"

#include "ambf_client/ambf_client.h"