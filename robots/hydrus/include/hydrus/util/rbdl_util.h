#ifndef RBDL_UTIL_HYDRUS
#define RBDL_UTIL_HYDRUS

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <string>
#include <vector>
#include <algorithm>
#include <utility>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace rbdl_util
{
  std::string get_body_name (const Model &model, unsigned int body_id);

  std::vector<unsigned int> get_torsion_dof_update_order(RigidBodyDynamics::Model* model, int torsion_num, std::string name_prefix="torsion_dummy", int start_num=1);

}

#endif /* ifndef RBDL_UTIL_HYDRUS */
