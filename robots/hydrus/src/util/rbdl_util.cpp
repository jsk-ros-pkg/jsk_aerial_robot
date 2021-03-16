#include <hydrus/util/rbdl_util.h>

namespace rbdl_util
{

  std::string get_body_name (const Model &model, unsigned int body_id) {
    if (model.mBodies[body_id].mIsVirtual) {
      // if there is not a unique child we do not know what to do...
      if (model.mu[body_id].size() != 1)
        return "";

      return get_body_name (model, model.mu[body_id][0]);
    }

    return model.GetBodyName(body_id);
  }

  std::vector<unsigned int> get_torsion_dof_update_order(RigidBodyDynamics::Model* model, int torsion_num, std::string name_prefix, int start_num) {
    std::vector<unsigned int> torsion_dof_update_order(torsion_num);
    unsigned int q_index = 0;
    for (unsigned int i = 1; i < model->mBodies.size(); i++) {
      std::string body_name = get_body_name(*model, i);
      if (model->mJoints[i].mDoFCount == 1) {
        if (body_name.find(name_prefix) == 0) {
          torsion_dof_update_order.at(std::stoi(body_name.substr(name_prefix.size()))-start_num) = q_index;
        }
        q_index++;
      } else {
        for (unsigned int j = 0; j < model->mJoints[i].mDoFCount; j++) {
          if (body_name.find(name_prefix) == 0) {
            torsion_dof_update_order.at(std::stoi(body_name.substr(name_prefix.size()))-start_num) = q_index;
          }
          q_index++;
        }
      }
    }
    return torsion_dof_update_order;
  }

}
