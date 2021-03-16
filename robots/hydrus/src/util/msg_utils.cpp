#include <hydrus/util/msg_utils.h>

namespace msg_utils {
  std_msgs::Float32MultiArray EigenMatrix2Float32MultiArray(const Eigen::MatrixXd& mat)
  {
    std_msgs::Float32MultiArray msg;
    msg.data.clear();
    msg.data.resize(mat.cols() * mat.rows());
    msg.layout.data_offset = 0;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "row";
    msg.layout.dim[0].size = mat.rows();
    msg.layout.dim[0].stride = mat.cols() * mat.rows();
    msg.layout.dim[1].label = "column";
    msg.layout.dim[1].size = mat.cols();
    msg.layout.dim[1].stride = mat.cols();

    for (int i = 0; i < mat.rows(); ++i) {
      for (int j = 0; j < mat.cols(); ++j) {
        msg.data[i*msg.layout.dim[1].stride + j] = mat(i,j);
      }
    }
    return msg;
  }

  Eigen::MatrixXd Float32MultiArray2EigenMatrix(const std_msgs::Float32MultiArrayConstPtr& msg)
  {
    Eigen::MatrixXd mat;
    mat.resize(msg->layout.dim[0].size, msg->layout.dim[1].size);
    for(int i = 0; i<msg->layout.dim[0].stride; i++) {
      mat(int(i/msg->layout.dim[1].stride), i%msg->layout.dim[1].stride) = msg->data[i];
    }
    return mat;
  }

  std_msgs::Float32MultiArray Vector2Float32MultiArray(const std::vector<double>& vec)
  {
    std_msgs::Float32MultiArray msg;
    msg.data.clear();
    msg.data.resize(vec.size());
    msg.layout.data_offset = 0;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "vector";
    msg.layout.dim[0].size = vec.size();
    msg.layout.dim[0].stride = vec.size();

    for (int i = 0; i < vec.size(); ++i) {
      msg.data[i] = vec[i];
    }
    return msg;
  }

  std::vector<double> Float32MultiArray2Vector(const std_msgs::Float32MultiArrayConstPtr& msg)
  {
    std::vector<double> vec(msg->layout.dim[0].stride);
    for (int i = 0; i < msg->layout.dim[0].stride; ++i) {
      vec.at(i) = msg->data[i];
    }
    return vec;
  }

}

