/*
 * imu_ros_cmd.cpp
 *
 *  Created on: 2016/12/02
 *      Author: anzai
 */

#include "imu_ros_cmd.h"
#include <vector>
#include <string>
#include "flashmemory/flashmemory.h"

namespace IMU_ROS_CMD
{
  namespace
  {
    ros::NodeHandle* nh_;
    std::vector<IMU*> imu_;
    spinal::ImuCalib::Response imu_calib_res_;
    bool first_call_ = true;
    constexpr uint8_t CALIB_DATA_SIZE = 12; // gyro_bias (3) + acc_bias (3) + mag_bias (3) + mag_scale (3)
  }

  void getImuCalibData()
  {
    for(unsigned int i = 0; i < imu_.size(); i++)
      {
        Vector3f gyro_bias = imu_.at(i)->getGyroBias();
        Vector3f acc_bias = imu_.at(i)->getAccBias();
        Vector3f mag_bias = imu_.at(i)->getMagBias();
        Vector3f mag_scale = imu_.at(i)->getMagScale();
        imu_calib_res_.data[i * CALIB_DATA_SIZE] = gyro_bias.x;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 1] = gyro_bias.y;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 2] = gyro_bias.z;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 3] = acc_bias.x;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 4] = acc_bias.y;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 5] = acc_bias.z;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 6] = mag_bias.x;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 7] = mag_bias.y;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 8] = mag_bias.z;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 9] = mag_scale.x;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 10] = mag_scale.y;
        imu_calib_res_.data[i * CALIB_DATA_SIZE + 11] = mag_scale.z;
      }
  }

  void imuCalibCallback(const spinal::ImuCalib::Request& req, spinal::ImuCalib::Response& res)
  {
    if(first_call_)
      {
        /* init response for imu calib */
        imu_calib_res_.data_length = imu_.size() * CALIB_DATA_SIZE;
        imu_calib_res_.data = new float[imu_.size() * CALIB_DATA_SIZE];
        first_call_ = false;
      }

    switch (req.command)
      {
      case spinal::ImuCalib::Request::GET_CALIB_DATA:
        {
          imu_calib_res_.success = true;
          getImuCalibData();
          res = imu_calib_res_;
          break;
        }
      case spinal::ImuCalib::Request::RESET_CALIB_DATA:
        {
          for(unsigned int i = 0; i < imu_.size(); i++)
            imu_.at(i)->resetCalib();

          getImuCalibData();
          imu_calib_res_.success = true;
          res = imu_calib_res_;

          break;
        }
      case spinal::ImuCalib::Request::CALIB_GYRO:
        {
          bool calib_flag = req.data[0]; // implicit casting from float to bool

          for(unsigned int i = 0; i < imu_.size(); i++)
            imu_.at(i)->gyroCalib(calib_flag, req.data[1]);

          res.success =true;
          res.data_length = 0;

          break;
        }
      case spinal::ImuCalib::Request::CALIB_ACC:
        {
          bool calib_flag = req.data[0]; // implicit casting from float to bool

          for(unsigned int i = 0; i < imu_.size(); i++)
            imu_.at(i)->accCalib(calib_flag, req.data[1]);

          res.success =true;
          res.data_length = 0;

          break;
        }
      case spinal::ImuCalib::Request::CALIB_MAG:
        {
          bool calib_flag = req.data[0]; // implicit casting from float to bool

          for(unsigned int i = 0; i < imu_.size(); i++)
            imu_.at(i)->magCalib(calib_flag, req.data[1]);

          res.success =true;
          res.data_length = 0;
          break;
        }
      case spinal::ImuCalib::Request::SEND_CALIB_DATA:
        {
          int imu_id = req.data[0]; // slave id (0: imu in spinal)
          int module_id = req.data[1]; // gyro/acc/mag

          res.success = true;
          switch (module_id)
            {
            case spinal::ImuCalib::Request::CALIB_GYRO:
              {
                imu_.at(imu_id)->setGyroBias(Vector3f(req.data[2], req.data[3], req.data[4]));
                break;
              }
            case spinal::ImuCalib::Request::CALIB_ACC:
              {
                imu_.at(imu_id)->setAccBias(Vector3f(req.data[2], req.data[3], req.data[4]));
                break;
              }
            case spinal::ImuCalib::Request::CALIB_MAG:
              {
                imu_.at(imu_id)->setMagBias(Vector3f(req.data[2], req.data[3], req.data[4]));
                imu_.at(imu_id)->setMagScale(Vector3f(req.data[5], req.data[6], req.data[7]));
                break;
              }
            default:
              {
                res.success = false;
                nh_->logwarn("unsupported command");
                break;
              }
            }

          res.data_length = 0;
          break;
        }
      case spinal::ImuCalib::Request::SAVE_CALIB_DATA:
        {
          /* because we have more than one imu, so calling IMU::writeCalibData() is very inefficient */
          FlashMemory::erase();
          FlashMemory::write();

          res.success =true;
          res.data_length = 0;
          break;
        }
      default:
        {
          nh_->logwarn("unsupported command");
          res.success =false;
          res.data_length = 0;
          break;
        }
      }
  }

  namespace
  {
    ros::ServiceServer<spinal::ImuCalib::Request, spinal::ImuCalib::Response> imu_calib_srv_("imu_calib", imuCalibCallback);
  }

  void init(ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertiseService(imu_calib_srv_);
  }

  void addImu(IMU* imu)
  {
    imu_.push_back(imu);
  }
}


