#include "Xbee/soldier_mode/soldier_mode.h"

namespace xbee
{
  SoldierMode::SoldierMode(ros::NodeHandle nh, ros::NodeHandle nh_private):
    NormalMode(nh, nh_private, false),
    nh_(nh),
    nhPrivate_(nh_private),
    tf(ros::Duration(10))
  {
    rosInit();
    soldierInit();

    //the SMAN node
    smanNode_ = new SMAN(nh, nh_private, tf);
    wavePropagation_ = smanNode_->getWavePropagation();
    serialDeploymentVector.resize(0);

  //the init order
    ros::Duration d (1.0 / freq_);
    ros::Duration imageShowD (1.0 / imageShowFreq_);

    timer_ = nhPrivate_.createTimer (d, &SoldierMode::spin, this);
    imageShowTimer_ = nhPrivate_.createTimer (imageShowD, &SoldierMode::imageShowSpin, this);

    threadOk = true;
    image_thread = boost::thread(boost::bind(&SoldierMode::feedImages, this));
  }

  SoldierMode::~SoldierMode ()
  {
    printf ("                  Destroying SoldierMode Interface");
    threadOk = false;
    image_thread.join();
    if(imageSendFlag_)
      delete cameraImage_;
    if(fileOutputFlag_)
      fclose(fpRssiPosition_);
    delete smanNode_;
  }

  void SoldierMode::rosInit()
  {
    //**** get parameters
    //******* timer
    printf("freq is %f\n", freq_);
    if (!nhPrivate_.getParam ("imageShowFreq", imageShowFreq_))
      imageShowFreq_ = 1.0; //base:
    printf("imageShowFreq is %f\n", imageShowFreq_);
    if (freq_ <= 0.0 || imageShowFreq_ <= 0.0 )
      ROS_FATAL ("Invalid frequency param");
    initTime = ros::Time::now();

    //******* camera image
    if (!nhPrivate_.getParam ("camNo",camNo_))
      camNo_ = 0;
    printf("camNo_ is %d\n", camNo_);
    if (!nhPrivate_.getParam ("imageProcessMode",imageProcessMode_))
      imageProcessMode_ = RESIZED_COLORSCALE;
    printf("imageProcessMode_ is %d\n", imageProcessMode_);
    if (!nhPrivate_.getParam ("resizeMode",resizeMode_))
      resizeMode_ = cameraImage_->RESIZE_WITH_RATE;
    printf("resizeMode_ is %d\n",resizeMode_);
    if (!nhPrivate_.getParam ("resizeWidthRate",resizeWidthRate_))
      resizeWidthRate_ = 0.2;
    printf("resizeWidthRate_ is %f\n", resizeWidthRate_);
    if (!nhPrivate_.getParam ("resizeHeightRate",resizeHeightRate_))
      resizeHeightRate_ = 0.2;
    printf("resizeHeightRate_ is %f\n", resizeHeightRate_);
    if (!nhPrivate_.getParam ("resizeWidth",resizeWidth_))
      resizeWidth_ = 128;
    printf("resizeWidth_ is %d\n", resizeWidth_);
    if (!nhPrivate_.getParam ("resizeHeight",resizeHeight_))
      resizeHeight_ = 96;
    printf("resizeHeight_ is %d\n", resizeHeight_);
    if (!nhPrivate_.getParam ("compressRate",compressRate_))
      compressRate_ = 1.0;
    printf("compressRate_ is %f\n", compressRate_);
    if (!nhPrivate_.getParam ("saveFileCompressRate",saveFileCompressRate_))
      saveFileCompressRate_ = 1.0;
    printf("saveFileCompressRate_ is %f\n", saveFileCompressRate_);
    if (!nhPrivate_.getParam ("imageName",imageName_))
      imageName_ = "output";
    printf("imageName_ is %s\n", imageName_.c_str());
    //****** data transmission
    if (!nhPrivate_.getParam ("rfDataLimit",rfDataLimit_))
      rfDataLimit_ = IMAGE_DATA_BUFFER_SIZE;
    printf("rfDataLimit_ is %d\n", rfDataLimit_);
    if (!nhPrivate_.getParam ("imageSendMinInterval",imageSendMinInterval_))
      imageSendMinInterval_ = 0.05;
    printf("imageSendMinInterval_ is %f [sec]\n", imageSendMinInterval_);
    //****** route serach
    if (!nhPrivate_.getParam ("rssiBroadcastInterval",rssiBroadcastInterval_))
      rssiBroadcastInterval_ = 0.4;
    printf("rssiBroadcastInterval_ is %f [sec]\n", rssiBroadcastInterval_);
    if (!nhPrivate_.getParam ("rssiRouterTimeoutThre",rssiRouterTimeoutThre_))
      rssiRouterTimeoutThre_ = 5.0;
    printf("rssiRouterTimeoutThre is %f [sec]\n", rssiRouterTimeoutThre_);
    if (!nhPrivate_.getParam ("routeReqInterval",routeReqInterval_))
      routeReqInterval_ = 0.5;
    printf("routeReqInterval is %f [sec]\n", routeReqInterval_);
    //****** router deployment
    if (!nhPrivate_.getParam ("routerDeploymentRssiInterval",routerDeploymentRssiInterval_))
      routerDeploymentRssiInterval_ = 1.0;
    printf("routerDeploymentRssiInterval is %f [sec]\n", routerDeploymentRssiInterval_);
    if (!nhPrivate_.getParam ("idleIntervalBetweenDeployment",idleIntervalBetweenDeployment_))
      idleIntervalBetweenDeployment_ = 4.0;
    printf("idleIntervalBetweenDeployment is %f [sec]\n", idleIntervalBetweenDeployment_);
    if (!nhPrivate_.getParam ("droppingAltitude", droppingAltitude_))
      droppingAltitude_ = 0.12;
    printf("droppingAltitude is %f\n", droppingAltitude_);

    if (!nhPrivate_.getParam ("droppingXYPosThresh", droppingXYPosThresh_))
      droppingXYPosThresh_ = 0.15;
    printf("droppingXYPosThresh is %f\n", droppingXYPosThresh_);

    if (!nhPrivate_.getParam ("obstacleDetectionRssiThresh", obstacleDetectionRssiThresh_))
      obstacleDetectionRssiThresh_ = -60; //-60[dB]
    printf("obstacleDetectionRssiThresh is %f\n", obstacleDetectionRssiThresh_);

    //******* rssi filter
    if (!nhPrivate_.getParam ("iirFilterRxFreq", iirFilterRxFreq_))
      iirFilterRxFreq_ = 1.0;
    printf("iirFilterRxFreq is %f\n", iirFilterRxFreq_);

    if (!nhPrivate_.getParam ("iirFilterCutoffFreq", iirFilterCutoffFreq_))
      iirFilterCutoffFreq_ = 1.0;
    printf("iirFilterCutoffFreq is %f\n", iirFilterCutoffFreq_);

    if (!nhPrivate_.getParam ("shadowingFadingRssiThreshForNonPro", shadowingFadingRssiThreshForNonPro_))
      shadowingFadingRssiThreshForNonPro_ = 1.0;
    printf("shadowingFadingRssiThreshForNonPro is %f\n", shadowingFadingRssiThreshForNonPro_);

    if (!nhPrivate_.getParam ("shadowingFadingRssiThreshForPro", shadowingFadingRssiThreshForPro_))
      shadowingFadingRssiThreshForPro_ = 1.0;
    printf("shadowingFadingRssiThreshForPro is %f\n", shadowingFadingRssiThreshForPro_);

    if (!nhPrivate_.getParam ("multiPathFadingRssiThresh", multiPathFadingRssiThresh_))
      multiPathFadingRssiThresh_ = 1.0;
    printf("multiPathFadingRssiThresh is %f\n", multiPathFadingRssiThresh_);

    //******* looping algorithm
    if (!nhPrivate_.getParam ("sameDistanceThresh",sameDistanceThresh_))
      sameDistanceThresh_ = 1.0;
    printf("sameDistanceThresh_ is %f\n", sameDistanceThresh_);
    if (!nhPrivate_.getParam ("nodesIntervalThresh",nodesIntervalThresh_))
      nodesIntervalThresh_ = 13.0; //[m]
    printf("nodesIntervalThresh_ is %f\n", nodesIntervalThresh_);
    if (!nhPrivate_.getParam ("candidatePivotNodeRssiResTimeoutThresh",candidatePivotNodeRssiResTimeoutThresh_))
      candidatePivotNodeRssiResTimeoutThresh_ = 3.0;
    printf("candidatePivotNodeRssiResTimeoutThresh_ is %f\n", candidatePivotNodeRssiResTimeoutThresh_);
    if (!nhPrivate_.getParam ("goodCandidatePivotNodeRssiThresh",goodCandidatePivotNodeRssiThresh_))
      goodCandidatePivotNodeRssiThresh_ = -63; //[dB]
    printf("goodCandidatePivotNodeRssiThresh_ is %f\n", goodCandidatePivotNodeRssiThresh_);
    if (!nhPrivate_.getParam ("betterCandidatePivotNodeRssiGapThresh",betterCandidatePivotNodeRssiGapThresh_))
      betterCandidatePivotNodeRssiGapThresh_ = -3; //[dB]
    printf("betterCandidatePivotNodeRssiGapThresh_ is %f\n", betterCandidatePivotNodeRssiGapThresh_);

    //******** flag
    if (!nhPrivate_.getParam ("fileOutputFlag",fileOutputFlag_))
      fileOutputFlag_ = false;
    printf("fileOutputFlag is %s\n", fileOutputFlag_ ? ("true") : ("false"));
    if (!nhPrivate_.getParam ("imageSendFlag", imageSendFlag_))
      imageSendFlag_ = true;
    printf("imageSendFlag is %s\n", imageSendFlag_ ? ("true") : ("false"));
    if (!nhPrivate_.getParam ("imageShowFlag",imageShowFlag_))
      imageShowFlag_ = true;
    printf("imageShowFlag is %s\n", imageShowFlag_ ? ("true") : ("false"));
    if (!nhPrivate_.getParam ("aerialFlightControlFlag", aerialFlightControlFlag_))
      aerialFlightControlFlag_ = true;
    printf("aerialFlightControlFlag is %s\n", aerialFlightControlFlag_ ? ("true") : ("false"));
    if (!nhPrivate_.getParam ("routerInitForceActivateFlag",routerInitForceActivateFlag_))
      routerInitForceActivateFlag_ = false;
    printf("routerInitForceActivateFlag is %s\n", routerInitForceActivateFlag_ ? ("true") : ("false"));
    if (!nhPrivate_.getParam ("routerDeploymentFlag", routerDeploymentFlag_))
      routerDeploymentFlag_ = true;
    printf("routerDeploymentFlag is %s\n", routerDeploymentFlag_ ? ("true") : ("false"));
    if (!nhPrivate_.getParam ("loopingDeploymentAlgorithmFlag", loopingDeploymentAlgorithmFlag_))
      loopingDeploymentAlgorithmFlag_ = true;
    printf("loopingDeploymentAlgorithmFlag is %s\n", loopingDeploymentAlgorithmFlag_ ? ("true") : ("false"));
    if (!nhPrivate_.getParam ("routingCmdFlag",routingCmdFlag_))
      routingCmdFlag_ = true;
    printf("routingCmdFlag is %s\n", routingCmdFlag_ ? ("true") : ("false"));
    if (!nhPrivate_.getParam ("rootDirectConnectFlag",rootDirectConnectFlag_))
      rootDirectConnectFlag_ = true;
    printf("rootDirectConnectFlag is %s\n", rootDirectConnectFlag_ ? ("true") : ("false"));
    //+*+*+* for force activate
    if (!nhPrivate_.getParam ("manualDeploymentAlgorithmFlag", manualDeploymentAlgorithmFlag_))
      manualDeploymentAlgorithmFlag_ = false;
    printf("manualDeploymentAlgorithmFlag is %s\n", manualDeploymentAlgorithmFlag_ ? ("true") : ("false"));


    if(imageSendFlag_)
      {
        if(imageShowFlag_) cameraImage_ = new xbee::CameraImage(imageWidth, imageHeight, 1, camNo_);
        else  cameraImage_ = new xbee::CameraImage(imageWidth, imageHeight, 0, camNo_);
      }

    if(fileOutputFlag_)
      {
        if ((fpRssiPosition_ = fopen("rssi_position.txt", "a+")) == NULL) 
          {
            ROS_ERROR("can not open the file, rssi_position");
            ROS_BREAK();
          }
        int time = (int)initTime.toSec();
        fprintf(fpRssiPosition_, ">>>>>>>>>> time: %d[sec] <<<<<<<<<\n", time);
      }


    iirFilterRssi_ = new IirFilter(iirFilterRxFreq_, iirFilterCutoffFreq_);

    //**** publisher
    dropCmdPub_ = nh_.advertise<std_msgs::Empty>("drop_cmd",1);
    dropPositionPub_ = nh_.advertise<std_msgs::Float32MultiArray>("drop_position", 5);
    //**** subscriber
    soldierPositionSub_ = nh_.subscribe<geometry_msgs::PoseStamped>("slam_out_pose", 1, &SoldierMode::soldierPositionCallback, this, ros::TransportHints().tcpNoDelay());
    forceActivateSub_ = nh_.subscribe("force_activate", 1, &SoldierMode::forceActivateCallback, this, ros::TransportHints().tcpNoDelay());
    forceDropActivateSub_ = nh_.subscribe("force_drop_activate", 1, &SoldierMode::forceDropActivateCallback, this, ros::TransportHints().tcpNoDelay());
    forcePivotNodeChangeSub_ = nh_.subscribe("force_pivot_node_change", 1, &SoldierMode::forcePivotNodeChangeCallback, this, ros::TransportHints().tcpNoDelay());
    forceStateShiftSub_ = nh_.subscribe("force_state_shift", 1, &SoldierMode::forceStateShiftCallback, this, ros::TransportHints().tcpNoDelay());


    imageTransactionSub_ = nh_.subscribe("image_transaction", 1, &SoldierMode::imageTransactionCallback, this, ros::TransportHints().tcpNoDelay());


  }

  void SoldierMode::soldierInit()
  {
    //**** XBee soldier init
    soldierState = IDLE_STATE;  
    lowestRouterRssi = RSSI_LOWEST_LEVEL_THRE;
    lastConnectedNodeAddress16 = 0xfffe;
    deploymentSequence = 1;
    pivotNode = COORDINATOR_NODE;
    directLinkFlag = false;
    droppingCmd = false;
    readyToDropNewNodeFlag = false; //for shadowing fading effect
    //looping deployment
    candidatePivotNode = -1; 
    prevDistancePivotSoldier = 0;
    prevDistanceCandidatePivotSoldier = 0;
    foundCandidatePivotNodeTime = ros::Time::now();

    //counter
    routerDeploymentCnt = 0;
    routeDiscoveryCnt = 0;
    rssiBroadcastCnt  = 0;
    routerRouteReqCnt = 0;
    idleToDeploymentCnt = 0;

    //soldier' position
    posData[0] = ROUTER_POSITION_INFO;
    //x
    posData[1] = 0x00; posData[2] = 0x00;
    posData[3] = 0x00; posData[4] = 0x01;
    //y
    posData[5] = 0x00; posData[6] = 0x00;
    posData[7] = 0x00; posData[8] = 0x02;
    //z
    posData[9]= 0x00; posData[10]= 0x03;
    //theta
    posData[11]= 0x00; posData[12]= 0x04;

    posX = 1202;  //test
    posY = -6464; //test
    theta = 157;  //test

    if(routerInitForceActivateFlag_)
      {//all of routers are activated initially
        for(int i = 0; i < nodeTotalNum; i++)
          NODES_INFO_LIST_[i].activateFlag = true;
      }

  }

 
  void SoldierMode::imageShowSpin(const ros::TimerEvent & e)
  {
    if(imageSendFlag_ && imageShowFlag_)
      {
        int c = cv::waitKey(10);
        if( c == 27)
          {
            ROS_INFO("halt this node");
            ros::shutdown();
          }
        try
          {
            cameraImage_->show();
          }
        catch(cv::Exception ex)
          {
            ROS_WARN("can not show image");
          }
      }
  }

  void SoldierMode::spin(const ros::TimerEvent & e)
  {

    //**** soldier state process
    soldierStateProcess();

    //**** receive
    serialInterface_->readPacket();
    if(serialInterface_->getResponse().isAvailable())
      {
        if(serialInterface_->getResponse().getApiId() == ZB_RX_RESPONSE)
          {
            serialInterface_->getResponse().getZBRxResponse(zbRx);
            zbRxProcess(zbRx);
          }
        else if(serialInterface_->getResponse().getApiId() == REMOTE_AT_COMMAND_RESPONSE)
          {//***** rssi at command broadcasted from this device
            serialInterface_->getResponse().getRemoteAtCommandResponse(remoteAtResponse);
            remoteAtResProcess(remoteAtResponse);
          }
        else
          {
            //ROS_INFO("apiID is 0x%x",serialInterface_->getResponse().getApiId());
          }
      }
  }

  void SoldierMode::soldierStateProcess()
  {
    switch(soldierState)
      {
      case IDLE_STATE:
        break;
      case DEPLOYMENT_STATE:
        if(routerDeploymentCnt > routerDeploymentRssiInterval_ * freq_)
          {
            uint8_t rssiCmd[2];
            rssiCmd[0] = 'D'; rssiCmd[1] = 'B';
            remoteAtRequest 
              = RemoteAtCommandRequest(NODES_INFO_LIST_[pivotNode].address64,
                                       NODES_INFO_LIST_[pivotNode].address16,
                                       rssiCmd, NULL, 0);
            serialInterface_->send(remoteAtRequest);
            routerDeploymentCnt = 0;

            // for looping deployment
            if(loopingDeploymentAlgorithmFlag_ && candidatePivotNode > -1)
              {//candidatePivot_node does exist
                ROS_INFO("   send rssi req to candidate pivot node");
                ros::Duration(0.03).sleep();
                remoteAtRequest 
                  = RemoteAtCommandRequest(NODES_INFO_LIST_[pivotNode].address64,
                                           NODES_INFO_LIST_[pivotNode].address16,
                                           rssiCmd, NULL, 0);
                serialInterface_->send(remoteAtRequest);
              }
          }
        //+++++ 更新
        routerDeploymentCnt++;

        //looping deployment
        if(loopingDeploymentAlgorithmFlag_ && candidatePivotNode > -1)
          {
            if(foundCandidatePivotNodeTime.toSec() - ros::Time::now().toSec()
               > candidatePivotNodeRssiResTimeoutThresh_)
              {//time out
                candidatePivotNode = -1;
                //foundCandidatePivotNodeTime = ros::Time::now();
                ROS_INFO("candidate pivot node rssi res time out, bad candidate pivot node");
              }
          }

        break;
      case ROUTE_DISCOVER_STATE:

        if(lowestRouterRssi < RSSI_FIRST_LEVEL_THRE)
          {//best
            soldierState      =  ROUTE_FOUND_STATE;
            lowestRouterRssi  = RSSI_LOWEST_LEVEL_THRE;
            routeDiscoveryCnt = 0;
            printf("\n");
            ROS_INFO("Find Good Neigbour, Shift to ROUTE_FOUND_STATE");
            lastConnectedNodeAddress16 = closetNeighborAddress16;
          }
        else if(routeDiscoveryCnt > rssiRouterTimeoutThre_ * freq_)
          {//time out
            if(lowestRouterRssi < RSSI_SECOND_LEVEL_THRE)
              {
                soldierState =  ROUTE_FOUND_STATE;
                lowestRouterRssi  = RSSI_LOWEST_LEVEL_THRE;
                routeDiscoveryCnt = 0;
                printf("\n");
                ROS_INFO("Find Bad Neigbour, Shift to ROUTE_FOUND_STATE");
                lastConnectedNodeAddress16 = closetNeighborAddress16;
              }
          }
        else
          printf(".");

        //+++++ send broadcast of rssi
        if(rssiBroadcastCnt > rssiBroadcastInterval_ * freq_)
          {
            XBeeAddress64 broadcastAddress = XBeeAddress64(0x00000000, 0x0000ffff);
            uint8_t rssiCmd[2];
            rssiCmd[0] = 'D'; rssiCmd[1] = 'B';
            remoteAtRequest =
              RemoteAtCommandRequest(broadcastAddress, rssiCmd, NULL, 0);
            serialInterface_->send(remoteAtRequest);
            rssiBroadcastCnt = 0;
          }

        //+++++ 更新
        routeDiscoveryCnt++;
        rssiBroadcastCnt++;
        break;
      case ROUTE_FOUND_STATE:
        if(closetNeighborAddress64.getLsb() ==
           NODES_INFO_LIST_[COORDINATOR_NODE].address64.getLsb() 
           && rootDirectConnectFlag_)
          {
            ROS_ERROR("can we be here?");
            directLinkFlag = true;
            soldierState = IMAGE_SEND_STATE;
          }
        else
          {
            if(++routerRouteReqCnt >  routeReqInterval_ * freq_)
              {
                routerRouteReqCnt = 0;
                uint8_t requestCmd[1] = {INFO_FROM_OR_TO_ROUTER};
                zbTx = ZBTxRequest(closetNeighborAddress64, closetNeighborAddress16,
                                   requestCmd, sizeof(requestCmd));
                zbTx.setFrameId(0);
                serialInterface_->send(zbTx);
                printf("\n");
                ROS_INFO("Send Route Req to No.%d Router, addr16 is 0x%x",
                         addressMatching(closetNeighborAddress64),
                         closetNeighborAddress16);
                directLinkFlag = false;
              }

            printf(".");

            //+++++ send broadcast of rssi
            if(++ routeDiscoveryCnt >  2 * rssiBroadcastInterval_ * freq_)
              {
                XBeeAddress64 broadcastAddress 
                  = XBeeAddress64(0x00000000, 0x0000ffff);
                uint8_t rssiCmd[2];
                rssiCmd[0] = 'D'; rssiCmd[1] = 'B';
                remoteAtRequest 
                  = RemoteAtCommandRequest(broadcastAddress, rssiCmd, NULL, 0);
                serialInterface_->send(remoteAtRequest);
                routeDiscoveryCnt = 0;
              }
          }
        break;
      case IMAGE_SEND_STATE:
        routerRouteReqCnt = 0;
        routeDiscoveryCnt = 0;
        rssiBroadcastCnt = 0;

        //------  future work: change the position send to ack to check whether the route can use
        sendPosition();
        ROS_INFO("Send position, Before Sending Image");
        ros::Duration(0.03).sleep();
        if(imageSendFlag_)
          {
            //ROS_INFO("Load Image From Camera");
            //+*+*+*+**+*+*+*+*+*+*+*+*+*+*
            //cameraImage_->capture_image();  //necessary?

            if(fileOutputFlag_)
              {
                char file_prefix[30];
                sprintf(file_prefix,"%d",
                        (int)(ros::Time::now().toSec() - initTime.toSec()));
                cameraImage_->save(cameraImage_->color_img, file_prefix,
                                   cameraImage_->BOTH, saveFileCompressRate_);
              }

            imageProcess();
            serialInterface_->txflush();
            sendImage(COORDINATOR_NODE, cameraImage_->compress_rawdata, 
                       60 + 30*(int)NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.size());
          }

        if(!routingCmdFlag_)
          {//shift to idle state directly
              soldierState = IDLE_STATE;
          }
        else
          {//shift to restart routing state
            if(!directLinkFlag)
              {
                ros::Duration(0.1).sleep();
                uint8_t routingRestartReq[3];
                routingRestartReq[0] = ROUTING_CMD;
                routingRestartReq[1] = ROUTING_RESTART_CMD;
                routingRestartReq[2] = ROUTING_CMD_REQ;

                zbTx = ZBTxRequest(closetNeighborAddress64, routingRestartReq, sizeof(routingRestartReq));
                zbTx.setAddress16(closetNeighborAddress16);
                serialInterface_->send(zbTx);
                printf("restarting routing about all router");

                soldierState = ROUTING_RESTART_STATE;
              }
            else
              {
                soldierState = IDLE_STATE;
              }
          }
        directLinkFlag = false;
        break;
      case ROUTING_STOP_STATE:
        printf(".");
        break;
      case ROUTING_RESTART_STATE:
        printf(".");
        break;
      default:
        break;
      }
  }



  void SoldierMode::forceDropActivateCallback(std_msgs::Int8 msg)
  {
    if(msg.data == 0)
      {
        soldierState = IDLE_STATE;
        deploymentSequence ++;
        newRouterActivateProcess(NORMAL_NODE);
        dropCmdPub_.publish(std_msgs::Empty()); 

        std_msgs::Float32MultiArray position;
        position.data.push_back(posX);
        position.data.push_back(posY);
        dropPositionPub_.publish(position);

        iirFilterRssi_->reset();
        pivotNode = deploymentSequence;
        //looping deployment
        candidatePivotNode = -1; 
        prevDistancePivotSoldier = 0;
        prevDistanceCandidatePivotSoldier = 0;
        foundCandidatePivotNodeTime = ros::Time::now();
      }
    else
      {
        //todo
      }
  }


  void SoldierMode::soldierPositionCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
  {//Msb 
    posData[0] = ROUTER_POSITION_INFO;

    posX = pose_msg->pose.position.x;
    int position_x = pose_msg->pose.position.x * 10; //resolution: 0.1m
    posData[4] = (uint8_t)(0xff & position_x);
    posData[3] = (uint8_t)(0xff & (position_x >> 8));
    posData[2] = (uint8_t)(0xff & (position_x >> 16));
    posData[1] = (uint8_t)(0xff & (position_x >> 24));

    posY = pose_msg->pose.position.y;
    int position_y = pose_msg->pose.position.y * 10;  //resolution: 0.1m
    posData[8] = (uint8_t)(0xff & position_y);
    posData[7] = (uint8_t)(0xff & (position_y >> 8));
    posData[6] = (uint8_t)(0xff & (position_y >> 16));
    posData[5] = (uint8_t)(0xff & (position_y >> 24));

#if 0
    short position_z = (short)msg.data[10];
    posData[10] = (uint8_t)(0xff & position_z);
    posData[9] = (uint8_t)(0xff & (position_z >> 8));
    posZ = (int)position_z;
#endif
    posData[9] = 0;
    posData[10] = 0;

    theta = (tf::getYaw(pose_msg->pose.orientation));
    short position_theta = theta * 100;  //resolution: 0.01(rad)
    posData[12] = (uint8_t)(0xff & position_theta);
    posData[11] = (uint8_t)(0xff & (position_theta >> 8));




    if(loopingDeploymentAlgorithmFlag_ && candidatePivotNode == -1)
      {//just do this process when candidate pivot node is not found
        float min_distance = 10000; //[m]
        int node = -1;

        geometry_msgs::Point central_point = pose_msg->pose.position;
        wavePropagation_->computePotential(central_point);
        double tolerance = wavePropagation_->getTolerance(); 
        geometry_msgs::PoseStamped end_point;

        for(int i = 0; i < nodeTotalNum; i ++)
          {
            if(i == SOLDIER_NODE || i == pivotNode) continue;
            if(NODES_INFO_LIST_[i].activateFlag)
              {
                //get distance of path
                end_point.pose.position.x = NODES_INFO_LIST_[i].posX;
                end_point.pose.position.y = NODES_INFO_LIST_[i].posY;
                end_point.pose.position.z = 0;
                std::vector<geometry_msgs::PoseStamped> plan;
                wavePropagation_->pathCalculation(end_point, tolerance, plan);
                float distanceRouterSoldier = smanNode_->distanceCalc(plan);

                //float distanceRouterSoldier = distanceBetweenTwoNodes(posX, NODES_INFO_LIST_[i].posX, posY, NODES_INFO_LIST_[i].posY);
                if(min_distance > distanceRouterSoldier)
                  {
                    min_distance = distanceRouterSoldier;
                    node = i;
                  }
              }
          }

        //get distance of path
        end_point.pose.position.x = NODES_INFO_LIST_[pivotNode].posX;
        end_point.pose.position.y = NODES_INFO_LIST_[pivotNode].posY;
        end_point.pose.position.z = 0;
        std::vector<geometry_msgs::PoseStamped> plan;
        wavePropagation_->pathCalculation(end_point, tolerance, plan);
        float disancePivotNodeSoldier = smanNode_->distanceCalc(plan);
        //float disancePivotNodeSoldier = distanceBetweenTwoNodes(posX, NODES_INFO_LIST_[pivotNode].posX, posY, NODES_INFO_LIST_[pivotNode].posY);

        if(fabs(min_distance - disancePivotNodeSoldier) < sameDistanceThresh_) //1m
          {
            ROS_INFO("       Found Cadidate Pivot Node: No.%d Node, Distance is %f", node, min_distance);
            candidatePivotNode = node;
            foundCandidatePivotNodeTime = ros::Time::now();
            prevDistancePivotSoldier = disancePivotNodeSoldier;
            prevDistanceCandidatePivotSoldier = min_distance;

          }
      }
  }

#if 0 //deprecated?
  void SoldierMode::soldierAltitudeCallback(const jsk_quadcopter::MirrorModuleDebugConstPtr & altitdue_msg)
  {
    posZ = altitdue_msg->crrPosZ1;
    if(aerialFlightControlFlag_ && droppingCmd)
      {
        idleToDeploymentCnt = 0; // always IDLE STATE

        if(posZ < droppingAltitude_ &&
           fabs(posX - NODES_INFO_LIST_[deploymentSequence].posX) < droppingXYPosThresh_ &&
           fabs(posY - NODES_INFO_LIST_[deploymentSequence].posY) < droppingXYPosThresh_)
          {
            newRouterActivateProcess(NORMAL_NODE); //send activate req to new router
            dropCmdPub_.publish(std_msgs::Empty()); //send drop cmd to servo
            droppingCmd = false;
          }
      }
  }
#endif

  void SoldierMode::imageTransactionCallback(std_msgs::Empty msg)
  {
    printf("            Image Transaction, Shift to ROUTE_DISCOVER_STATE");
    soldierState = ROUTE_DISCOVER_STATE;
  }


  void SoldierMode::forceActivateCallback(std_msgs::Int8 msg)
  {
    soldierState = IDLE_STATE;
    uint8_t joinCmd[1];
    joinCmd[0] = FORCE_JOINT_CMD;

    if(msg.data == 0)
      {
        ROS_INFO("Stimulate No.%d Router to Refresh the Route Info", deploymentSequence - 1);
        zbTx = ZBTxRequest(NODES_INFO_LIST_[deploymentSequence].address64,
                           NODES_INFO_LIST_[deploymentSequence].address16,
                           joinCmd, sizeof(joinCmd));
      }
    else
      {
        ROS_INFO("Stimulate No.%d Router to Refresh the Route Info", msg.data);
        zbTx = ZBTxRequest(NODES_INFO_LIST_[msg.data + 1].address64,
                           NODES_INFO_LIST_[msg.data + 1].address16,
                           joinCmd, sizeof(joinCmd));
      }
    serialInterface_->send(zbTx);
    serialInterface_->send(zbTx);
  }

  void SoldierMode::forcePivotNodeChangeCallback(std_msgs::Int8 msg)
  {
    if(msg.data == 0)
      {
        pivotNode = COORDINATOR_NODE;
        ROS_INFO("change the pivot node to cooridnator node");
      }
  }

  void SoldierMode::forceStateShiftCallback(std_msgs::Int8 msg)
  {
    if(soldierState == IDLE_STATE && msg.data == 1)
      {
        soldierState = DEPLOYMENT_STATE;
        ROS_INFO("shift from IDLE_STATE to DEPLOYMENT_STATE");
      }
    if( msg.data == 0)
      {
        soldierState = IDLE_STATE;
        ROS_WARN("shift to IDLE_STATE ");
      }

  }


  void SoldierMode::imageProcess()
  {
    switch(imageProcessMode_)
    {
    case ORIGINAL_COLORSCALE:
      cameraImage_->compress(cameraImage_->color_img, cameraImage_->JPEG, compressRate_);
      break;
    case ORIGINAL_GREYSACLE:
      cameraImage_->grey_scale();
      cameraImage_->compress(cameraImage_->grey_img, cameraImage_->JPEG, compressRate_);
      break;
    case RESIZED_COLORSCALE:
      if(resizeMode_ == cameraImage_->RESIZE_WITH_ABS_VAL)
        cameraImage_->resizeWithAbsVal(cameraImage_->color_img,resizeWidth_,resizeHeight_);
      else if(resizeMode_ == cameraImage_->RESIZE_WITH_RATE)
        cameraImage_->resizeWithRate(cameraImage_->color_img,resizeWidthRate_,resizeHeightRate_);
      cameraImage_->compress(cameraImage_->resize_img, cameraImage_->JPEG, compressRate_);
      break;
    case RESIZED_GREYSACLE:
      cameraImage_->grey_scale();
      if(resizeMode_ == cameraImage_->RESIZE_WITH_ABS_VAL)
        cameraImage_->resizeWithAbsVal(cameraImage_->grey_img,resizeWidth_,resizeHeight_);
      else if(resizeMode_ == cameraImage_->RESIZE_WITH_RATE)
        cameraImage_->resizeWithRate(cameraImage_->grey_img,resizeWidthRate_,resizeHeightRate_);
      cameraImage_->compress(cameraImage_->resize_img, cameraImage_->JPEG, compressRate_);
      break;
    default:
      ROS_ERROR("wrong mode of image processing!");
      break;
    }
  }

  void SoldierMode::sendPosition()
  {
    if(!directLinkFlag)
      {
        ROS_INFO("              No Direct Link");
        uint8_t buffer[21];
        buffer[0] = NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.size();
        for(int i=0; i<(int)NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.size(); i++)
          {
            buffer[2*i+1] 
              = (uint8_t)(NODES_INFO_LIST_[COORDINATOR_NODE].routeTable[i] >> 8);
            buffer[2*i+2] 
              = (uint8_t)(0xff & NODES_INFO_LIST_[COORDINATOR_NODE].routeTable[i]);
          }
        cSR = CreateSourceRoute(NODES_INFO_LIST_[COORDINATOR_NODE].address64,
                                NODES_INFO_LIST_[COORDINATOR_NODE].address16,
                                buffer, 1+buffer[0]*2);
        cSR.setFrameId(0);
        serialInterface_->send(cSR);
      }
    else
      ROS_INFO("             Direct Link");

    zbTx = ZBTxRequest( NODES_INFO_LIST_[COORDINATOR_NODE].address64,
                        NODES_INFO_LIST_[COORDINATOR_NODE].address16,
                        posData, sizeof(posData));
    zbTx.setFrameId(0);
    serialInterface_->send(zbTx);
  }

  void SoldierMode::sendImage(int node, std::vector<uchar> buffer_, int waittime)
  {
    int limit_byte = rfDataLimit_; //一回に付き最大どれぐらい送れるか?
    int size = buffer_.size();
    int num = buffer_.size() / limit_byte;
    int rest = buffer_.size() % limit_byte;

    printf("               size: %d, num: %d , rest: %d", (int)buffer_.size(), num, rest);
    int cnt = 0;
    int route_length = 1;

    //***  create source route
    if(!directLinkFlag)
      {
        printf("  No Direct Link\n");
        uint8_t buffer[21];
        buffer[0] = NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.size();

        for(int i=0; i<(int)NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.size();i++)
          {
            buffer[2*i+1] 
              = (uint8_t)(NODES_INFO_LIST_[COORDINATOR_NODE].routeTable[i] >> 8);
            buffer[2*i+2] 
              = (uint8_t)(0xff & NODES_INFO_LIST_[COORDINATOR_NODE].routeTable[i]);
          }
        cSR = CreateSourceRoute(NODES_INFO_LIST_[COORDINATOR_NODE].address64,
                                NODES_INFO_LIST_[COORDINATOR_NODE].address16,
                                buffer, 1+buffer[0]*2);
        cSR.setFrameId(0);
        serialInterface_->send(cSR);

        //***  start signal
        uint8_t rfData[4 + NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.size() * 2];
        rfData[0] =  IMAGE_DATA;
        rfData[1] =  IMAGE_DATA_FIRST_BUFFER;
        rfData[2] = (uint8_t)(size&0xff);
        rfData[3] = (uint8_t)(size >> 8) ;

        for(int i=0; i < (int)NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.size(); i++)
          {
            rfData[2*i+4] = (uint8_t)(NODES_INFO_LIST_[COORDINATOR_NODE].routeTable[i] >> 8);
            rfData[2*i+5] = (uint8_t)(0xff & NODES_INFO_LIST_[COORDINATOR_NODE].routeTable[i]);
          }

        zbTx = ZBTxRequest(NODES_INFO_LIST_[COORDINATOR_NODE].address64,
                           NODES_INFO_LIST_[COORDINATOR_NODE].address16,
                           rfData, sizeof(rfData));
        zbTx.setFrameId(0);
        serialInterface_->send(zbTx);

        route_length = (int)NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.size();
      }
    else
      {
        printf("   Direct Link\n");
        //***  start signal
        uint8_t rfData[4];
        rfData[0] = IMAGE_DATA;
        rfData[1] = IMAGE_DATA_FIRST_BUFFER;
        rfData[2] = (uint8_t)(size&0xff);
        rfData[3] = (uint8_t)(size >> 8) ;
        zbTx = ZBTxRequest(NODES_INFO_LIST_[COORDINATOR_NODE].address64,
                           NODES_INFO_LIST_[COORDINATOR_NODE].address16,
                           rfData, sizeof(rfData));
        zbTx.setFrameId(0);
        serialInterface_->send(zbTx);
      }


    ros::Duration(imageSendMinInterval_ * route_length).sleep();

    //***  image data
    //     説明：rf_dataLimit_(32byte)づつ送ることにする.そこは勘である
    unsigned char* rf_data = (unsigned char*)malloc(limit_byte + 2);
    rf_data[0] = IMAGE_DATA;
    rf_data[1] = IMAGE_DATA_OTHER_BUFFER;
    for(int i = 0; i < num; i++)
      {
        for(int j = 0; j < limit_byte; j++)
          {
            rf_data[j+2] = buffer_[cnt++];
          }
        zbTx = ZBTxRequest(NODES_INFO_LIST_[COORDINATOR_NODE].address64,
                           NODES_INFO_LIST_[COORDINATOR_NODE].address16,
                           rf_data, limit_byte + 2);
        zbTx.setFrameId(0);
        
        serialInterface_->send(zbTx);
        ros::Duration(imageSendMinInterval_ * route_length).sleep();
        //imageSendFlag
      }

    rf_data[0] = IMAGE_DATA;
    rf_data[1] = IMAGE_DATA_OTHER_BUFFER;
    for(int j = 0; j < rest; j++)
      {
        rf_data[j+2] = buffer_[cnt++];
      }
    zbTx = ZBTxRequest(NODES_INFO_LIST_[COORDINATOR_NODE].address64,
                       NODES_INFO_LIST_[COORDINATOR_NODE].address16,
                       rf_data, rest + 2);
    zbTx.setFrameId(0);
    serialInterface_->send(zbTx);
    std::cout << "Transcation Finish" <<std::endl;

    free(rf_data);

    //***** bebug
    if(cnt++ != (int)buffer_.size())
      {
        ROS_ERROR("sending byte mismatch");
        ROS_BREAK();
      }
  }

  void SoldierMode::newRouterActivateProcess(uint8_t node_type)
  {
    if(deploymentSequence < nodeTotalNum )
      {
        NODES_INFO_LIST_[deploymentSequence].activateFlag = true;
        NODES_INFO_LIST_[deploymentSequence].posX = posX;
        NODES_INFO_LIST_[deploymentSequence].posY = posY;
        uint8_t rfData[2 + 8];

        rfData[0] =  INFO_FROM_OR_TO_ROUTER; 
        rfData[1] =  node_type; 

        int position_x = posX * 10; //resolution: 0.1m
        rfData[2] = (0xff & (position_x >> 24));
        rfData[3] = (0xff & (position_x >> 16));
        rfData[4] = (0xff & (position_x >> 8));
        rfData[5] = (0xff & position_x);

        int position_y =  posY * 10; //resolution: 0.1m
        rfData[6] = (0xff & (position_y >> 24));
        rfData[7] = (0xff & (position_y >> 16));
        rfData[8] = (0xff & (position_y >> 8));
        rfData[9] = (0xff & position_y);

        zbTx = ZBTxRequest(NODES_INFO_LIST_[deploymentSequence].address64,
                           NODES_INFO_LIST_[deploymentSequence].address16,
                           rfData, sizeof(rfData));
        serialInterface_->send(zbTx);
        ros::Duration(0.05).sleep();
        serialInterface_->send(zbTx);

        ROS_INFO("Activate No.%d Router", deploymentSequence - 1 );
      }
  }

  void SoldierMode::zbRxProcess(ZBRxResponse zbRx)
  {
    //*+*+*+*+  image transaction  *+*+*+*+*
    //*****   route response sent from router
    if(soldierState ==  ROUTE_FOUND_STATE )
      {
        if(closetNeighborAddress16 == zbRx.getRemoteAddress16() && 
           closetNeighborAddress64.getLsb() == zbRx.getRemoteAddress64().getLsb()
           &&  zbRx.getData(0) == INFO_FROM_OR_TO_ROUTER)
          {//zbRx Data = hearder + destation address16 + route
            printf("\n");
            ROS_INFO("Receive Route from Neighbor, nodeNum is %d", zbRx.getData(1));

            int numNode = zbRx.getData(1);
            NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.clear();
            NODES_INFO_LIST_[COORDINATOR_NODE].rssiTable.clear();

            for(int i = 0; i < numNode ; i++)
              {
                uint8_t  rssi = zbRx.getData(3 * i + 2); 
                NODES_INFO_LIST_[COORDINATOR_NODE].rssiTable.push_back(rssi);

                uint16_t  addr = ((uint16_t)zbRx.getData(3 * i + 3) << 8) 
                  + ((uint16_t)zbRx.getData(3 * i + 4));

                NODES_INFO_LIST_[COORDINATOR_NODE].routeTable.push_back(addr);
                int node = addressMatching(addr);
                printf("No.%d Router(0x%x) -> ", node - 1 , addr);
              }
            printf("Coordinator\n ");
            if(!routingCmdFlag_)
              {//send image directly
                soldierState = IMAGE_SEND_STATE;
              }
            else
              {//shift to stop routing state
                soldierState = ROUTING_STOP_STATE;
                ros::Duration(0.03).sleep();
                uint8_t routingStopReq[3];
                routingStopReq[0] = ROUTING_CMD;
                routingStopReq[1] = ROUTING_STOP_CMD;
                routingStopReq[2] = ROUTING_CMD_REQ;

                zbTx = ZBTxRequest(closetNeighborAddress64, routingStopReq, sizeof(routingStopReq));
                zbTx.setAddress16(closetNeighborAddress16);
                serialInterface_->send(zbTx);

                printf("stopping routing about all router");
              }
          }
      }
    else if(soldierState ==  ROUTING_STOP_STATE)
      {
        if(zbRx.getData(0) == ROUTING_CMD &&
           zbRx.getData(1) == ROUTING_STOP_CMD &&
           zbRx.getData(2) == ROUTING_CMD_ACK)
          {
            if(closetNeighborAddress16 == zbRx.getRemoteAddress16() && 
               closetNeighborAddress64.getLsb() == zbRx.getRemoteAddress64().getLsb())
                {//verbose condition check
                  printf("\n");
                  ROS_INFO("Receive Routing Stop Cmd Ack from Closest Router");
                  soldierState = IMAGE_SEND_STATE;
                }
          }
      }
    else if(soldierState ==  ROUTING_RESTART_STATE)
      {
        if(zbRx.getData(0) == ROUTING_CMD &&
           zbRx.getData(1) == ROUTING_RESTART_CMD &&
           zbRx.getData(2) == ROUTING_CMD_ACK)
          {
            if(closetNeighborAddress16 == zbRx.getRemoteAddress16() && 
               closetNeighborAddress64.getLsb() == zbRx.getRemoteAddress64().getLsb())
                {//verbose condition check
                  printf("\n");
                  ROS_INFO("Receive Routing Restart Cmd Ack from Closest Router");

                  soldierState = IDLE_STATE;
                }
          }
      }
    else
      {
        if(zbRx.getData(0) == INFO_FROM_OR_TO_ROUTER && zbRx.getDataLength() == 1)
          {
            int node = addressRegistration(zbRx.getRemoteAddress64(),
                                           zbRx.getRemoteAddress16());
            NODES_INFO_LIST_[node].activateFlag = true;
            ROS_INFO("Activate Req from No.%d router", node - 1);
          }
      }
  }

  void SoldierMode::remoteAtResProcess(RemoteAtCommandResponse remoteAtResponse)
  {
    if (remoteAtResponse.isOk() && remoteAtResponse.getValueLength() > 0)
      {
        //*** RSSI
        if(remoteAtResponse.getCommand()[0] =='D' && 
           remoteAtResponse.getCommand()[1] =='B')
          {
            //**** Record the address16, same function as coordinator
            int node = addressRegistration(remoteAtResponse.getRemoteAddress64()
                                           ,remoteAtResponse.getRemoteAddress16());

            //**** Display
            printf("\n");
            ROS_INFO("Get DB Response from No.%d node, db is %d",
                     node,remoteAtResponse.getValue()[0]);

            if(soldierState == ROUTE_DISCOVER_STATE &&
               NODES_INFO_LIST_[node].activateFlag)
              {
                if(node == COORDINATOR_NODE)
                  {
                    if(remoteAtResponse.getValue()[0] < RSSI_SECOND_LEVEL_THRE
                       && rootDirectConnectFlag_)
                      {
                        directLinkFlag = true;
                        soldierState = IMAGE_SEND_STATE;
                      }
                  }
                else if(remoteAtResponse.getValue()[0] < lowestRouterRssi)
                  {
                    lowestRouterRssi = remoteAtResponse.getValue()[0];
                    closetNeighborAddress16 = remoteAtResponse.getRemoteAddress16();
                    closetNeighborAddress64 = remoteAtResponse.getRemoteAddress64();
                  }
              }


            //***遅めにcoordinator を発見
            if(soldierState == ROUTE_FOUND_STATE && node == COORDINATOR_NODE  &&
               remoteAtResponse.getValue()[0] < RSSI_FIRST_LEVEL_THRE &&
               remoteAtResponse.getValue()[0] <= lowestRouterRssi &&
               rootDirectConnectFlag_)
              {
                directLinkFlag = true;
                soldierState = IMAGE_SEND_STATE;
                ROS_INFO("Better Route: Direct Connect");
              }

            //** State of router deployment
            if(soldierState == DEPLOYMENT_STATE)
              {
                int rssi = -remoteAtResponse.getValue()[0];
                bool newNodeDroppingFlag = false;
                uint8_t node_type = NORMAL_NODE;

                geometry_msgs::PoseStamped central_point,end1_point,end2_point;
                central_point.pose.position.x = posX;
                central_point.pose.position.y = posY;
                central_point.pose.position.z = 0;
                end1_point.pose.position.x = NODES_INFO_LIST_[candidatePivotNode].posX;
                end1_point.pose.position.y = NODES_INFO_LIST_[candidatePivotNode].posY;
                end1_point.pose.position.z = 0;
                end2_point.pose.position.x = NODES_INFO_LIST_[pivotNode].posX;
                end2_point.pose.position.y = NODES_INFO_LIST_[pivotNode].posY;
                end2_point.pose.position.z = 0;

                if(node == pivotNode)
                   {
                     float filteredRssi = 0;
                     iirFilterRssi_->filterFunction(rssi, filteredRssi);

                     ROS_INFO("From Pivot Node No.%d, db is %d, dbFir is %f", 
                              node,rssi,filteredRssi);

                     if(fileOutputFlag_)
                       fprintf(fpRssiPosition_, "%d %d %f %f %f\n", 
                               node, rssi, filteredRssi, posX, posY);

                     float shadowingFadingRssiThresh = 0;
                     if(pivotNode > (proNum + 2 - 1)) //non_pro<->non_pro
                       shadowingFadingRssiThresh = shadowingFadingRssiThreshForNonPro_;
                     else //pro<->pro OR pro<->non_pro
                       shadowingFadingRssiThresh = shadowingFadingRssiThreshForPro_;


                     //obstacle detection (new)
                     if(rssi < obstacleDetectionRssiThresh_)
                       {
                         bool obstacle_existance_flag = false;
                         costmap_2d::Costmap2D costmap;
                         wavePropagation_->getCostmap(costmap);
                         smanNode_->obstacle_detection(costmap, central_point, end1_point, obstacle_existance_flag);

                         if(obstacle_existance_flag)
                           {
                             ROS_ERROR("obstacle exists!!!");
                             //1. obstacle deployment
                             newNodeDroppingFlag = true;
                             serialDeploymentVector.push_back(pivotNode + 1); //store the deployed node

                           }
                       }

                     //shadowing effect + multipath effect (new)
                     if(filteredRssi < shadowingFadingRssiThresh) 
                       {
                         ROS_WARN("shadowing fading effect: weak rssi!!!");
                         readyToDropNewNodeFlag = true;
                       }
                     if(rssi >= multiPathFadingRssiThresh_ && readyToDropNewNodeFlag)
                       {
                         ROS_ERROR("multipath fading effect: weak rssi!!!");
                         //2. shadowing effect + multipath effect (new)
                         newNodeDroppingFlag = true;
                         readyToDropNewNodeFlag = false;

                         serialDeploymentVector.push_back(pivotNode + 1); //store the deployed node
                      }
                  }
                else if(node == candidatePivotNode && loopingDeploymentAlgorithmFlag_)
                  {
                    ROS_INFO("Receive Rssi Response From candidate pivot node: No.%d,rssi:%d",node,rssi);
                    foundCandidatePivotNodeTime = ros::Time::now(); //time reset

                    wavePropagation_->computePotential(central_point.pose.position);
                    std::vector<geometry_msgs::PoseStamped> plan1, plan2;
                    double tolerance = wavePropagation_->getTolerance(); 
                    wavePropagation_->pathCalculation(end1_point, tolerance, plan1);
                    float currDistanceCandidatePivotSoldier = smanNode_->distanceCalc(plan1);
                    wavePropagation_->pathCalculation(end2_point, tolerance, plan2);
                    float currDistancePivotSoldier = smanNode_->distanceCalc(plan2);
                    float distancePivotCandidatePivot = currDistanceCandidatePivotSoldier + currDistancePivotSoldier;

                    if(abs(rssi - iirFilterRssi_->getLastRawValue()) < 10) //same level
                      {//intermediate deployment: not consecutive + vectorsize > 1 => deploy + loop-closing
                        if(distancePivotCandidatePivot > nodesIntervalThresh_ )
                          {//drop midiate new router
                            ROS_WARN("distancePivotCandidatePivot > nodesIntervalThresh_");
                            //are not consecutive(problem for backward movement)
                            if(abs(pivotNode -candidatePivotNode) != 1 )
                              {
                                ROS_ERROR("should drop new module, the nodes interval exceeds the threshold");
                                //3. mediate node deployment
                                newNodeDroppingFlag = true;

                                //TODO the loop closing flag packet for xbee
                                //the size pf serial node relay is bigger than 2 
                                if(serialDeploymentVector.size() > 1)
                                  node_type = LOOP_CLOSING_NODE;

                                serialDeploymentVector.resize(0);
                              }
                          }
                        else
                          {//distancePivotCandidatePivot < nodesIntervalThresh
                            //switch of pivot:  consecutive => just switch, vector < 2 => just switch, not consecutive + vector > 1 => loop-closing
                            //TODO: use distance comparison is not good way to switch the pivot node, calculate the average rssi
                            //if(rssi > iirFilterRssi_->getLastRawValue())
                            ROS_INFO("maybe candidate pivot node is better than pivot node");

                            if(currDistancePivotSoldier > prevDistancePivotSoldier &&
                               currDistanceCandidatePivotSoldier < prevDistanceCandidatePivotSoldier)
                              {
                                //TODO the loop closing flag packet for xbee
                                //the size pf serial node relay is bigger than 2, and not adjascent nodes
                                if(serialDeploymentVector.size() > 1 && abs(pivotNode -candidatePivotNode) != 1)
                                  sendLoopClosingFlag(pivotNode, candidatePivotNode); //TODO: enough?
                                serialDeploymentVector.resize(0);
                            
                                pivotNode = candidatePivotNode;
                                candidatePivotNode = -1;
                                //prevDistancePivotSoldier = 0;
                                //prevDistanceCandidatePivotSoldier = 0;
                                //foundCandidatePivotNodeTime = ros::Time::now();
                                ROS_WARN("change pivot node to candidate pivot node");
                            
                              }
                          }
                      }
                  }
                else
                  {
                  }

                //new node deployment time
                if(newNodeDroppingFlag && !manualDeploymentAlgorithmFlag_)
                  {
                    soldierState = IDLE_STATE;
                    deploymentSequence ++;

                    iirFilterRssi_->reset();
                    pivotNode = deploymentSequence;

                    if( !aerialFlightControlFlag_ )
                      {
                        newRouterActivateProcess(node_type);
                        dropCmdPub_.publish(std_msgs::Empty());
                      }

                    droppingCmd = true;
                    std_msgs::Float32MultiArray position;
                    position.data.push_back(posX);
                    position.data.push_back(posY);
                    dropPositionPub_.publish(position);

                    //looping deployment
                    candidatePivotNode = -1; 
                    prevDistancePivotSoldier = 0;
                    prevDistanceCandidatePivotSoldier = 0;
                    foundCandidatePivotNodeTime = ros::Time::now();
                  }
              }
          }
      }
  }

  void SoldierMode::sendLoopClosingFlag(uint8_t pivot_node, uint8_t candidate_pivot_node)
  {
    uint8_t rfData[2];

    rfData[0] =  LOOP_CLOSING_CMD; 
    rfData[1] = candidate_pivot_node;

    zbTx = ZBTxRequest(NODES_INFO_LIST_[pivot_node].address64,
                       NODES_INFO_LIST_[pivot_node].address16,
                       rfData, sizeof(rfData));
    serialInterface_->send(zbTx);
    ros::Duration(0.05).sleep();
    serialInterface_->send(zbTx);

    ROS_INFO("Send Loop-Closing Flag to No.%d Router", pivot_node);

  }

  float SoldierMode::distanceBetweenTwoNodes(float x1, float x2, float y1, float y2)
  {
    return sqrt((x1 - x2) * (x1 -x2) + (y1 - y2) * (y1 - y2));
  }

  void SoldierMode::feedImages()
  {
    while(threadOk)
      {
        if(imageSendFlag_)
          cameraImage_->capture_image(); //task
      }
  }

}
