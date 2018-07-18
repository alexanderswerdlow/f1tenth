
#include "localmap.h"



DE_Localmap::DE_Localmap() :
    nodeG_(),
    nodeP_("~")
{

    depthSub_ = nodeG_.subscribe ("/depth_image", 1, &DE_Localmap::imageCallback, this);

    cameraInfoSub_ = nodeG_.subscribe ("/camera_info", 1, &DE_Localmap::ci_callback, this);


#ifdef ELEVATION_CLOUD_DEBUG
    imageCloud_pub_ =    nodeG_.advertise<sensor_msgs::PointCloud2>("/elevation_cloud",1);

#endif


    zImagePub_ =     nodeG_.advertise<sensor_msgs::Image>("/elevation_map",1);
    assignImagePub_ =     nodeG_.advertise<sensor_msgs::Image>("/assign_image",1);
    debugImagePub_ =     nodeG_.advertise<sensor_msgs::Image>("/debug_image",1);

    double tval;


    nodeP_.param("mapResolution", mapResolution_,1024);
    nodeP_.param("mapSize", tval,16.0);
    mapSize_ = tval;

    nodeP_.param("numBlocks", numBlocks_,8);
    blockStep_ = mapSize_ / (float)numBlocks_;


    pixelResolution_ = (double)mapResolution_ / mapSize_;

    proc_.pixelResolution_ = pixelResolution_;


    blockMap_.mapSize_ = mapSize_;
    blockMap_.mapResolution_ = mapResolution_;
    blockMap_.pixelResolution_ = pixelResolution_;
    blockMap_.numBlocks_ = numBlocks_;


    nodeP_.param("processMode", processMode_,(int)PM_INTERP);
    nodeP_.param("fuseMode", fuseMode_,(int)FM_OVERWRITE);
    nodeP_.param("output16U", output16U_,true);
    nodeP_.param("mapScale", mapScale_,1000.0);
    nodeP_.param("mapOffset", mapOffset_,10000.0);
    nodeP_.param("mapZeroLevel", mapZeroLevel_,0.0);
    nodeP_.param("mapNotVisibleLevel", mapNotVisibleLevel_,1000.0);

    nodeP_.param("transform2BaseLink", transform2BaseLink_,true);
    nodeP_.param("useLatestTransform", useLatestTransform_,0);

    nodeP_.param("removeLeftImageCols", removeLeftImageCols_,-1);

    nodeP_.param("removeWindowStartCol", removeWindowStartCol_,-1);
    nodeP_.param("removeWindowEndCol", removeWindowEndCol_,-1);
    nodeP_.param("removeWindowStartRow", removeWindowStartRow_,-1);
    nodeP_.param("removeWindowEndRow", removeWindowEndRow_,-1);


    mapZeroLevelf_ = mapZeroLevel_;

    nodeP_.param("minAssignValue", tval,0.1);
    proc_.minAssignValue_ = tval;

    nodeP_.param("minDepthThreshold", tval,0.4);
    proc_.minDepthThreshold_ = tval;


    nodeP_.param("mapFrame", mapFrame_,std::string("map"));
    nodeP_.param("baseLinkFrame", baseFrame_,std::string("base_link"));
    nodeP_.param("localMapFrame", localMapFrame_,std::string("local_map"));

    numRegistered_ = 0;
    totalRegisterTime_ = 0;

    hasCamInfo_ = false;
    hasCam2Base_ = false;

    cAssign_ = cv::Mat(mapResolution_,mapResolution_,CV_32F);
    cZImg_ = cv::Mat(mapResolution_,mapResolution_,CV_32F);

    cZImg_.setTo(mapZeroLevel_);
    cAssign_.setTo(0);

    blockMap_.mapNotVisibleLevel_ = mapNotVisibleLevel_;
    blockMap_.mapBaseLevel_ = mapOffset_;
    blockMap_.Setup();
    blockMap_.SetSafeBlocksTo(mapOffset_);

    poseEstimator_.Initialize(nodeP_);

}



void DE_Localmap::SetupMatrices(tf::Transform &transform)
{
    tf::Matrix3x3 rotMat(transform.getRotation());

    proc_.r11 = rotMat[0][0];
    proc_.r12 = rotMat[0][1];
    proc_.r13 = rotMat[0][2];
    proc_.r21 = rotMat[1][0];
    proc_.r22 = rotMat[1][1];
    proc_.r23 = rotMat[1][2];
    proc_.r31 = rotMat[2][0];
    proc_.r32 = rotMat[2][1];
    proc_.r33 = rotMat[2][2];

    tf::Vector3 nTrans = transform.getOrigin();

    proc_.t1 = nTrans.x();
    proc_.t2 = nTrans.y();
    proc_.t3 = nTrans.z();


}


bool DE_Localmap::GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans)
{
    try{//Try to get the latest avaiable Transform
        tf_listener.lookupTransform(targetFrame, sourceFrame, time, trans);
    }catch(tf::TransformException ex){//if not available, then wait
        (void) ex;
        if(!tf_listener.waitForTransform(targetFrame, sourceFrame, time, ros::Duration(0.05))){
            ROS_WARN_STREAM_THROTTLE(0.5,"DE_Localmap: cannot lookup transform from: " << targetFrame << " to " << sourceFrame);
            return false;
        }
        tf_listener.lookupTransform(targetFrame, sourceFrame, time, trans);
    }

    return true;


}

void DE_Localmap::ci_callback(const sensor_msgs::CameraInfoConstPtr& info)
{
    if (hasCamInfo_) return;
    if (info->P.at(0) == 0) return;
    camInfo_ = *info;

    hasCamInfo_ = true;

    proc_.SetupCam(camInfo_.P[0],camInfo_.P[5],camInfo_.P[2],camInfo_.P[6]);

    ROS_INFO_STREAM("Received camera info!");

}

cv::Point2f DE_Localmap::ConvertPoint(cv::Point2f &p)
{
    return cv::Point2f((p.x-proc_.minXVal_) * proc_.pixelResolution_,(p.y-proc_.minYVal_) * proc_.pixelResolution_);

}


void DE_Localmap::UpdateLocalMapOverwrite(cv::Mat &localMap, const cv::Mat & zImage, const cv::Mat &assignImage, const cv::Vec4i &minMax)
{
    const float *zImageP;
    float *localMapP;
    const float *assignP;

    float minVal = proc_.minAssignValue_;
    int xl,yl;
    float mapScaleF = mapScale_;
    float mapOffsetF = mapOffset_;

    for (yl = minMax[1]; yl < minMax[3];++yl)
    {
        zImageP = zImage.ptr<float>(yl);
        localMapP = localMap.ptr<float>(yl);
        assignP = assignImage.ptr<float>(yl);

        for (xl = minMax[0]; xl < minMax[2];++xl)
        {
            if (assignP[xl] >= minVal)
            {
                localMapP[xl] = (zImageP[xl]/(assignP[xl]))*mapScaleF+mapOffsetF;

            }

        }
    }

}

void DE_Localmap::UpdateLocalMapMax(cv::Mat &localMap, const cv::Mat & zImage, const cv::Mat &assignImage, const cv::Vec4i &minMax)
{
    const float *zImageP;
    float *localMapP;
    const float *assignP;

    float minVal = proc_.minAssignValue_;
    int xl,yl;
    float mapScaleF = mapScale_;
    float mapOffsetF = mapOffset_;

    //float zVal = 0
    for (yl = minMax[1]; yl < minMax[3];++yl)
    {
        zImageP = zImage.ptr<float>(yl);
        localMapP = localMap.ptr<float>(yl);
        assignP = assignImage.ptr<float>(yl);

        for (xl = minMax[0]; xl < minMax[2];++xl)
        {
            if (assignP[xl] >= minVal)
            {
                const float nval = (zImageP[xl]/(assignP[xl]))*mapScaleF+mapOffsetF;
                if (nval > localMapP[xl]) localMapP[xl] = nval;

            }

        }
    }

}




void DE_Localmap::imageCallback(const sensor_msgs::ImageConstPtr& depth)
{


    if (!hasCamInfo_) return;

    std::string cameraFrame = depth->header.frame_id;
    ros::Time timeStamp = depth->header.stamp;
    if (useLatestTransform_ == 1) timeStamp = ros::Time::now();
    if (useLatestTransform_ == 2) timeStamp = ros::Time(0);

    if (!hasCam2Base_)
    {

        if (!GetTransform(ros::Time(0),baseFrame_, cameraFrame, cam2Base_)) {
            ROS_ERROR_STREAM("Error looking up Camera to Base transform: " << cameraFrame << " to " << baseFrame_);

            return;

        }
        hasCam2Base_ = true;
    }


    tf::StampedTransform base2map;
    bool lookUpOk = GetTransform(timeStamp,mapFrame_, baseFrame_, base2map);
    if (!lookUpOk) {
        ROS_ERROR_STREAM("Error looking up Base to Map transform: " << baseFrame_ << " to " << mapFrame_ << " Time: " << timeStamp);

        return;

    }

    cv::Point2f robotPos;
    robotPos.x = (base2map.getOrigin().x());
    robotPos.y = (base2map.getOrigin().y());

    cv::Point2d robotPosD;
    robotPosD.x = (base2map.getOrigin().x());
    robotPosD.y = (base2map.getOrigin().y());


    tf::Matrix3x3 rotMat(base2map.getRotation());
    double bRoll,bPitch,bYaw;
    rotMat.getRPY(bRoll,bPitch,bYaw);

    timeval tZstart;
    gettimeofday(&tZstart, NULL);

    poseEstimator_.UpdateLocalMap(blockMap_.currentMap_,blockMap_.origin_);

    if (poseEstimator_.UseEstimate())
    {
        poseEstimator_.GetEstimate(base2map);
    }
    tf::Transform cam2map;
    cam2map = base2map*cam2Base_;




    cv::Mat cvDepth = cv_bridge::toCvShare(depth,"")->image;


    SetupMatrices(cam2map);


    if (cvDepth.type() != CV_32F)
    {
        cv::Mat convImg;
        cvDepth.convertTo(convImg,CV_32F,1.0/1000.0,0);
        cvDepth = convImg;
    }

    if (removeLeftImageCols_ > 0)
    {
        UtilsDepthImage::RemoveLeftBorderNoise(cvDepth,removeLeftImageCols_,0);
    }

    if (removeWindowStartCol_ >= 0 && removeWindowEndCol_ >= 0 && removeWindowStartRow_ >= 0 && removeWindowEndRow_ >= 0)
    {
        UtilsDepthImage::RemoveWindow(cvDepth,removeWindowStartCol_,removeWindowEndCol_,removeWindowStartRow_,removeWindowEndRow_);
    }




    blockMap_.ReCenter(robotPos*(1.0));


    proc_.minXVal_ = blockMap_.origin_.x;
    proc_.minYVal_ = blockMap_.origin_.y;



    cv::Vec4i minMax;


    switch (processMode_) {
    case PM_NN: proc_.ProcessDepthImageNN(cvDepth,cZImg_, cAssign_,minMax, mapScale_,mapOffset_,mapZeroLevel_);  break;
    case PM_MAX: proc_.ProcessDepthImageMaxNN(cvDepth,cZImg_, cAssign_,minMax, mapScale_,mapOffset_,mapZeroLevel_);  break;
    default:proc_.ProcessDepthImage(cvDepth,cZImg_, cAssign_,minMax, mapScale_,mapOffset_,mapZeroLevel_);  break;
    }

    switch (fuseMode_) {
    case FM_MAX: UpdateLocalMapMax(blockMap_.currentMap_,cZImg_, cAssign_,minMax);  break;
    default: UpdateLocalMapOverwrite(blockMap_.currentMap_,cZImg_, cAssign_,minMax);  break;
    }

    cv::Mat resultImg = blockMap_.currentMap_;
    std::string resultFrameID = localMapFrame_;

    if (transform2BaseLink_)
    {
        blockMap_.Transform2BaseLink(robotPosD,bYaw);
        resultImg = blockMap_.baseLinkMap_;
        resultFrameID = baseFrame_;

    }

    if (output16U_)
    {
        cv::Mat tempImg;
        resultImg.convertTo(tempImg,CV_16U);
        resultImg = tempImg;

    }



    timeval tZend;
    gettimeofday(&tZend, NULL);


    float zImgMsElapsed = (float)(tZend.tv_sec - tZstart.tv_sec)*1000.0+ (float)(tZend.tv_usec - tZstart.tv_usec)/1000.0;
    numRegistered_++;
    totalRegisterTime_ += zImgMsElapsed;
    ROS_INFO_STREAM_THROTTLE(3,"ZImage Register Time: " << zImgMsElapsed << " Number: " << numRegistered_ << " AVG: " << totalRegisterTime_/(double)numRegistered_);


    geometry_msgs::PoseStamped localMapPose;

    localMapPose.pose.position.x = blockMap_.origin_.x;
    localMapPose.pose.position.y = blockMap_.origin_.y;
    localMapPose.pose.position.z = 0;

    localMapPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    localMapPose.header.stamp = timeStamp;
    localMapPose.header.frame_id = localMapFrame_;

    tf::Transform tfPose;
    tf::poseMsgToTF(localMapPose.pose,tfPose);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(tfPose, timeStamp, mapFrame_, localMapFrame_));



#ifdef ELEVATION_CLOUD_DEBUG
    if (imageCloud_pub_.getNumSubscribers() > 0) UtilsDem2PC::PublishCloud(timeStamp,mapFrame_,blockMap_.currentMap_,imageCloud_pub_,blockMap_.origin_, blockMap_.pixelResolution_);
#endif

    if (zImagePub_.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage out_z_image;
        out_z_image.header   = depth->header; // Same timestamp and tf frame as input image
        out_z_image.header.stamp   = timeStamp; // Same timestamp and tf frame as input image
        out_z_image.header.frame_id   = resultFrameID; // Same timestamp and tf frame as input image
        if (resultImg.type() == CV_32F)
        {
            out_z_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        }
        else
        {
            out_z_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever
        }
        out_z_image.image    = resultImg; // Your cv::Mat
        zImagePub_.publish(out_z_image.toImageMsg());
    }


    if (assignImagePub_.getNumSubscribers() > 0)
    {

        cv_bridge::CvImage out_assign_image;
        out_assign_image.header   = depth->header; // Same timestamp and tf frame as input image
        out_assign_image.header.stamp   = timeStamp; // Same timestamp and tf frame as input image
        out_assign_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        out_assign_image.image    = cAssign_; // Your cv::Mat
        assignImagePub_.publish(out_assign_image.toImageMsg());
    }


}



