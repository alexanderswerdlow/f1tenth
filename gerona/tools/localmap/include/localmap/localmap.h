#ifndef DE_LOCALMAP_H
#define DE_LOCALMAP_H


#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"

#include <tf/transform_listener.h>

#include "rgbd2dem2.h"
#include <tf/transform_broadcaster.h>

//#include <tf_conversions/tf_eigen.h>
#include "blockmap.h"
#include "utils_pose_estimator.h"


/// Define if point cloud map is required
#define ELEVATION_CLOUD_DEBUG 1

#ifdef ELEVATION_CLOUD_DEBUG
#include "utils_dem2pc.h"
#endif



/**
 * @brief The local map implementation
 */
class DE_Localmap {

public:

    /**
     * @brief Process mode for the current depth image
     */
    enum PROCESS_MODE { PM_INTERP,PM_NN, PM_MAX};
    /**
     * @brief Fuse mode defines how the transformed image is fused with the map
     */
    enum FUSE_MODE { FM_OVERWRITE,FM_MAX};

    DE_Localmap();

    void imageCallback(const sensor_msgs::ImageConstPtr& depth);
    /**
     * @brief Convert a point from world to map coordinates
     */
    cv::Point2f ConvertPoint(cv::Point2f &p);
    void ci_callback(const sensor_msgs::CameraInfoConstPtr& info);
    /**
     * @brief Lookup a tf transform
     */
    bool GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans);
    /**
     * @brief Setup up transform matrices
     */
    void SetupMatrices(tf::Transform &transform);

    /**
     * @brief Updates the local map with the current depth images, assigning the current height for all valid pixels in the current depth image
     */
    void UpdateLocalMapOverwrite(cv::Mat &localMap, const cv::Mat & zImage, const cv::Mat &assignImage, const cv::Vec4i &minMax);
    /**
     * @brief Updates the local map with the current depth images, assigning the maximum of local map and current height values
     */
    void UpdateLocalMapMax(cv::Mat &localMap, const cv::Mat & zImage, const cv::Mat &assignImage, const cv::Vec4i &minMax);


private:
    ros::NodeHandle nodeG_;
    ros::NodeHandle nodeP_;



    ros::Subscriber depthSub_;

    ros::Subscriber cameraInfoSub_;

    ros::Publisher zImagePub_;
    ros::Publisher assignImagePub_;
#ifdef ELEVATION_CLOUD_DEBUG
    ros::Publisher imageCloud_pub_;
    ros::Publisher debugImagePub_;

#endif

    tf::TransformListener tf_listener;


    sensor_msgs::CameraInfo camInfo_;
    bool hasCamInfo_;
    bool hasCam2Base_;

    std::string baseFrame_;
    std::string mapFrame_;
    std::string localMapFrame_;

    tf::StampedTransform cam2Base_;


    int numRegistered_;
    double totalRegisterTime_;


    int mapResolution_;
    float mapSize_;
    float pixelResolution_;
    int numBlocks_;
    float blockStep_;

    int processMode_;
    int fuseMode_;
    //bool transposeImage_;
    bool transform2BaseLink_;

    int removeLeftImageCols_;

    int removeWindowStartCol_;
    int removeWindowEndCol_;
    int removeWindowStartRow_;
    int removeWindowEndRow_;

    bool output16U_;

    int useLatestTransform_;
    //bool useMultiChannel_;

    double mapScale_,mapOffset_, mapZeroLevel_,mapNotVisibleLevel_;
    float mapZeroLevelf_;
    ZImageProc proc_;

    cv::Mat cZImg_,cAssign_;
    //CVAlignedMat::ptr acZImg_,acAssign_;
    BlockMap blockMap_;

    //bool usePoseEstimator_;
    //bool hasPoseEstimator_;
    //PoseEstimator poseEstimator_;

    UtilsPoseEstimator poseEstimator_;




};





#endif //DE_LOCALMAP_H

