#ifndef SLAMBASE_H
#define SLAMBASE_H
// 各种头文件
// C++标准库
#include <fstream>
#include <vector>
#include <string>
#include <map>
using namespace std;
//using namespace cv;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/eigen.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

//g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 把g2o的定义放到前面
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

enum CHECK_RESULT{NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};

// 帧结构
struct FRAME
{
    cv::Mat m_rgb, m_depth; //该帧对应的彩色图与深度图
    cv::Mat m_desp;       //特征描述子
    vector<cv::KeyPoint> m_kp; //关键点
    int frameID;
};
// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};



namespace RGBDSLAM{

class rgbdslam
{
protected:
    //保存关键帧结果
    vector< FRAME > keyframes;
// PnP 结果
    RESULT_OF_PNP result;

//    cv::Mat rvec, tvec;
//    int inliers;
public:
    g2o::SparseOptimizer globalOptimizer;
    cv::FileStorage mf_settings;
    //特征类型
    string m_detector;
    string m_descriptor;
    int m_good_match_threshold;
    //camera
    float m_cx;
    float m_cy;
    float m_fx;
    float m_fy;
    float m_scale;
    //点云分辨率
    float m_voxel_grid ;
    //是否实时可视化
    int m_visualize_pointcloud;
    //最小匹配数量
    int m_min_good_match;
    //最小内点
    int m_min_inliers;
    //最大运动误差
    float m_max_norm;

    double m_keyframe_threshold;
    float m_max_norm_lp;
    int m_check_loop_closure;
    int m_nearby_loops;
    int m_random_loops;
//本类及类外的函数调用
public:
    rgbdslam(const string strName);

    ~rgbdslam();

    int init(FRAME& frames);

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
    int computeKeyPointsAndDesp(FRAME& frames, string detector, string descriptor );

    CHECK_RESULT checkKeyframes( FRAME& f1, g2o::SparseOptimizer& opti, bool is_loops=false);

    void play(CHECK_RESULT result,FRAME& currFrame);

    void optimize(g2o::SparseOptimizer&);

    void SplitJoint();

//本类的函数调用，及派生类的函数调用
protected:
    // 函数接口
    // image2PointCloud 将rgb图转换为点云
        PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth);

    // point2dTo3d 将单个点从图像坐标转换为空间坐标
    // input: 3维点Point3f (u,v,d)
        cv::Point3f point2dTo3d( cv::Point3f& point);

        RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2);

    // cvMat2Eigen
        Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

    // joinPointCloud
        PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, cv::Mat& rgb, cv::Mat& depth, Eigen::Isometry3d T );

        double normofTransform( cv::Mat rvec, cv::Mat tvec);

        void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);

        void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);

};

}
#endif // SLAMBASE_H
