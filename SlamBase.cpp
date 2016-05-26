#include "SlamBase.h"

//有参数的构造函数，读取参数并进行初始化
RGBDSLAM::rgbdslam::rgbdslam(const string strName):m_detector("SIFT"),m_descriptor("SIFT"),m_good_match_threshold(4),
    m_cx(325.5),m_cy(253.5),m_fx(518.0),m_fy(519.0),m_scale(1000.0),m_voxel_grid(0.01),m_visualize_pointcloud(1),
    m_min_good_match(10),m_min_inliers(5),m_max_norm(0.3),m_keyframe_threshold(0.1),m_max_norm_lp(5.0),
    m_check_loop_closure(0),m_nearby_loops(5),m_random_loops(5)
{
    mf_settings.open(strName, cv::FileStorage::READ);
    if(mf_settings.isOpened())
    {
        m_detector = (string)mf_settings["detector"];
        m_descriptor = (string)mf_settings["descriptor"];
        m_good_match_threshold = (int)mf_settings["good_match_threshold"];

        m_cx = (float)mf_settings["camera.cx"];
        m_cy = (float)mf_settings["camera.cy"];
        m_fx = (float)mf_settings["camera.fx"];
        m_fy = (float)mf_settings["camera.fy"];
        m_scale = (float)mf_settings["camera.scale"];

        m_voxel_grid = (float)mf_settings["voxel_grid"];
        m_visualize_pointcloud = (int)mf_settings["visualize_pointcloud"];
        m_min_good_match = (int)mf_settings["min_good_match"];
        m_min_inliers = (int)mf_settings["min_inliers"];
        m_max_norm = (float)mf_settings["max_norm"];

        m_keyframe_threshold = (double)mf_settings["keyframe_threshold"];
        m_max_norm_lp = (float)mf_settings["max_norm_lp"];
        m_check_loop_closure = (int)mf_settings["check_loop_closure"];
        m_nearby_loops = (int)mf_settings["nearby_loops"];
        m_random_loops = (int)mf_settings["random_loops"];
    }

}

RGBDSLAM::rgbdslam::~rgbdslam()
{
    mf_settings.release();
    globalOptimizer.clear();
    keyframes.clear();
}
//*******************//
//处理第一帧数据
//*******************//
int RGBDSLAM::rgbdslam::init(FRAME& frames)//frames为形参
{
    int Num = 0;
    Num ++;
    // 所有的关键帧都放在了这里
//    vector< FRAME > keyframes;
    // initialize
    cout<<"Initializing ..."<<endl;
//    FRAME currFrame;
    frames.frameID = Num; // 当前索引为currIndex
    cout << "currFrame.frameID=" << frames.frameID << endl;

//    next_depthMap.copyTo(depthFrame);
//    next_bgrImage.copyTo(rgbFrame);

//    FRAME currFrame = readFrame( currIndex, pd ); // 上一帧数据
    string detector = m_detector;
    string descriptor = m_descriptor;
    //同时提取关键点与特征描述子
    computeKeyPointsAndDesp(frames, detector, descriptor );
    PointCloud::Ptr cloud = image2PointCloud(frames.m_depth,frames.m_rgb);

    /*******************************
    // 新增:有关g2o的初始化
    *******************************/
    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

//    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm( solver );
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );


    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( frames.frameID);
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );

    keyframes.push_back( frames);
    cout << "当前帧帧数：" << frames.m_depth.size() << endl;
    cout << "关键帧帧数：" << keyframes[0].m_depth.size() << endl;

    double keyframe_threshold = m_keyframe_threshold;
    bool check_loop_closure = (bool)m_check_loop_closure;
}

PointCloud::Ptr RGBDSLAM::rgbdslam::image2PointCloud( cv::Mat& rgb, cv::Mat& depth )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / m_scale;
            p.x = (n - m_cx) * p.z / m_fx;
            p.y = (m - m_cy) * p.z / m_fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f RGBDSLAM::rgbdslam::point2dTo3d( cv::Point3f& point)
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / m_scale;
    p.x = ( point.x - m_cx) * p.z / m_fx;
    p.y = ( point.y - m_cy) * p.z / m_fy;
    return p;
}

int RGBDSLAM::rgbdslam::computeKeyPointsAndDesp(FRAME& frames, string detector, string descriptor )
{
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    cv::initModule_nonfree();
    _detector = cv::FeatureDetector::create( detector.c_str() );
    _descriptor = cv::DescriptorExtractor::create( descriptor.c_str() );

    if (!_detector || !_descriptor)
    {
        cerr<<"Unknown detector or discriptor type !"<<detector<<","<<descriptor<<endl;
        return -1;
    }

    _detector->detect( frames.m_rgb, frames.m_kp );
    _descriptor->compute( frames.m_rgb, frames.m_kp, frames.m_desp );

    return 1;
}

// cvMat2Eigen
Eigen::Isometry3d RGBDSLAM::rgbdslam::cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(0,1);
    T(2,3) = tvec.at<double>(0,2);
    return T;
}

PointCloud::Ptr RGBDSLAM::rgbdslam::joinPointCloud( PointCloud::Ptr original, cv::Mat& rgb, cv::Mat& depth, Eigen::Isometry3d T )
{
//    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb,newFrame.depth);

    PointCloud::Ptr newCloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / m_scale;
            p.x = (n - m_cx) * p.z / m_fx;
            p.y = (m - m_cy) * p.z / m_fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            newCloud->points.push_back( p );
        }
    // 设置并保存点云
    newCloud->height = 1;
    newCloud->width = newCloud->points.size();
    newCloud->is_dense = false;

//    return cloud;

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;

    double gridsize = m_voxel_grid;
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}

CHECK_RESULT RGBDSLAM::rgbdslam::checkKeyframes( FRAME& f1,g2o::SparseOptimizer& opti, bool is_loops)
{
    static int min_inliers = m_min_inliers;
    static double max_norm = m_max_norm;
    static double keyframe_threshold = m_keyframe_threshold;
    static double max_norm_lp = m_max_norm_lp;
    static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );
    // 比较f1 和 f2
    result = estimateMotion( keyframes.back(), f1);
    cout << "result.inliers=" << result.inliers << endl;
    cout << "min_inliers=" << min_inliers << endl;
    if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
        return NOT_MATCHED;
    // 计算运动范围是否太大
    double norm = normofTransform(result.rvec, result.tvec);
    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;   // too adjacent frame
    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f1.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    // 边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    edge->vertices() [0] = opti.vertex( keyframes.back().frameID );
    edge->vertices() [1] = opti.vertex( f1.frameID );
    edge->setRobustKernel( robustKernel );
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
    edge->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(edge);
    return KEYFRAME;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP RGBDSLAM::rgbdslam::estimateMotion( FRAME& frame1, FRAME& frame2)
{
    vector< cv::DMatch > matches;
    cv::FlannBasedMatcher matcher;
    matcher.match( frame1.m_desp, frame2.m_desp, matches );

    cout << "matches=" << matches.size() << endl;

    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = m_good_match_threshold;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }

    cout<<"good matches: "<<goodMatches.size()<<endl;

    if (goodMatches.size() <= 5)
    {
        result.inliers = -1;
        return result;
    }
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.m_kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.m_depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.m_kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt);
        pts_obj.push_back( pd );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = {
        {m_fx, 0, m_cx},
        {0, m_fy, m_cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;
    //显示R和T以及内点
    cout << "result.rvec=" << result.rvec << endl;
    cout << "result.tvec=" << result.tvec << endl;
    cout << "result.inliers=" << result.inliers << endl;
    return result;
}

double RGBDSLAM::rgbdslam::normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

//显示
void RGBDSLAM::rgbdslam::play(CHECK_RESULT result,FRAME& currFrame)
{
   switch (result) // 根据匹配结果不同采取不同策略
 {
 case NOT_MATCHED:
     //没匹配上，直接跳过
     cout<<RED"Not enough inliers."<<endl;
     break;
 case TOO_FAR_AWAY:
     // 太远了，可能出错了
     cout<<RED"Too far away, may be an error."<<endl;
     break;
 case TOO_CLOSE:
     //太近了，也直接跳
     cout<<RESET"Too close, not a keyframe"<<endl;
     break;
 case KEYFRAME:
     cout<<GREEN"This is a new keyframe"<<endl;
     // 不远不近，刚好
     /**
      * This is important!!
      * This is important!!
      * This is important!!
      * (very important so I've said three times!)
      */
     // 检测回环
     if (m_check_loop_closure)
     {
         checkNearbyLoops( keyframes, currFrame, globalOptimizer );
         checkRandomLoops( keyframes, currFrame, globalOptimizer );
     }
     keyframes.push_back( currFrame );
     break;
 default:
     break;
 }


}

void RGBDSLAM::rgbdslam::checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{

    static int nearby_loops = m_nearby_loops;

    // 就是把currFrame和 frames里末尾几个测一遍
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( currFrame, opti, true );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( currFrame, opti, true );
        }
    }
}


void RGBDSLAM::rgbdslam::checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static int random_loops = m_random_loops;
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测

    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes(currFrame, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes(currFrame, opti, true );
        }
    }
}

//优化并保存结果
// 优化
void RGBDSLAM::rgbdslam::optimize(g2o::SparseOptimizer&)
{
    cout<<RESET"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./data/result_after.g2o" );
    cout<<"Optimization done."<<endl;
}

//拼接点云地图并显示
void RGBDSLAM::rgbdslam::SplitJoint()
{
   cout<<"saving the point cloud map..."<<endl;
   PointCloud::Ptr output ( new PointCloud() ); //全局地图
   PointCloud::Ptr tmp ( new PointCloud() );

   pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
   pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
   pass.setFilterFieldName("z");
   pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了

   double gridsize = m_voxel_grid; //分辨图可以在parameters.txt里调
   voxel.setLeafSize( gridsize, gridsize, gridsize );

   for (size_t i=0; i<keyframes.size(); i++)
   {
       // 从g2o里取出一帧
       g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
       cout << "keyframes[i].frameID=" << keyframes[i].frameID << endl;
       Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
       PointCloud::Ptr newCloud = image2PointCloud( keyframes[i].m_rgb, keyframes[i].m_depth); //转成点云
       // 以下是滤波
       voxel.setInputCloud( newCloud );
       voxel.filter( *tmp );
       pass.setInputCloud( tmp );
       pass.filter( *newCloud );
       // 把点云变换后加入全局地图中
       pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
       *output += *tmp;
       tmp->clear();
       newCloud->clear();
   }

   voxel.setInputCloud( output );
   voxel.filter( *tmp );
   //存储
   pcl::io::savePCDFile( "./data/result.pcd", *tmp );

   cout<<"Final map is saved."<<endl;
}
