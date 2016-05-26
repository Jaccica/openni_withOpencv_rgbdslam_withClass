#include "SlamBase.h"

int main()
{

//    cv::Mat depthMap,bgrImage;
    cout << "opening device(s)" << endl;

    cv::VideoCapture sensor1;sensor1.open(CV_CAP_OPENNI_ASUS);

    if( !sensor1.isOpened() )
    {
        cout << "Can not open capture object 1." << endl;
        return -1;
    }

    cv::Mat next_depthMap,next_bgrImage;
    if( !sensor1.grab() )
    {
        cout << "Sensor1 can not grab images." << endl;
        return -1;
    }
    sensor1.grab();
    sensor1.retrieve( next_depthMap, CV_CAP_OPENNI_DEPTH_MAP );
    sensor1.retrieve( next_bgrImage, CV_CAP_OPENNI_BGR_IMAGE );

    //*******************//
    //处理第一帧数据
    //*******************//
    RGBDSLAM::rgbdslam RS("parameters.yaml");

    FRAME firstFrame;
    next_depthMap.copyTo(firstFrame.m_depth);
    next_bgrImage.copyTo(firstFrame.m_rgb);
    cout << "第一帧深度尺寸：" << firstFrame.m_depth.size() << endl;
    RS.init(firstFrame); //firstFrame为实参

    //******************//
    //处理第二帧及之后的数据,主要提取关键帧
    //******************//
    int Num = 1;
    for(;;)
    {
        Num++;
        sensor1.grab();
        sensor1.retrieve( next_depthMap, CV_CAP_OPENNI_DEPTH_MAP );
        sensor1.retrieve( next_bgrImage, CV_CAP_OPENNI_BGR_IMAGE );
        FRAME currFrame;
        next_depthMap.copyTo(currFrame.m_depth);
        next_bgrImage.copyTo(currFrame.m_rgb);

        currFrame.frameID = Num;  //将当前的帧数赋给frameID
        //同时提取关键点与特征描述子
        RS.computeKeyPointsAndDesp(currFrame, RS.m_detector, RS.m_descriptor );
        cout << "RS.m_detector of currFrame:" << currFrame.m_kp.size() << endl;//是否检测到特征点

        CHECK_RESULT result = RS.checkKeyframes(currFrame, RS.globalOptimizer );
 //       cout << "RS.keyframes.back().m_depth.size()=" << KEYFRAMES.back().m_depth.size() << endl;

        RS.play(result,currFrame);
        cv::imshow("DEPTH",currFrame.m_depth);
        cv::imshow("BGR",currFrame.m_rgb);
        //按ESC键跳出循环
       if( cv::waitKey( 33 ) == 27 )
         {
            break;
         }
    }
    //******************//
    //g2o优化关键帧
    //******************//
    RS.optimize(RS.globalOptimizer);
    RS.SplitJoint();

    return 0;
}
