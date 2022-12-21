#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>


// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/linemod.hpp>
// Qt
#include <QTimer>

#include "ui_SamplePlugin.h"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <functional>

#include <QApplication>
#include <QChartView>
#include <QLineSeries>
#include <QMainWindow>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


using namespace rws;

using namespace cv;

using namespace std::placeholders;

enum class TargetObject{
    BOTTEL,
    CYLINDER,
    SQUARE,
    NOT_DEFINED
};
enum class GraspPostion{
    TOP,
    CENTER,
    NOT_DEFINED
};

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();
    void timer();
    void getImage();
    void get25DImage();
  
    void stateChangedListener(const rw::kinematics::State& state);

    bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q);
    
    void createPathRRTConnect(Q from, Q to,  double extend, double maxTime);
	void printProjectionMatrix(std::string frameName);

    void GoToHomePostion();
    int SavePathLoaderFile(std::string sFileName, std::vector< Q > vSolutions, double dSimultionTime = 0.01);
    std::vector<Q> CheckForColltion (std::vector<Q> vPossibleSoultion, bool bOneSoultion= true);

    void MoveTheRobot();
    void OpenGripper();
    void CloseGripper(TargetObject eTargetObject, bool bTop);
    std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                    rw::models::SerialDevice::Ptr robot,
                                    rw::models::WorkCell::Ptr wc, State& state);

    void CheckReachability();
    void GetStringGraspPostionAndObject(TargetObject eTargetObject, GraspPostion eGraspPostion);
    void SetNewMountingPostionAndNewHomePos();



    void VisionM4();
    

    cv::linemod::Detector createLinemodDetector();
    cv::linemod::Match linemod(cv::Mat camera_img, const std::string object_name);
    cv::Rect autocrop(cv::Mat &src);
    bool isBorder(cv::Mat &edge, cv::Vec3b color);


    void ReadHomorgraphyFile(vector<Point2d>& vMatching,vector<Point2d>& vCordi);
    void GenerateHomoPoints();
    cv::Mat TakeImage(bool bLeft = true);
    cv::Mat FilterTheObjectFromTheBackground(cv::Mat img,bool bGenrate = false );
    cv::Mat DrawCircleAtEstimatedPOs(cv::Mat img, double x, double y);


    Eigen::Matrix<double, 4, 4>  ReadHomogenousTransTemplate(std::string sFileId);



    TimedStatePath linInterp (Device::Ptr device, State state, Q from, Q passing1, Q passing2, Q passing3, Q passing4, Q passing5, Q to, double duration);
    TimedStatePath InterpParabolic (Device::Ptr device, State state, Q from, Q passing1, Q passing2, Q passing3, Q passing4, Q passing5, Q to, double duration);
    void ExecutePointTopointPlanner();
    void ExecuteParabolic();
    void WriteTrajectoryFile(std::string sFileName, vector <Q> qValue, vector <double> time);
    


private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QTimer* _timer;
    QTimer* _timer25D;
    
    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    State m_RobotState;

    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;    
    std::vector<std::string> _cameras;
    std::vector<std::string> _cameras25D;
    Device::Ptr _device;
    Device::Ptr mGripper;

    rw::models::SerialDevice::Ptr m_Robot;
    QPath _path;


    std::string  m_sGraspTargetName;
    std::string  m_sTargetName;
    TargetObject m_eTargetObject;
    GraspPostion m_eGraspPostion;

    Eigen::Matrix<double, 4, 4> m_Extrinisic;
    int _step;

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
