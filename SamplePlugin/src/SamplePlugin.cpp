#include "SamplePlugin.hpp"

#include <filesystem>

SamplePlugin::SamplePlugin() : RobWorkStudioPlugin("SamplePluginUI", QIcon((std::filesystem::path(__FILE__).parent_path() / "pa_icon.png").c_str()))
{
    setupUi(this);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

    // now connect stuff from the ui component
    connect(_btn_im, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_btn_scan, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_btn0, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_btn1, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_spinBox, SIGNAL(valueChanged(int)), this, SLOT(btnPressed()));

    _framegrabber = NULL;

    _cameras = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize()
{
    log().info() << "INITALIZE"
                 << "\n";

    getRobWorkStudio()->stateChangedEvent().add(
        std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Auto load workcell

    std::filesystem::path wc_path(__FILE__);
    wc_path = wc_path.parent_path() / "../../WorkCell/Scene.wc.xml";
    std::cout << "wc path: " << wc_path << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wc_path.string());

    getRobWorkStudio()->setWorkCell(wc);
}

void SamplePlugin::open(WorkCell *workcell)
{
    log().info() << "OPEN"
                 << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL)
    {
        // Add the texture render to this workcell if there is a frame for texture
        Frame *textureFrame = _wc->findFrame("MarkerTexture");
        if (textureFrame != NULL)
        {
            getRobWorkStudio()->getWorkCellScene()->addRender(
                "TextureImage", _textureRender, textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame *bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL)
        {
            getRobWorkStudio()->getWorkCellScene()->addRender(
                "BackgroundImage", _bgRender, bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame *cameraFrame = _wc->findFrame(_cameras[0]);
        if (cameraFrame != NULL)
        {
            if (cameraFrame->getPropertyMap().has("Camera"))
            {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss(camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new GLFrameGrabber(width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber->init(gldrawer);
            }
        }

        Frame *cameraFrame25D = _wc->findFrame(_cameras25D[0]);
        if (cameraFrame25D != NULL)
        {
            if (cameraFrame25D->getPropertyMap().has("Scanner25D"))
            {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam =
                    cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
                std::istringstream iss(camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D = new GLFrameGrabber25D(width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber25D->init(gldrawer);
            }
        }

        _device = _wc->findDevice("UR-6-85-5-A");
        m_Robot = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
        mGripper = _wc->findDevice("WSG50");
        if (m_Robot.isNull())
        {
            RW_THROW("Could not find device... Check Model!");
        }
        if (mGripper.isNull())
        {
            RW_THROW("Could not find the gripper... Check Model!");
        }
        m_sGraspTargetName = "";
        SetNewMountingPostionAndNewHomePos();
        GoToHomePostion();
        _step = -1;
    }
}

void SamplePlugin::close()
{
    log().info() << "CLOSE"
                 << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
    Frame *textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL)
    {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage", textureFrame);
    }
    // Remove the background render
    Frame *bgFrame = _wc->findFrame("Background");
    if (bgFrame != NULL)
    {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage", bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL)
    {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image &img)
{
    Mat res(img.getHeight(), img.getWidth(), CV_8SC3);
    res.data = (uchar *)img.getImageData();
    return res;
}

void SamplePlugin::btnPressed()
{
    QObject *obj = sender();
    if (obj == _btn0)
    { // Calculate  path button
        // VisionM4();
        //ExecutePointTopointPlanner();
        ExecuteParabolic();
        _timer->stop();
        GoToHomePostion();
    }
    else if (obj == _btn1)
    { // Run path button
        log().info() << "Button 1\n";

        // MoveTheRobot();

        // GrapTheBottel();
        // CheckReachability();

        // Toggle the timer on and off
        // if (!_timer->isActive ()) {
        //     _timer->start (100);    // run 10 Hz
        //     _step = 0;
        // }
        // else
        //     _step = 0;
    }
    else if (obj == _spinBox)
    {
        log().info() << "spin value:" << _spinBox->value() << "\n";
    }
    else if (obj == _btn_im)
    {
        getImage();
    }
    else if (obj == _btn_scan)
    {
        get25DImage();
    }
}

void SamplePlugin::get25DImage()
{
    if (_framegrabber25D != NULL)
    {
        for (size_t i = 0; i < _cameras25D.size(); i++)
        {
            // Get the image as a RW image
            Frame *cameraFrame25D = _wc->findFrame(_cameras25D[i]); // "Camera");
            _framegrabber25D->grab(cameraFrame25D, _state);

            // const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud *img = &(_framegrabber25D->getImage());

            std::ofstream output(_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth() << "\n";
            output << "HEIGHT " << img->getHeight() << "\n";
            output << "POINTS " << img->getData().size() << "\n";
            output << "DATA ascii\n";
            for (const auto &p_tmp : img->getData())
            {
                rw::math::Vector3D<float> p = p_tmp;
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();
        }
    }
}

void SamplePlugin::getImage()
{
    if (_framegrabber != NULL)
    {
        for (size_t i = 0; i < _cameras.size(); i++)
        {
            // Get the image as a RW image
            Frame *cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
            _framegrabber->grab(cameraFrame, _state);

            const rw::sensor::Image *rw_image = &(_framegrabber->getImage());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat(rw_image->getHeight(),
                                    rw_image->getWidth(),
                                    CV_8UC3,
                                    (rw::sensor::Image *)rw_image->getImageData());

            // Convert to OpenCV image
            Mat imflip, imflip_mat;
            cv::flip(image, imflip, 1);
            cv::cvtColor(imflip, imflip_mat, COLOR_RGB2BGR);

            cv::imwrite(_cameras[i] + ".png", imflip_mat);

            // Show in QLabel
            QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p = QPixmap::fromImage(img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap(p.scaled(maxW, maxH, Qt::KeepAspectRatio));
        }
    }
    // std::cout << "Camera_Right" << std::endl;
    printProjectionMatrix("Camera_Right");
    // std::cout << "Camera_Left" << std::endl;
    printProjectionMatrix("Camera_Left");
}

void SamplePlugin::timer()
{
    if (0 <= _step && (size_t)_step < _path.size())
    {
        _device->setQ(_path.at(_step), _state);
        getRobWorkStudio()->setState(_state);
        _step++;
    }
}

void SamplePlugin::stateChangedListener(const State &state)
{
    _state = state;
}

bool SamplePlugin::checkCollisions(Device::Ptr device, const State &state,

                                   const CollisionDetector &detector, const Q &q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q, testState);
    colFrom = detector.inCollision(testState, &data);
    if (colFrom)
    {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++)
        {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
}

void SamplePlugin::createPathRRTConnect(Q from, Q to, double extend, double maxTime)
{
    _device->setQ(from, _state);
    getRobWorkStudio()->setState(_state);
    CollisionDetector detector(_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector, _device, _state);
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(_device),
                                                      constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner =
        RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    _path.clear();
    if (!checkCollisions(_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions(_device, _state, detector, to))
        cout << to << " is in colission!" << endl;
    ;
    Timer t;
    t.resetAndResume();
    planner->query(from, to, _path, maxTime);
    t.pause();

    if (t.getTime() >= maxTime)
    {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

    const int duration = 10;

    if (_path.size() == 2)
    { // The interpolated path between Q start and Q goal is collision
      // free. Set the duration with respect to the desired velocity
        LinearInterpolator<Q> linInt(from, to, duration);
        QPath tempQ;
        for (int i = 0; i < duration + 1; i++)
        {
            tempQ.push_back(linInt.x(i));
        }

        _path = tempQ;
    }
}

void SamplePlugin::printProjectionMatrix(std::string frameName)
{
    Frame *cameraFrame = _wc->findFrame(frameName);
    if (cameraFrame != NULL)
    {
        if (cameraFrame->getPropertyMap().has("Camera"))
        {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss(camParam, std::istringstream::in);
            iss >> fovy >> width >> height;
            // std::cout << std::endl
            //    << "Fovy " << fovy << std::endl
            //  << std::endl;
            double fovy_pixel = height / 2 / tan(fovy * (2 * M_PI) / 360.0 / 2.0);
            // std::cout << std::endl
            //         << "FovyPixel  " << fovy_pixel << std::endl
            //       << std::endl;
            // std::cout << std::endl
            //         << "Width  " << width << std::endl
            //       << std::endl;
            // std::cout << std::endl
            //         << "height  " << height << std::endl
            //       << std::endl;

            Eigen::Matrix<double, 3, 4> KA;
            KA << fovy_pixel, 0, width / 2.0, 0, 0, fovy_pixel, height / 2.0, 0, 0, 0, 1, 0;

            // std::cout << "Intrinsic parameters:" << std::endl;
            // std::cout << KA << std::endl;

            MovableFrame::Ptr pMovableObjectFrane = _wc->findFrame<MovableFrame>("Camera_Right");
            Vector3D<> pObject_Pos = pMovableObjectFrane->getTransform(m_RobotState).P();
            std::cout << "Camera ppp parameters:" << pObject_Pos << std::endl;

            Transform3D<> camPosOGL = cameraFrame->wTf(_state);
            Transform3D<> openGLToVis = Transform3D<>(RPY<>(-Pi, 0, Pi).toRotation3D());
            Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            // std::cout << "Extrinsic parameters:" << std::endl;

            m_Extrinisic = H.e();
            // std::cout << m_Extrinisic << std::endl;
        }
    }
}

/* Home postion */
void SamplePlugin::GoToHomePostion()
{
    rw::math::Math::seed();
    double extend = 0.05;
    double maxTime = 60;
    Q from(6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
    Q to(6, 1.571, -1.61, -1.492, -1.571, 1.492, -0.079); // From pose estimationQ[6]{1.571, -1.61, -1.492, -1.571, 1.492, -0.079}

    createPathRRTConnect(from, to, extend, maxTime);
}

std::vector<Q> SamplePlugin::CheckForColltion(std::vector<Q> vPossibleSoultion, bool bOneSoultion)
{

    std::vector<Q> vCollisionFree;
    rw::proximity::CollisionDetector detector(
        _wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    m_RobotState = _wc->getDefaultState();

    for (size_t i = 0; i < vPossibleSoultion.size(); i++)
    {
        // set the robot in that configuration and check if it is in collision
        m_Robot->setQ(vPossibleSoultion[i], m_RobotState);

        if (!detector.inCollision(m_RobotState))
        {
            vCollisionFree.push_back(vPossibleSoultion[i]); // save the soultion
            if (bOneSoultion)
            {
                break;
            }
        }
    }
    return vCollisionFree;
}

int SamplePlugin::SavePathLoaderFile(std::string sFileName, std::vector<Q> vSolutions, double dSimultionTime)
{

    if (vSolutions.empty())
    {

        std::cout << "There are no soultion avaliable!" << std::endl;
        return -1;
    }
    std::string sTempFilePath = "/home/rovi2022/Desktop/Project/PlayBackFolder/";
    sTempFilePath += sFileName;
    sTempFilePath += ".rwplay";

    m_RobotState = _wc->getDefaultState();

    rw::trajectory::TimedStatePath tStatePath;
    double time = 0;
    for (unsigned int i = 0; i < vSolutions.size(); i++)
    {
        m_Robot->setQ(vSolutions[i], m_RobotState);
        tStatePath.push_back(rw::trajectory::TimedState(time, m_RobotState));
        time += dSimultionTime;
    }
    rw::loaders::PathLoader::storeTimedStatePath(*_wc, tStatePath, sTempFilePath);
    return 0;
}

void SamplePlugin::MoveTheRobot()

{

    MovableFrame::Ptr pMovableBaseFrame = _wc->findFrame<MovableFrame>("URReference");
    m_RobotState = _wc->getDefaultState();
    // Vector3D<> p = (pMovableBaseFrame->getTransform(m_RobotState).P());
    Rotation3D<> Rotaion = pMovableBaseFrame->getTransform(m_RobotState).R();

    int i = 1;
    while (i < 5)
    {
        Vector3D<> newPostion(0.307, 0.2, 0.1); // x, y, z= 0.1 as defult
        std::cout << "New Postion of base frame: " << newPostion << std::endl;
        pMovableBaseFrame->moveTo(Transform3D<>(newPostion, Rotaion), m_RobotState);
        getRobWorkStudio()->setState(m_RobotState);

        break;
    }
    Vector3D<> p = (pMovableBaseFrame->getTransform(m_RobotState).P());
    std::cout << "Postion of base frame: " << p << std::endl;
}

void SamplePlugin::OpenGripper()
{
    m_RobotState = _wc->getDefaultState();
    Q qGripper(0.055);
    mGripper->setQ(qGripper, m_RobotState);
    getRobWorkStudio()->setState(m_RobotState);
}

void SamplePlugin::CloseGripper(TargetObject eTargetObject, bool bTop)
{

    Q qGripper(0.0);
    if (bTop)
    {
        switch (eTargetObject)
        {
        case TargetObject::BOTTEL:
            qGripper(0.027);
            break;
        case TargetObject::CYLINDER:
            qGripper(0.044);
            break;
        case TargetObject::SQUARE:
            qGripper(0.044);
            break;
        default:
            break;
        }
    }

    m_RobotState = _wc->getDefaultState();

    mGripper->setQ(qGripper, m_RobotState);
    getRobWorkStudio()->setState(m_RobotState);
}

// This function is taken from Exercies 3
std::vector<Q> SamplePlugin::getConfigurations(const std::string nameGoal, const std::string nameTcp,
                                               rw::models::SerialDevice::Ptr robot,
                                               rw::models::WorkCell::Ptr wc, State &state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame::Ptr goal_f = wc->findFrame(nameGoal);
    Frame::Ptr tcp_f = wc->findFrame(nameTcp);
    Frame::Ptr robotBase_f = wc->findFrame(nameRobotBase);
    Frame::Ptr robotTcp_f = wc->findFrame(nameRobotTcp);
    if (goal_f.isNull() || tcp_f.isNull() || robotBase_f.isNull() || robotTcp_f.isNull())
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (goal_f.isNull() ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (tcp_f.isNull() ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameRobotBase
                  << "\": " << (robotBase_f.isNull() ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp
                  << "\": " << (robotTcp_f.isNull() ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    Transform3D<> baseTGoal = Kinematics::frameTframe(robotBase_f, goal_f, state);
    Transform3D<> tcpTRobotTcp = Kinematics::frameTframe(tcp_f, robotTcp_f, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = baseTGoal * tcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler =
        rw::core::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robot, state));

    return closedFormSovler->solve(targetAt, state);
}

void SamplePlugin::CheckReachability()
{
    std::string sFolderPath = "/home/rovi2022/Desktop/Project/Reachability/";

    ofstream LogFile;

    MovableFrame::Ptr pMovableRobotBase = _wc->findFrame<MovableFrame>("URReference");

    for (size_t itTarget = 0; itTarget < 6; itTarget++)
    {
        std::string sTempFileName;
        std::string sPlaceFrame;
        switch (itTarget)
        {
        case 0:
            m_eTargetObject = TargetObject::BOTTEL;
            m_eGraspPostion = GraspPostion::CENTER;
            GetStringGraspPostionAndObject(m_eTargetObject, m_eGraspPostion);
            sPlaceFrame = "Place_Center";
            break;
        case 1:
            m_eTargetObject = TargetObject::BOTTEL;
            m_eGraspPostion = GraspPostion::TOP;
            GetStringGraspPostionAndObject(m_eTargetObject, m_eGraspPostion);
            sPlaceFrame = "Place_Top";
            break;
        case 2:
            m_eTargetObject = TargetObject::SQUARE;
            m_eGraspPostion = GraspPostion::TOP;
            GetStringGraspPostionAndObject(m_eTargetObject, m_eGraspPostion);
            sPlaceFrame = "Place_Top";
            break;
        case 3:
            m_eTargetObject = TargetObject::SQUARE;
            m_eGraspPostion = GraspPostion::CENTER;
            GetStringGraspPostionAndObject(m_eTargetObject, m_eGraspPostion);
            sPlaceFrame = "Place_Center";
            // Q startPos =  m_Robot->getQ(m_RobotState);
            break;
        case 4:
            m_eTargetObject = TargetObject::CYLINDER;
            m_eGraspPostion = GraspPostion::TOP;
            GetStringGraspPostionAndObject(m_eTargetObject, m_eGraspPostion);
            sPlaceFrame = "Place_Top";
            break;
        case 5:
            m_eTargetObject = TargetObject::CYLINDER;
            m_eGraspPostion = GraspPostion::CENTER;
            GetStringGraspPostionAndObject(m_eTargetObject, m_eGraspPostion);
            sPlaceFrame = "Place_Center";
            break;
        default:
            break;
        }
        sTempFileName += sFolderPath;
        sTempFileName += m_sGraspTargetName;
        sTempFileName += "_Final";
        sTempFileName += ".csv";
        LogFile.open(sTempFileName);

        MovableFrame::Ptr pMovableObject = _wc->findFrame<MovableFrame>(m_sTargetName);
        if (pMovableRobotBase.isNull())
        {
            RW_THROW("Could not find frame!!\n");
        }

        for (double x = -0.300; x <= 0.305; x += 0.025)
        {
            for (double y = -0.325; y <= 0.175; y += 0.025)
            {
                m_RobotState = _wc->getDefaultState();
                Vector3D<> newPostion(x, y, 0.1); // x, y, z= 0.1 as defult

                getRobWorkStudio()->setState(m_RobotState);

                std::cout << "New base frame postion: " << newPostion << std::endl;
                pMovableRobotBase->moveTo(Transform3D<>(newPostion, pMovableRobotBase->getTransform(m_RobotState).R()), m_RobotState);

                pMovableObject->moveTo(Transform3D<>(Vector3D<>(pMovableObject->getTransform(m_RobotState).P()),
                                                     RPY<>(0, 0, 0)),
                                       m_RobotState);

                std::vector<Q> All_Possible_solutions = getConfigurations(m_sGraspTargetName, "GraspTCP", m_Robot, _wc, m_RobotState);

                std::vector<Q> CollisionFreesolutions = CheckForColltion(All_Possible_solutions, false);

                std::vector<Q> All_Possible_solutions_Place = getConfigurations(sPlaceFrame, "GraspTCP", m_Robot, _wc, m_RobotState);
                std::vector<Q> CollisionFreesolutions_Place = CheckForColltion(All_Possible_solutions_Place, false);
                // std::string sTEmp  = m_sGraspTargetName;
                // sTEmp  += "_Pick";
                // std::string sTEmpPlace  = m_sGraspTargetName;
                // sTEmpPlace  += sPlaceFrame;
                // SavePathLoaderFile(sTEmp ,CollisionFreesolutions);
                // SavePathLoaderFile(sTEmpPlace ,CollisionFreesolutions_Place);
                if (CollisionFreesolutions_Place.size() >= 1)
                {
                    LogFile << x << "," << y << ","
                            << All_Possible_solutions.size() << ","
                            << CollisionFreesolutions.size() << "\n";
                }
                break;
            }
            break;
        }
        LogFile.close();
    }
    std::cout << "----------------------DONE----------------------\n"
              << std::endl;
}

void SamplePlugin::SetNewMountingPostionAndNewHomePos()
{
    m_RobotState = _wc->getDefaultState();

    Vector3D<> newRobotPostion(0.075, -0.15, 0.1); // x, y, z= 0.1 as defult

    MovableFrame::Ptr pMovableRobotBase = _wc->findFrame<MovableFrame>("URReference");

    pMovableRobotBase->moveTo(Transform3D<>(newRobotPostion, pMovableRobotBase->getTransform(m_RobotState).R()), m_RobotState);
    getRobWorkStudio()->setState(m_RobotState);
}

void SamplePlugin::GetStringGraspPostionAndObject(TargetObject eTargetObject, GraspPostion eGraspPostion)
{

    m_sGraspTargetName = "";
    m_sTargetName = "";
    switch (eTargetObject)
    {
    case TargetObject::BOTTEL:
        m_sTargetName = "Bottle";
        switch (eGraspPostion)
        {
        case GraspPostion::TOP:
            m_sGraspTargetName = "GraspTargetBottleTop";
            break;
        case GraspPostion::CENTER:
            m_sGraspTargetName = "GraspTargetBottleCenter";
            break;
        default:
            break;
        }
        break;

    case TargetObject::CYLINDER:
        m_sTargetName = "Cylinder";
        switch (eGraspPostion)
        {
        case GraspPostion::TOP:
            m_sGraspTargetName = "GraspTargetCylinderTop";
            break;
        case GraspPostion::CENTER:
            m_sGraspTargetName = "GraspTargetCylinderCenter";
            break;

        default:
            break;
        }
        break;

    case TargetObject::SQUARE:
        m_sTargetName = "Square";
        switch (eGraspPostion)
        {
        case GraspPostion::TOP:
            m_sGraspTargetName = "GraspTargetSquareTop";

            break;
        case GraspPostion::CENTER:
            m_sGraspTargetName = "GraspTargetSquareCenter";
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/*################################ M4 #########################################################3*/

void SamplePlugin::VisionM4()
{
    MovableFrame::Ptr pMovableObjectFrane = _wc->findFrame<MovableFrame>("Bottle");
    Vector3D<> pObject_Pos = pMovableObjectFrane->getTransform(m_RobotState).P();

    ofstream myfile;
    ofstream myfile2;
    std::string sHomFile = "/home/rovi2022/Desktop/Project/VisionM4/Result.txt";
    myfile.open(sHomFile);

    std::string sHomFile2 = "/home/rovi2022/Desktop/Project/VisionM4/Pos.txt";
    myfile2.open(sHomFile2);

    for (double x = -0.350; x <= 0.350; x += 0.05)
    {

        for (double y = 0.348; y <= 0.523; y += 0.05)
        {
            Vector3D<> newPostion(x, y, pObject_Pos[2]);

            std::cout << "New Postion of base frame: " << newPostion << std::endl;
            pMovableObjectFrane->moveTo(Transform3D<>(newPostion, pMovableObjectFrane->getTransform(m_RobotState).R()), m_RobotState);
            getRobWorkStudio()->setState(m_RobotState);

            getImage();
            m_RobotState = _wc->getDefaultState();
            vector<Point2d> vMatching;
            vector<Point2d> vActualPoint;

            cv::Mat image = TakeImage();

            image = FilterTheObjectFromTheBackground(image, true);

            cv::imwrite("/home/rovi2022/Desktop/Project/VisionM4/Filtered.png", image);
            cv::Mat noise = cv::Mat::zeros(image.size(), image.type());
            cv::randn(noise, cv::Scalar::all(0), cv::Scalar::all(20));

            // Add the noise to the image
            cv::Mat noisy_image;
            cv::add(image, noise, noisy_image);

            // Save the noisy image
            cv::imwrite("/home/rovi2022/Desktop/Project/VisionM4/noisy_image.jpg", noisy_image);

            cv::linemod::Match match = linemod(noisy_image, "Bottel");
            std::cout << "match x" << match.x << " match y" << match.y << " match id" << match.class_id << std::endl;

            image = DrawCircleAtEstimatedPOs(noisy_image, match.x, match.y);

            std::cout << "match: " << match.x << " , " << match.y << " Match id " << match.class_id << std::endl;

            cv::imwrite("/home/rovi2022/Desktop/Project/VisionM4/Resultimage.png", image);

            Eigen::Matrix<double, 3, 3> KA;
            KA(0, 0) = 514.682;
            KA(0, 1) = 0;
            KA(0, 2) = 320;
            KA(1, 0) = 0;
            KA(1, 1) = 514.682;
            KA(1, 2) = 240;
            KA(2, 0) = 0;
            KA(2, 1) = 0;
            KA(2, 2) = 1;
            std::cout << " KA:" << KA << std::endl;

            Eigen::Vector3d PixelPoint;
            PixelPoint(0) = match.x;
            PixelPoint(1) = match.y;
            PixelPoint(2) = 1;
            // PixelPoint << match.x << match.y << 1;

            std::cout << "Pixel point match:" << PixelPoint << std::endl;

            Eigen::Vector3d P(3);

            // cv::Mat P (1,3,cv::DataType<double>::type);
            P = KA.inverse() * PixelPoint;
            std::cout << std::endl
                      << " P:  " << P << std::endl
                      << std::endl;

            Eigen::Matrix<double, 4, 4> H_t;

            H_t = ReadHomogenousTransTemplate(match.class_id); // Read Template Homogueouse transformation

            std::cout << std::endl
                      << "Template homgogenous " << H_t << std::endl;

            double alpha = atan2(P(1), sqrt(pow(P(2), 2) + pow(P(0), 2)));
            double beta = atan((P(0) / P(2)));

            std::cout << "Beta " << beta << " Alpha " << alpha << std::endl;

            Eigen::Matrix<double, 3, 3> RotationX;
            RotationX(0, 0) = 1;
            RotationX(0, 1) = 0;
            RotationX(0, 2) = 0;
            RotationX(1, 0) = 0;
            RotationX(1, 1) = cos(alpha);
            RotationX(1, 2) = -sin(alpha);
            RotationX(2, 0) = 0;
            RotationX(2, 1) = sin(alpha);
            RotationX(2, 2) = cos(alpha);

            std::cout << "RotationX " << RotationX << std::endl;
            Eigen::Matrix<double, 3, 3> RotationY;

            RotationY(0, 0) = cos(beta);
            RotationY(0, 1) = 0;
            RotationY(0, 2) = sin(beta);
            RotationY(1, 0) = 0;
            RotationY(1, 1) = 1;
            RotationY(1, 2) = 0;
            RotationY(2, 0) = -sin(beta);
            RotationY(2, 1) = 0;
            RotationY(2, 2) = cos(beta);

            std::cout << "RotationY " << RotationY << std::endl
                      << std::endl;

            // Transform template mTRIX TO REAL OBJECT

            Eigen::Matrix<double, 4, 4> H_T_C;
            Eigen::Matrix<double, 3, 3> Rotaionxy;
            Eigen::Matrix<double, 4, 4> Rotaionxy_4_4;
            Eigen::Matrix<double, 4, 4> Final_Transformation;

            H_T_C = m_Extrinisic.inverse() * H_t;
            Rotaionxy = RotationY * RotationX;
            std::cout << " H_T_C: " << std::endl
                      << H_T_C << std::endl;
            std::cout << " Rotaionxy: " << std::endl
                      << Rotaionxy << std::endl;

            for (int i = 0; i < Rotaionxy.rows(); i++)
            {
                for (int j = 0; j < Rotaionxy.cols(); j++)
                {
                    Rotaionxy_4_4(i, j) = Rotaionxy(i, j);
                }
            }
            Rotaionxy_4_4(0, 3) = 0;
            Rotaionxy_4_4(1, 3) = 0;
            Rotaionxy_4_4(2, 3) = 0;

            Rotaionxy_4_4(3, 0) = 0;
            Rotaionxy_4_4(3, 1) = 0;
            Rotaionxy_4_4(3, 2) = 0;
            Rotaionxy_4_4(3, 3) = 1;

            std::cout << " Rotaionxy_4_4: " << std::endl
                      << Rotaionxy_4_4 << std::endl;

            Final_Transformation = Rotaionxy_4_4 * H_T_C;
            std::cout << " Final_Transformation: " << std::endl
                      << Final_Transformation << std::endl;

            Eigen::Vector3d EstimatedPos(3);
            EstimatedPos(0) = Final_Transformation(0, 3);
            EstimatedPos(1) = Final_Transformation(1, 3);
            EstimatedPos(2) = Final_Transformation(2, 3);
            std::cout << "################################" << std::endl;
            std::cout << " Estimated pos: " << std::endl
                      << EstimatedPos << std::endl;

            m_RobotState = getRobWorkStudio()->getState();
            Frame *ObjectFrame = _wc->findFrame("Bottle");
            Vector3D<> framepos = ObjectFrame->getTransform(m_RobotState).P();
            std::cout << "Actual Pos: (" << framepos[0] << ", " << framepos[1] << ", " << framepos[2] << ")" << std::endl;

            double Euclidean_d = sqrt((pow(framepos[0] - EstimatedPos(0), 2)) +
                                      (pow(framepos[1] - EstimatedPos(1), 2)) + (pow(framepos[2] - EstimatedPos(2), 2)));

            std::cout << "Euclidean_d " << Euclidean_d << std::endl;

            std::cout << "################################" << std::endl;

            myfile << x << ";" << y << ";" << Euclidean_d << std::endl;
            myfile2 << EstimatedPos << ";" << framepos << std::endl;
        }
    }
}

Eigen::Matrix<double, 4, 4> SamplePlugin::ReadHomogenousTransTemplate(std::string sFileId)
{
    Eigen::Matrix<double, 4, 4> H_t;
    std::string sTempId;
    switch (sFileId.size())
    {
    case 1:
        sTempId = "000";
        sTempId += sFileId;
        break;
    case 2:
        sTempId = "00";
        sTempId += sFileId;
        break;
    case 3:
        sTempId = "0";
        sTempId += sFileId;
        break;

    default:
        sTempId = sFileId;
        break;
    }

    ifstream infile;

    std::string sHomFile = "/home/rovi2022/Desktop/Project/VisionM4/Templates/Bottel/template";
    sHomFile += sTempId;
    sHomFile += "_pose.txt";

    std::cout << "File path " << sHomFile << std::endl;
    // cv::Mat HomographyMatrix(4, 4, cv::DataType<double>::type);

    infile.open(sHomFile);
    if (infile.is_open())
    {
        std::string line;

        int row = 0;
        while (std::getline(infile, line))
        {
            std::istringstream iss(line);
            std::string substring{};
            std::vector<std::string> substrings{};
            double dValue = 0;

            int col = 0;
            while (std::getline(iss, substring, ' '))
            {

                dValue = stod(substring);
                H_t(row, col) = dValue;
                col++;
            }
            row++;
        }
    }
    else
    {
        cout << "Error opening file";
    }

    std::cout << "Template matrix " << H_t << std::endl;
    return H_t;
}

void SamplePlugin::GenerateHomoPoints()
{

    MovableFrame::Ptr pMovableObjectFrane = _wc->findFrame<MovableFrame>("Bottle");
    Vector3D<> pObject_Pos = pMovableObjectFrane->getTransform(m_RobotState).P();

    ofstream myfile;
    std::string sHomFile = "/home/rovi2022/Desktop/Project/VisionM4/Homo.txt";
    myfile.open(sHomFile);

    int nCouter = 0;
    for (double x = -0.050; x <= 0.050; x += 0.05)
    {

        for (double y = 0.473; y <= 0.548; y += 0.025)
        {

            Vector3D<> newPostion(x, y, pObject_Pos[2]);

            std::cout << "New Postion of base frame: " << newPostion << std::endl;
            pMovableObjectFrane->moveTo(Transform3D<>(newPostion, pMovableObjectFrane->getTransform(m_RobotState).R()), m_RobotState);
            getRobWorkStudio()->setState(m_RobotState);

            cv::Mat image = TakeImage();
            cv::imwrite("/home/rovi2022/Desktop/Project/VisionM4/Orignal" + _cameras[0] + "_" + to_string(nCouter) + ".png", image);

            image = FilterTheObjectFromTheBackground(image, true);

            cv::imwrite("/home/rovi2022/Desktop/Project/VisionM4/OrignalFiltered" + _cameras[0] + "_" + to_string(nCouter) + ".png", image);

            cv::linemod::Match match = linemod(image, "Bottel");

            std::cout << "Match ID" << match.class_id << std::endl;
            std::cout << "Match Cord: (" << match.x << ", " << match.y << ")" << std::endl;
            std::cout << "Similarity: (" << match.similarity << std::endl;

            myfile << x << ";" << y << ";" << pObject_Pos[2] << ";" << match.x << ";" << match.y << std::endl;
            std::cout << "Object postion" << x << y << pObject_Pos[2] << std::endl
                      << std::endl;

            image = DrawCircleAtEstimatedPOs(image, match.x, match.y);
            cv::imwrite("/home/rovi2022/Desktop/Project/VisionM4/Match" + to_string(nCouter) + ".png", image);
            nCouter++;
        }
    }

    myfile.close();
    std::cout << "############ Done ###########\n\n";
}

void SamplePlugin::ReadHomorgraphyFile(vector<Point2d> &vMatching, vector<Point2d> &vCordi)
{
    ifstream infile;
    std::string sHomFile = "/home/rovi2022/Desktop/Project/VisionM4/Homo.txt";
    vMatching.clear();
    vCordi.clear();

    infile.open(sHomFile);
    if (infile.is_open())
    {
        std::string line;
        while (std::getline(infile, line))
        {

            // Now we read a complete line into our std::string line
            // Put it into a std::istringstream to be able to extract it with iostream functions
            std::istringstream iss(line);

            // We will use a vector to store the substrings
            std::string substring{};
            std::vector<std::string> substrings{};
            Point2d obj_cordinate;
            Point2d matching_point;
            // Now, in a loop, get the substrings from the std::istringstream
            int i = 0;
            while (std::getline(iss, substring, ';'))
            {
                switch (i)
                {
                case 0:
                    obj_cordinate.x = stod(substring);
                    break;
                case 1:
                    obj_cordinate.y = stod(substring);
                    break;
                case 3:
                    matching_point.x = stod(substring);
                    break;
                case 4:
                    matching_point.y = stod(substring);
                    break;

                default:
                    break;
                }
                i++;
            }
            vCordi.push_back(obj_cordinate);
            vMatching.push_back(matching_point);
        }
    }
    else
    {
        cout << "Error opening file";
    }
    std::cout << "Number of point " << vCordi.size() << std::endl;
}

cv::Mat SamplePlugin::DrawCircleAtEstimatedPOs(cv::Mat img, double x, double y)
{

    cv::Mat image = img;
    Point center(x, y);           // Declaring the center point
    int radius = 5;               // Declaring the radius
    Scalar line_Color(0, 0, 255); // Color of the circle
    int thickness = 2;            // thickens of the line

    circle(image, center, radius, line_Color, thickness); // Using circle()function to draw the line//
    return image;
}

cv::Mat SamplePlugin::FilterTheObjectFromTheBackground(cv::Mat img, bool bGenrate)
{

    cv::Mat image = img;
    cv::Mat absolute_difference;
    string sPath = "/home/rovi2022/Desktop/Project/VisionM4/Static.png";
    if (bGenrate)
    {
        sPath = "/home/rovi2022/Desktop/Project/VisionM4/Static2.png";
    }

    cv::Mat static_img = cv::imread(sPath);
    cv::absdiff(img, static_img, absolute_difference);

    for (double i = 0; i < absolute_difference.rows; i++)
    {
        for (double j = 0; j < absolute_difference.cols; j++)
        {
            Vec3b point(0, 0);

            if (absolute_difference.at<Vec3b>(i, j) == point)
            {
                image.at<Vec3b>(i, j) = point;
            }
        }
    }
    return image;
}

cv::Mat SamplePlugin::TakeImage(bool bLeft)
{

    int n = 0;
    if (!bLeft)
    {
        n = 1;
    }
    // m_RobotState = _wc->getDefaultState();

    Frame *cameraFrame = _wc->findFrame(_cameras[n]); // "Camera");
    _framegrabber->grab(cameraFrame, m_RobotState);

    const rw::sensor::Image *rw_image = &(_framegrabber->getImage());

    // Convert to OpenCV matrix.
    cv::Mat image = cv::Mat(rw_image->getHeight(),
                            rw_image->getWidth(),
                            CV_8UC3,
                            (rw::sensor::Image *)rw_image->getImageData());
    Mat imflip, imflip_mat;
    cv::flip(image, imflip, 1);
    cv::cvtColor(imflip, imflip_mat, COLOR_RGB2BGR);

    return imflip;
}
// Used from Lecture 7
cv::linemod::Detector SamplePlugin::createLinemodDetector()
{
    std::vector<int> pyramid;
    pyramid.push_back(4);
    pyramid.push_back(2);
    pyramid.push_back(1);
    std::vector<cv::Ptr<cv::linemod::Modality>> modals;
    modals.push_back(cv::linemod::Modality::create("ColorGradient"));
    cv::linemod::Detector detector(modals, pyramid);
    return detector;
}

// Used from Lecture 7
cv::linemod::Match SamplePlugin::linemod(cv::Mat camera_img, const std::string object_name)
{
    // Training data to be loaded for the 2D matcher
    std::vector<cv::Mat> templates;

    // Line detector
    cv::linemod::Detector detector = createLinemodDetector();

    // Add images
    for (int cnt = 0; true; ++cnt)
    {
        // Get RGB template
        char tfile[1024];
        sprintf(tfile, "/template%04i.png", cnt);
        cv::Mat t = cv::imread("/home/rovi2022/Desktop/Project/VisionM4/Templates/" + object_name + std::string(tfile), cv::IMREAD_UNCHANGED);

        // If the template wasn't found, return
        if (t.empty())
        {
            break;
        }

        // Update window
        cv::Rect win = autocrop(t);
        win.height += 4;
        win.width += 4;
        win.x -= 2;
        win.y -= -2;
        t = t(win);

        // Extract the red colors from the templates
        cv::Mat out;
        cv::inRange(t, cv::Scalar(0, 0, 244), cv::Scalar(1, 1, 255), out);

        // Insert templates into the detector
        detector.addTemplate({t.clone()},
                             std::string(std::to_string(cnt)),
                             255 - out);
    }

    // Run match detector with the template
    std::vector<cv::linemod::Match> matches;

    const float_t threshold = 0.0F;
    detector.match(std::vector<cv::Mat>{camera_img}, threshold, matches);

    // Ensure a match where found
    if (matches.size() == 0)
    {
        std::cout << "No matches where found" << std::endl;
        return cv::linemod::Match(0, 0, 0, "", 0);
    }

    // Extract the match with best similarity
    return matches.front();
}

// Internal function used by autocrop() // Used from Lecture 7
bool SamplePlugin::isBorder(cv::Mat &edge, cv::Vec3b color)
{
    cv::Mat im = edge.clone().reshape(0, 1);

    bool res = true;
    for (int i = 0; i < im.cols; ++i)
        res &= (color == im.at<cv::Vec3b>(0, i));

    return res;
}
// Used from Lecture 7
cv::Rect SamplePlugin::autocrop(cv::Mat &src)
{
    if (src.type() != CV_8UC3)
    {
        exit(-1);
    }
    cv::Rect win(0, 0, src.cols, src.rows);

    std::vector<cv::Rect> edges;
    edges.push_back(cv::Rect(0, 0, src.cols, 1));
    edges.push_back(cv::Rect(src.cols - 2, 0, 1, src.rows));
    edges.push_back(cv::Rect(0, src.rows - 2, src.cols, 1));
    edges.push_back(cv::Rect(0, 0, 1, src.rows));

    cv::Mat edge;
    int nborder = 0;
    cv::Vec3b color = src.at<cv::Vec3b>(0, 0);

    for (size_t i = 0; i < edges.size(); ++i)
    {
        edge = src(edges.at(i));
        nborder += isBorder(edge, color);
    }

    if (nborder < 4)
        return win;

    bool next;

    do
    {
        edge = src(cv::Rect(win.x, win.height - 2, win.width, 1));
        if ((next = isBorder(edge, color)))
            win.height--;
    } while (next && win.height > 0);

    do
    {
        edge = src(cv::Rect(win.width - 2, win.y, 1, win.height));
        if ((next = isBorder(edge, color)))
            win.width--;
    } while (next && win.width > 0);

    do
    {
        edge = src(cv::Rect(win.x, win.y, win.width, 1));
        if ((next = isBorder(edge, color)))
            win.y++, win.height--;
    } while (next && win.y <= src.rows);

    do
    {
        edge = src(cv::Rect(win.x, win.y, 1, win.height));
        if ((next = isBorder(edge, color)))
            win.x++, win.width--;
    } while (next && win.x <= src.cols);

    return win;
}

// ############################## Planing ################################################

TimedStatePath SamplePlugin::linInterp(Device::Ptr device, State state, Q from, Q passing1, Q passing2, Q passing3, Q passing4, Q passing5, Q to, double duration) //
{

    TimedStatePath res;

    LinearInterpolator<Q> interp1(from, passing1, duration);

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(interp1.x(i), state);
        res.push_back(TimedState(i, state));
    }

    LinearInterpolator<Q> interp2(passing1, passing2, duration);

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(interp2.x(i), state);
        res.push_back(TimedState(i, state));
    }

    LinearInterpolator<Q> interp3(passing2, passing3, duration);

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(interp3.x(i), state);
        res.push_back(TimedState(i, state));
    }

    LinearInterpolator<Q> interp4(passing3, passing4, duration);

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(interp4.x(i), state);
        res.push_back(TimedState(i, state));
    }

    LinearInterpolator<Q> interp5(passing4, passing5, duration);

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(interp5.x(i), state);
        res.push_back(TimedState(i, state));
    }

    LinearInterpolator<Q> interp6(passing5, to, duration);

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(interp6.x(i), state);
        res.push_back(TimedState(i, state));
    }

    return res;
}

TimedStatePath SamplePlugin::InterpParabolic(Device::Ptr device, State state, Q from, Q passing1, Q passing2, Q passing3, Q passing4, Q passing5, Q to, double duration)
{

    TimedStatePath res_para;

    LinearInterpolator<Q> interp1(from, passing1, duration);
    LinearInterpolator<Q> interp2(passing1, passing2, duration);
    LinearInterpolator<Q> interp3(passing2, passing3, duration);
    LinearInterpolator<Q> interp4(passing3, passing4, duration);
    LinearInterpolator<Q> interp5(passing4, passing5, duration);
    LinearInterpolator<Q> interp6(passing5, to, duration);
    
    
    ParabolicBlend<Q> para1(interp1, interp2, duration);
    ParabolicBlend<Q> para2(interp2, interp3, duration);
    ParabolicBlend<Q> para3(interp3, interp4, duration);
    ParabolicBlend<Q> para4(interp4, interp5, duration);
    ParabolicBlend<Q> para5(interp5, interp6, duration);
    //ParabolicBlend<Q> para6(interp6, interp7, duration);

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(para1.x(i), state);
        res_para.push_back(TimedState(i, state));
    }

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(para2.x(i), state);
        res_para.push_back(TimedState(i, state));
    }

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(para3.x(i), state);
        res_para.push_back(TimedState(i, state));
    }

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(para4.x(i), state);
        res_para.push_back(TimedState(i, state));
    }

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(para5.x(i), state);
        res_para.push_back(TimedState(i, state));
    }

    for (double i = 0; i < duration; i += 0.05)
    {
        device->setQ(interp6.x(i), state);
        res_para.push_back(TimedState(i, state));
    }

    return res_para;
}
#include <chrono>
#include <algorithm>
using namespace std::chrono;

void SamplePlugin::ExecutePointTopointPlanner()
{

    m_RobotState = getRobWorkStudio()->getState();
    MovableFrame::Ptr pMovableObject = _wc->findFrame<MovableFrame>("Bottle");
    pMovableObject->moveTo(Transform3D<>(Vector3D<>(pMovableObject->getTransform(m_RobotState).P()),
                                         RPY<>(0, 0, 0)),
                           m_RobotState);

    Q start_;
    Q _end;

    std::vector<Q> All_Possible_solutions = getConfigurations("GraspTargetBottleTop", "GraspTCP", m_Robot, _wc, m_RobotState);
    std::vector<Q> CollisionFreesolutions = CheckForColltion(All_Possible_solutions, true);
    std::cout << CollisionFreesolutions.size() << std::endl;

    for (unsigned int i = 0; i < CollisionFreesolutions.size(); i++)
    {

        start_ = CollisionFreesolutions[i];
        std::cout << start_ << std::endl;
    }

    m_RobotState = getRobWorkStudio()->getState();
    All_Possible_solutions.clear();
    CollisionFreesolutions.clear();
    All_Possible_solutions = getConfigurations("Place_Top", "GraspTCP", m_Robot, _wc, m_RobotState);
    CollisionFreesolutions = CheckForColltion(All_Possible_solutions, true);

    for (unsigned int i = 0; i < CollisionFreesolutions.size(); i++)
    {
        _end = CollisionFreesolutions[i];
        std::cout << _end << std::endl;
    }

    Q PickTwo(6, 1.482, -1.819, -1.54, -1.343, 1.582, 0.386);
    Q PickOne(6, 1.849, -1.918, -1.433, -1.332, 1.585, 0.391);
    Q Pickthree(6, 2.233, -2.205, -1.045, -1.343, 1.55, 0.385);

    for (unsigned int itpick = 0; itpick < 3; itpick++)
    {
        Q middle;
        ofstream myfile;
        std::string spath = "/home/rovi2022/Desktop/Project/Planing/p2pEvulation";
        spath += std::to_string(itpick);
        spath += ".txt";

        myfile.open(spath);
        switch (itpick)
        {
        case 0:
            middle = PickOne;
            break;
        case 1:
            middle = PickTwo;
            break;
        case 2:
            middle = Pickthree;
            break;
        default:
            break;
        }
        for (unsigned int itTest = 0; itTest < 50; itTest++)
        {
            // define movement

            Q start(6, 1.571, -1.61, -1.492, -1.571, 1.492, -0.079);
            Q passing1(6, 1.919, -1.861, -0.915, -1.936, 1.508, 0.248);
            Q passing2(6, 1.919, -1.801, -1.254, -1.657, 1.508, 0.248);
            Q passing3(6, 1.919, -1.801, -1.254, -1.657, 1.508, 0.248);
            Q passing4(6, 0, -1.6, -2, -1, 1.5, -1);
            Q end(-0.64, -1.408, -2.198, -1.061, 1.501, 0.884);

            auto start_timer = std::chrono::high_resolution_clock::now();
            TimedStatePath linearQMotion = linInterp(_device, m_RobotState, start, passing1, passing2, middle, passing3, passing4, end, 1); //

            auto stop_timer = std::chrono::high_resolution_clock::now();
            auto durration = std::chrono::duration_cast<microseconds>(stop_timer - start_timer);
            cout << "Time taken by pnp is: " << durration.count() << " microsecond" << std::endl;
            myfile << durration.count() << std::endl;
            std::string splyfile = "/home/rovi2022/Desktop/Project/Planing/ptp";
            splyfile += std::to_string(itpick);
            splyfile += ".rwplay";
            PathLoader::storeTimedStatePath(*_wc, linearQMotion, splyfile);
        }
        myfile.close();
    }

    // load time stat path
    rw::trajectory::TimedStatePath path = PathLoader::loadTimedStatePath(_wc, "/home/rovi2022/Desktop/Project/Planing/ptp.rwplay");
    _wc->setDefaultState(path.front().getValue());

    vector<Q> qValue;
    vector<double> time;

    for (unsigned int i = 0; i < path.size(); i++)
    {
        Q pathvalue = _device->getQ(path[i].getValue());
        qValue.push_back(pathvalue);
        time.push_back(path[i].getTime());
    }

    std::string sHomFile = "/home/rovi2022/Desktop/Project/Planing/pnp.txt";
    WriteTrajectoryFile(sHomFile, qValue, time);
}

void SamplePlugin::WriteTrajectoryFile(std::string sFileName, vector<Q> qValue, vector<double> time)
{

    ofstream myfile;
    myfile.open(sFileName);
    for (unsigned int i = 0; i < qValue.size(); i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            myfile << qValue[i](j) << ";";
        }
        myfile << std::endl;
    }
    myfile.close();
}

void SamplePlugin::ExecuteParabolic()
{
    m_RobotState = getRobWorkStudio()->getState();
    MovableFrame::Ptr pMovableObject = _wc->findFrame<MovableFrame>("Bottle");
    pMovableObject->moveTo(Transform3D<>(Vector3D<>(pMovableObject->getTransform(m_RobotState).P()),
                                         RPY<>(0, 0, 0)),
                           m_RobotState);

    m_RobotState = getRobWorkStudio()->getState();
    Q PickTwo(6, 1.482, -1.819, -1.54, -1.343, 1.582, 0.386);
    Q PickOne(6, 1.849, -1.918, -1.433, -1.332, 1.585, 0.391);
    Q Pickthree(6, 2.233, -2.205, -1.045, -1.343, 1.55, 0.385);
    
    Q start(6, 1.571, -1.61, -1.492, -1.571, 1.492, -0.079);
    Q passing1(6, 1.919, -1.861, -0.915, -1.936, 1.508, 0.248);
    Q passing2(6, 1.919, -1.801, -1.254, -1.657, 1.508, 0.248);
    Q passing3(6, 1.919, -1.801, -1.254, -1.657, 1.508, 0.248);
    Q passing4(6, 0, -1.6, -2, -1, 1.5, -1);
    Q end(-0.64, -1.408, -2.198, -1.061, 1.501, 0.884);
    
    for (unsigned int itpick = 0; itpick < 3; itpick++)
    {
        Q middle;
        
        ofstream myfile;
        std::string spath = "/home/rovi2022/Desktop/Project/Planing/ParabolicEvulation";
        spath += std::to_string(itpick);
        spath += ".txt";

        myfile.open(spath);
        switch (itpick)
        {
        case 0:
            middle = PickOne;
            break;
        case 1:
            middle = PickTwo;
            break;
        case 2:
            middle = Pickthree;
            break;
        default:
            break;
        }
        for (unsigned int itTest = 0; itTest < 50; itTest++)
        {
             auto start_timer = std::chrono::high_resolution_clock::now();
            TimedStatePath parabolicQMotion = InterpParabolic(_device, m_RobotState, start, passing1, passing2, middle, passing3, passing4, end, 1);
            auto stop_timer = std::chrono::high_resolution_clock::now();
            auto durration = std::chrono::duration_cast<microseconds>(stop_timer - start_timer);
            cout << "Time taken by pnp is: " << durration.count() << " microsecond" << std::endl;
            myfile << durration.count() << std::endl;
            
            std::string splyfile = "/home/rovi2022/Desktop/Project/Planing/parabolic";
            splyfile += std::to_string(itpick);
            splyfile += ".rwplay";
            PathLoader::storeTimedStatePath(*_wc, parabolicQMotion, splyfile);
        }
       
        
        myfile.close();
        
    }

     rw::trajectory::TimedStatePath path = PathLoader::loadTimedStatePath(_wc, "/home/rovi2022/Desktop/Project/Planing/parabolic0.rwplay");
    _wc->setDefaultState(path.front().getValue());

    vector<Q> qValue;
    vector<double> time;

    for (unsigned int i = 0; i < path.size(); i++)
    {
        Q pathvalue = _device->getQ(path[i].getValue());
        qValue.push_back(pathvalue);
        time.push_back(path[i].getTime());
    }

    std::string sHomFile = "/home/rovi2022/Desktop/Project/Planing/parabolic.txt";
    WriteTrajectoryFile(sHomFile, qValue, time);
}
