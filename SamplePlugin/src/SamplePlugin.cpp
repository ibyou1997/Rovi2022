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
        if (m_Robot.isNull())
        {
            RW_THROW("Could not find device... Check Model!");
        }
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
    { // Calculate path button

        _timer->stop();
        GoToHomePostion();
    }
    else if (obj == _btn1)
    { // Run path button
        log().info() << "Button 1\n";
        // MoveTheRobot();
        GrapTheBottel();
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
    std::cout << "Camera_Right" << std::endl;
    printProjectionMatrix("Camera_Right");
    std::cout << "Camera_Left" << std::endl;
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

            double fovy_pixel = height / 2 / tan(fovy * (2 * M_PI) / 360.0 / 2.0);

            Eigen::Matrix<double, 3, 4> KA;
            KA << fovy_pixel, 0, width / 2.0, 0, 0, fovy_pixel, height / 2.0, 0, 0, 0, 1, 0;

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            Transform3D<> camPosOGL = cameraFrame->wTf(_state);
            Transform3D<> openGLToVis = Transform3D<>(RPY<>(-Pi, 0, Pi).toRotation3D());
            Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e() << std::endl;
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
    Q to(6, 1.847, -2.465, -1.602, -0.647, 1.571, 0); // From pose estimation

    createPathRRTConnect(from, to, extend, maxTime);
}

/* Bottel Fram */
void SamplePlugin::GrapTheBottel()
{

    Frame::Ptr pBottelFrame = _wc->findFrame("GraspTargetBottleTop");
    if (NULL == pBottelFrame)
    {
        std::cout << "BottleFrame is defect" << std::endl;
    }
    Frame::Ptr pToolFram = _wc->findFrame("UR-6-85-5-A.TCP");
    if (NULL == pToolFram)
    {
        std::cout << "pToolFram is defect" << std::endl;
    }
    Frame::Ptr pBaseFram = _wc->findFrame("UR-6-85-5-A.Base");
    if (NULL == pBaseFram)
    {
        std::cout << "pBaseFram is defect" << std::endl;
    }

    Frame::Ptr pGraspTCPFram = _wc->findFrame("GraspTCP");
    if (NULL == pGraspTCPFram)
    {
        std::cout << "pGraspTCPFram is defect" << std::endl;
    }

    // std::cout << "Postion of base frame: " << p << std::endl;
    int i = 0;
    while (i < 5)
    {
        MovableFrame::Ptr pMovableBaseFrame = _wc->findFrame<MovableFrame>("Bottle");
        m_RobotState = _wc->getDefaultState();
        Vector3D<> p = (pMovableBaseFrame->getTransform(m_RobotState).P());
        Rotation3D<> Rotaion = pMovableBaseFrame->getTransform(m_RobotState).R();
        std::cout << "postion: " << p <<endl;
        Vector3D<> newPostion(0, i, 0);
        
        pMovableBaseFrame->moveTo(Transform3D<>(p, Rotaion), m_RobotState);

        i += 1;
        m_RobotState = _wc->getDefaultState();

        Transform3D<> T_base_to_object = Kinematics::frameTframe(pBaseFram, pBottelFrame, m_RobotState);
        Transform3D<> T_TCP_to_GraspTCP = Kinematics::frameTframe(pGraspTCPFram, pToolFram, m_RobotState);

        Transform3D<> T_Final = T_base_to_object * T_TCP_to_GraspTCP;

        rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler =
            rw::core::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(m_Robot, m_RobotState));

        std::vector<Q> solutions = closedFormSovler->solve(T_Final, m_RobotState);

        std::cout << "Number of soultions: " << solutions.size() << std::endl;
        for (size_t j = 0; j < solutions.size(); j++)
        {
            m_Robot->setQ(solutions[j], m_RobotState);
        }

        std::vector<Q> CollisionFreesolutions = CheckForColltion(solutions, false);
        std::cout << "Number of free collition soultions: " << CollisionFreesolutions.size() << std::endl;
        std::string sPathName = "CollisonFree";
        sPathName += std::to_string(i);
        SavePathLoaderFile(sPathName, solutions, 0.001);
    }

    // Kinematics::gripFrame(pBottelFrame, pGraspTCPFram, m_RobotState);
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

    MovableFrame::Ptr pMovableBaseFrame = _wc->findFrame<MovableFrame>("UR-6-85-5-A.BaseMov");
    m_RobotState = _wc->getDefaultState();
    Vector3D<> p = (pMovableBaseFrame->getTransform(m_RobotState).P());
    Rotation3D<> Rotaion = pMovableBaseFrame->getTransform(m_RobotState).R();

    std::cout << "Postion of base frame: " << p << std::endl;
    int i = 10;
    while (i < 500)
    {
        Vector3D<> newPostion(0, 0, i);
        std::cout << "New Postion of base frame: " << newPostion << std::endl;
        pMovableBaseFrame->moveTo(Transform3D<>(newPostion, Rotaion), m_RobotState);

        i += 10;
    }
}