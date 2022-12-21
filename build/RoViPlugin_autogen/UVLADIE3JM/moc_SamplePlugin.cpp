/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../SamplePlugin/src/SamplePlugin.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SamplePlugin_t {
    const uint offsetsAndSize[202];
    char stringdata0[1277];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_SamplePlugin_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_SamplePlugin_t qt_meta_stringdata_SamplePlugin = {
    {
QT_MOC_LITERAL(0, 12), // "SamplePlugin"
QT_MOC_LITERAL(13, 10), // "btnPressed"
QT_MOC_LITERAL(24, 0), // ""
QT_MOC_LITERAL(25, 5), // "timer"
QT_MOC_LITERAL(31, 8), // "getImage"
QT_MOC_LITERAL(40, 11), // "get25DImage"
QT_MOC_LITERAL(52, 20), // "stateChangedListener"
QT_MOC_LITERAL(73, 21), // "rw::kinematics::State"
QT_MOC_LITERAL(95, 5), // "state"
QT_MOC_LITERAL(101, 15), // "checkCollisions"
QT_MOC_LITERAL(117, 11), // "Device::Ptr"
QT_MOC_LITERAL(129, 6), // "device"
QT_MOC_LITERAL(136, 5), // "State"
QT_MOC_LITERAL(142, 17), // "CollisionDetector"
QT_MOC_LITERAL(160, 8), // "detector"
QT_MOC_LITERAL(169, 1), // "Q"
QT_MOC_LITERAL(171, 1), // "q"
QT_MOC_LITERAL(173, 20), // "createPathRRTConnect"
QT_MOC_LITERAL(194, 4), // "from"
QT_MOC_LITERAL(199, 2), // "to"
QT_MOC_LITERAL(202, 6), // "extend"
QT_MOC_LITERAL(209, 7), // "maxTime"
QT_MOC_LITERAL(217, 21), // "printProjectionMatrix"
QT_MOC_LITERAL(239, 11), // "std::string"
QT_MOC_LITERAL(251, 9), // "frameName"
QT_MOC_LITERAL(261, 15), // "GoToHomePostion"
QT_MOC_LITERAL(277, 18), // "SavePathLoaderFile"
QT_MOC_LITERAL(296, 9), // "sFileName"
QT_MOC_LITERAL(306, 14), // "std::vector<Q>"
QT_MOC_LITERAL(321, 10), // "vSolutions"
QT_MOC_LITERAL(332, 14), // "dSimultionTime"
QT_MOC_LITERAL(347, 16), // "CheckForColltion"
QT_MOC_LITERAL(364, 17), // "vPossibleSoultion"
QT_MOC_LITERAL(382, 12), // "bOneSoultion"
QT_MOC_LITERAL(395, 12), // "MoveTheRobot"
QT_MOC_LITERAL(408, 11), // "OpenGripper"
QT_MOC_LITERAL(420, 12), // "CloseGripper"
QT_MOC_LITERAL(433, 12), // "TargetObject"
QT_MOC_LITERAL(446, 13), // "eTargetObject"
QT_MOC_LITERAL(460, 4), // "bTop"
QT_MOC_LITERAL(465, 17), // "getConfigurations"
QT_MOC_LITERAL(483, 8), // "nameGoal"
QT_MOC_LITERAL(492, 7), // "nameTcp"
QT_MOC_LITERAL(500, 29), // "rw::models::SerialDevice::Ptr"
QT_MOC_LITERAL(530, 5), // "robot"
QT_MOC_LITERAL(536, 25), // "rw::models::WorkCell::Ptr"
QT_MOC_LITERAL(562, 2), // "wc"
QT_MOC_LITERAL(565, 6), // "State&"
QT_MOC_LITERAL(572, 17), // "CheckReachability"
QT_MOC_LITERAL(590, 30), // "GetStringGraspPostionAndObject"
QT_MOC_LITERAL(621, 12), // "GraspPostion"
QT_MOC_LITERAL(634, 13), // "eGraspPostion"
QT_MOC_LITERAL(648, 34), // "SetNewMountingPostionAndNewHo..."
QT_MOC_LITERAL(683, 8), // "VisionM4"
QT_MOC_LITERAL(692, 21), // "createLinemodDetector"
QT_MOC_LITERAL(714, 21), // "cv::linemod::Detector"
QT_MOC_LITERAL(736, 7), // "linemod"
QT_MOC_LITERAL(744, 18), // "cv::linemod::Match"
QT_MOC_LITERAL(763, 7), // "cv::Mat"
QT_MOC_LITERAL(771, 10), // "camera_img"
QT_MOC_LITERAL(782, 11), // "object_name"
QT_MOC_LITERAL(794, 8), // "autocrop"
QT_MOC_LITERAL(803, 8), // "cv::Rect"
QT_MOC_LITERAL(812, 8), // "cv::Mat&"
QT_MOC_LITERAL(821, 3), // "src"
QT_MOC_LITERAL(825, 8), // "isBorder"
QT_MOC_LITERAL(834, 4), // "edge"
QT_MOC_LITERAL(839, 9), // "cv::Vec3b"
QT_MOC_LITERAL(849, 5), // "color"
QT_MOC_LITERAL(855, 19), // "ReadHomorgraphyFile"
QT_MOC_LITERAL(875, 16), // "vector<Point2d>&"
QT_MOC_LITERAL(892, 9), // "vMatching"
QT_MOC_LITERAL(902, 6), // "vCordi"
QT_MOC_LITERAL(909, 18), // "GenerateHomoPoints"
QT_MOC_LITERAL(928, 9), // "TakeImage"
QT_MOC_LITERAL(938, 5), // "bLeft"
QT_MOC_LITERAL(944, 32), // "FilterTheObjectFromTheBackground"
QT_MOC_LITERAL(977, 3), // "img"
QT_MOC_LITERAL(981, 8), // "bGenrate"
QT_MOC_LITERAL(990, 24), // "DrawCircleAtEstimatedPOs"
QT_MOC_LITERAL(1015, 1), // "x"
QT_MOC_LITERAL(1017, 1), // "y"
QT_MOC_LITERAL(1019, 27), // "ReadHomogenousTransTemplate"
QT_MOC_LITERAL(1047, 25), // "Eigen::Matrix<double,4,4>"
QT_MOC_LITERAL(1073, 7), // "sFileId"
QT_MOC_LITERAL(1081, 9), // "linInterp"
QT_MOC_LITERAL(1091, 14), // "TimedStatePath"
QT_MOC_LITERAL(1106, 8), // "passing1"
QT_MOC_LITERAL(1115, 8), // "passing2"
QT_MOC_LITERAL(1124, 8), // "passing3"
QT_MOC_LITERAL(1133, 8), // "passing4"
QT_MOC_LITERAL(1142, 8), // "passing5"
QT_MOC_LITERAL(1151, 8), // "duration"
QT_MOC_LITERAL(1160, 15), // "InterpParabolic"
QT_MOC_LITERAL(1176, 26), // "ExecutePointTopointPlanner"
QT_MOC_LITERAL(1203, 16), // "ExecuteParabolic"
QT_MOC_LITERAL(1220, 19), // "WriteTrajectoryFile"
QT_MOC_LITERAL(1240, 9), // "vector<Q>"
QT_MOC_LITERAL(1250, 6), // "qValue"
QT_MOC_LITERAL(1257, 14), // "vector<double>"
QT_MOC_LITERAL(1272, 4) // "time"

    },
    "SamplePlugin\0btnPressed\0\0timer\0getImage\0"
    "get25DImage\0stateChangedListener\0"
    "rw::kinematics::State\0state\0checkCollisions\0"
    "Device::Ptr\0device\0State\0CollisionDetector\0"
    "detector\0Q\0q\0createPathRRTConnect\0"
    "from\0to\0extend\0maxTime\0printProjectionMatrix\0"
    "std::string\0frameName\0GoToHomePostion\0"
    "SavePathLoaderFile\0sFileName\0"
    "std::vector<Q>\0vSolutions\0dSimultionTime\0"
    "CheckForColltion\0vPossibleSoultion\0"
    "bOneSoultion\0MoveTheRobot\0OpenGripper\0"
    "CloseGripper\0TargetObject\0eTargetObject\0"
    "bTop\0getConfigurations\0nameGoal\0nameTcp\0"
    "rw::models::SerialDevice::Ptr\0robot\0"
    "rw::models::WorkCell::Ptr\0wc\0State&\0"
    "CheckReachability\0GetStringGraspPostionAndObject\0"
    "GraspPostion\0eGraspPostion\0"
    "SetNewMountingPostionAndNewHomePos\0"
    "VisionM4\0createLinemodDetector\0"
    "cv::linemod::Detector\0linemod\0"
    "cv::linemod::Match\0cv::Mat\0camera_img\0"
    "object_name\0autocrop\0cv::Rect\0cv::Mat&\0"
    "src\0isBorder\0edge\0cv::Vec3b\0color\0"
    "ReadHomorgraphyFile\0vector<Point2d>&\0"
    "vMatching\0vCordi\0GenerateHomoPoints\0"
    "TakeImage\0bLeft\0FilterTheObjectFromTheBackground\0"
    "img\0bGenrate\0DrawCircleAtEstimatedPOs\0"
    "x\0y\0ReadHomogenousTransTemplate\0"
    "Eigen::Matrix<double,4,4>\0sFileId\0"
    "linInterp\0TimedStatePath\0passing1\0"
    "passing2\0passing3\0passing4\0passing5\0"
    "duration\0InterpParabolic\0"
    "ExecutePointTopointPlanner\0ExecuteParabolic\0"
    "WriteTrajectoryFile\0vector<Q>\0qValue\0"
    "vector<double>\0time"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SamplePlugin[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      38,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,  242,    2, 0x08,    1 /* Private */,
       3,    0,  243,    2, 0x08,    2 /* Private */,
       4,    0,  244,    2, 0x08,    3 /* Private */,
       5,    0,  245,    2, 0x08,    4 /* Private */,
       6,    1,  246,    2, 0x08,    5 /* Private */,
       9,    4,  249,    2, 0x08,    7 /* Private */,
      17,    4,  258,    2, 0x08,   12 /* Private */,
      22,    1,  267,    2, 0x08,   17 /* Private */,
      25,    0,  270,    2, 0x08,   19 /* Private */,
      26,    3,  271,    2, 0x08,   20 /* Private */,
      26,    2,  278,    2, 0x28,   24 /* Private | MethodCloned */,
      31,    2,  283,    2, 0x08,   27 /* Private */,
      31,    1,  288,    2, 0x28,   30 /* Private | MethodCloned */,
      34,    0,  291,    2, 0x08,   32 /* Private */,
      35,    0,  292,    2, 0x08,   33 /* Private */,
      36,    2,  293,    2, 0x08,   34 /* Private */,
      40,    5,  298,    2, 0x08,   37 /* Private */,
      48,    0,  309,    2, 0x08,   43 /* Private */,
      49,    2,  310,    2, 0x08,   44 /* Private */,
      52,    0,  315,    2, 0x08,   47 /* Private */,
      53,    0,  316,    2, 0x08,   48 /* Private */,
      54,    0,  317,    2, 0x08,   49 /* Private */,
      56,    2,  318,    2, 0x08,   50 /* Private */,
      61,    1,  323,    2, 0x08,   53 /* Private */,
      65,    2,  326,    2, 0x08,   55 /* Private */,
      69,    2,  331,    2, 0x08,   58 /* Private */,
      73,    0,  336,    2, 0x08,   61 /* Private */,
      74,    1,  337,    2, 0x08,   62 /* Private */,
      74,    0,  340,    2, 0x28,   64 /* Private | MethodCloned */,
      76,    2,  341,    2, 0x08,   65 /* Private */,
      76,    1,  346,    2, 0x28,   68 /* Private | MethodCloned */,
      79,    3,  349,    2, 0x08,   70 /* Private */,
      82,    1,  356,    2, 0x08,   74 /* Private */,
      85,   10,  359,    2, 0x08,   76 /* Private */,
      93,   10,  380,    2, 0x08,   87 /* Private */,
      94,    0,  401,    2, 0x08,   98 /* Private */,
      95,    0,  402,    2, 0x08,   99 /* Private */,
      96,    3,  403,    2, 0x08,  100 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Bool, 0x80000000 | 10, 0x80000000 | 12, 0x80000000 | 13, 0x80000000 | 15,   11,    8,   14,   16,
    QMetaType::Void, 0x80000000 | 15, 0x80000000 | 15, QMetaType::Double, QMetaType::Double,   18,   19,   20,   21,
    QMetaType::Void, 0x80000000 | 23,   24,
    QMetaType::Void,
    QMetaType::Int, 0x80000000 | 23, 0x80000000 | 28, QMetaType::Double,   27,   29,   30,
    QMetaType::Int, 0x80000000 | 23, 0x80000000 | 28,   27,   29,
    0x80000000 | 28, 0x80000000 | 28, QMetaType::Bool,   32,   33,
    0x80000000 | 28, 0x80000000 | 28,   32,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 37, QMetaType::Bool,   38,   39,
    0x80000000 | 28, 0x80000000 | 23, 0x80000000 | 23, 0x80000000 | 43, 0x80000000 | 45, 0x80000000 | 47,   41,   42,   44,   46,    8,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 37, 0x80000000 | 50,   38,   51,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 55,
    0x80000000 | 57, 0x80000000 | 58, 0x80000000 | 23,   59,   60,
    0x80000000 | 62, 0x80000000 | 63,   64,
    QMetaType::Bool, 0x80000000 | 63, 0x80000000 | 67,   66,   68,
    QMetaType::Void, 0x80000000 | 70, 0x80000000 | 70,   71,   72,
    QMetaType::Void,
    0x80000000 | 58, QMetaType::Bool,   75,
    0x80000000 | 58,
    0x80000000 | 58, 0x80000000 | 58, QMetaType::Bool,   77,   78,
    0x80000000 | 58, 0x80000000 | 58,   77,
    0x80000000 | 58, 0x80000000 | 58, QMetaType::Double, QMetaType::Double,   77,   80,   81,
    0x80000000 | 83, 0x80000000 | 23,   84,
    0x80000000 | 86, 0x80000000 | 10, 0x80000000 | 12, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, QMetaType::Double,   11,    8,   18,   87,   88,   89,   90,   91,   19,   92,
    0x80000000 | 86, 0x80000000 | 10, 0x80000000 | 12, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 15, QMetaType::Double,   11,    8,   18,   87,   88,   89,   90,   91,   19,   92,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 23, 0x80000000 | 97, 0x80000000 | 99,   27,   98,  100,

       0        // eod
};

void SamplePlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SamplePlugin *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->btnPressed(); break;
        case 1: _t->timer(); break;
        case 2: _t->getImage(); break;
        case 3: _t->get25DImage(); break;
        case 4: _t->stateChangedListener((*reinterpret_cast< std::add_pointer_t<rw::kinematics::State>>(_a[1]))); break;
        case 5: { bool _r = _t->checkCollisions((*reinterpret_cast< std::add_pointer_t<Device::Ptr>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<State>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<CollisionDetector>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 6: _t->createPathRRTConnect((*reinterpret_cast< std::add_pointer_t<Q>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[4]))); break;
        case 7: _t->printProjectionMatrix((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1]))); break;
        case 8: _t->GoToHomePostion(); break;
        case 9: { int _r = _t->SavePathLoaderFile((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::vector<Q>>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 10: { int _r = _t->SavePathLoaderFile((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::vector<Q>>>(_a[2])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 11: { std::vector<Q> _r = _t->CheckForColltion((*reinterpret_cast< std::add_pointer_t<std::vector<Q>>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2])));
            if (_a[0]) *reinterpret_cast< std::vector<Q>*>(_a[0]) = std::move(_r); }  break;
        case 12: { std::vector<Q> _r = _t->CheckForColltion((*reinterpret_cast< std::add_pointer_t<std::vector<Q>>>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<Q>*>(_a[0]) = std::move(_r); }  break;
        case 13: _t->MoveTheRobot(); break;
        case 14: _t->OpenGripper(); break;
        case 15: _t->CloseGripper((*reinterpret_cast< std::add_pointer_t<TargetObject>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2]))); break;
        case 16: { std::vector<Q> _r = _t->getConfigurations((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::string>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<rw::models::SerialDevice::Ptr>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<rw::models::WorkCell::Ptr>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<State&>>(_a[5])));
            if (_a[0]) *reinterpret_cast< std::vector<Q>*>(_a[0]) = std::move(_r); }  break;
        case 17: _t->CheckReachability(); break;
        case 18: _t->GetStringGraspPostionAndObject((*reinterpret_cast< std::add_pointer_t<TargetObject>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<GraspPostion>>(_a[2]))); break;
        case 19: _t->SetNewMountingPostionAndNewHomePos(); break;
        case 20: _t->VisionM4(); break;
        case 21: { cv::linemod::Detector _r = _t->createLinemodDetector();
            if (_a[0]) *reinterpret_cast< cv::linemod::Detector*>(_a[0]) = std::move(_r); }  break;
        case 22: { cv::linemod::Match _r = _t->linemod((*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::string>>(_a[2])));
            if (_a[0]) *reinterpret_cast< cv::linemod::Match*>(_a[0]) = std::move(_r); }  break;
        case 23: { cv::Rect _r = _t->autocrop((*reinterpret_cast< std::add_pointer_t<cv::Mat&>>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Rect*>(_a[0]) = std::move(_r); }  break;
        case 24: { bool _r = _t->isBorder((*reinterpret_cast< std::add_pointer_t<cv::Mat&>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<cv::Vec3b>>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 25: _t->ReadHomorgraphyFile((*reinterpret_cast< std::add_pointer_t<vector<Point2d>&>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<vector<Point2d>&>>(_a[2]))); break;
        case 26: _t->GenerateHomoPoints(); break;
        case 27: { cv::Mat _r = _t->TakeImage((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 28: { cv::Mat _r = _t->TakeImage();
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 29: { cv::Mat _r = _t->FilterTheObjectFromTheBackground((*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 30: { cv::Mat _r = _t->FilterTheObjectFromTheBackground((*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[1])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 31: { cv::Mat _r = _t->DrawCircleAtEstimatedPOs((*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 32: { Eigen::Matrix<double,4,4> _r = _t->ReadHomogenousTransTemplate((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])));
            if (_a[0]) *reinterpret_cast< Eigen::Matrix<double,4,4>*>(_a[0]) = std::move(_r); }  break;
        case 33: { TimedStatePath _r = _t->linInterp((*reinterpret_cast< std::add_pointer_t<Device::Ptr>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<State>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[5])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[6])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[7])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[8])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[9])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[10])));
            if (_a[0]) *reinterpret_cast< TimedStatePath*>(_a[0]) = std::move(_r); }  break;
        case 34: { TimedStatePath _r = _t->InterpParabolic((*reinterpret_cast< std::add_pointer_t<Device::Ptr>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<State>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[5])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[6])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[7])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[8])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[9])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[10])));
            if (_a[0]) *reinterpret_cast< TimedStatePath*>(_a[0]) = std::move(_r); }  break;
        case 35: _t->ExecutePointTopointPlanner(); break;
        case 36: _t->ExecuteParabolic(); break;
        case 37: _t->WriteTrajectoryFile((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<vector<Q>>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<vector<double>>>(_a[3]))); break;
        default: ;
        }
    }
}

const QMetaObject SamplePlugin::staticMetaObject = { {
    QMetaObject::SuperData::link<rws::RobWorkStudioPlugin::staticMetaObject>(),
    qt_meta_stringdata_SamplePlugin.offsetsAndSize,
    qt_meta_data_SamplePlugin,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_SamplePlugin_t
, QtPrivate::TypeAndForceComplete<SamplePlugin, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const rw::kinematics::State &, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<Device::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<const State &, std::false_type>, QtPrivate::TypeAndForceComplete<const CollisionDetector &, std::false_type>, QtPrivate::TypeAndForceComplete<const Q &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<TargetObject, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<const std::string, std::false_type>, QtPrivate::TypeAndForceComplete<const std::string, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::SerialDevice::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::WorkCell::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<State &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<TargetObject, std::false_type>, QtPrivate::TypeAndForceComplete<GraspPostion, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<cv::linemod::Detector, std::false_type>, QtPrivate::TypeAndForceComplete<cv::linemod::Match, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<const std::string, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Rect, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat &, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat &, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Vec3b, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<vector<Point2d> &, std::false_type>, QtPrivate::TypeAndForceComplete<vector<Point2d> &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<Eigen::Matrix<double,4,4>, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<TimedStatePath, std::false_type>, QtPrivate::TypeAndForceComplete<Device::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<State, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<TimedStatePath, std::false_type>, QtPrivate::TypeAndForceComplete<Device::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<State, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<vector<double>, std::false_type>


>,
    nullptr
} };


const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(this);
    return rws::RobWorkStudioPlugin::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rws::RobWorkStudioPlugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 38)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 38;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 38)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 38;
    }
    return _id;
}

QT_PLUGIN_METADATA_SECTION
static constexpr unsigned char qt_pluginMetaData_SamplePlugin[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', '!',
    // metadata version, Qt version, architectural requirements
    0, QT_VERSION_MAJOR, QT_VERSION_MINOR, qPluginArchRequirements(),
    0xbf, 
    // "IID"
    0x02,  0x78,  0x2a,  'd',  'k',  '.',  's',  'd', 
    'u',  '.',  'm',  'i',  'p',  '.',  'R',  'o', 
    'b',  'w',  'o',  'r',  'k',  '.',  'R',  'o', 
    'b',  'W',  'o',  'r',  'k',  'S',  't',  'u', 
    'd',  'i',  'o',  'P',  'l',  'u',  'g',  'i', 
    'n',  '/',  '0',  '.',  '1', 
    // "className"
    0x03,  0x6c,  'S',  'a',  'm',  'p',  'l',  'e', 
    'P',  'l',  'u',  'g',  'i',  'n', 
    // "MetaData"
    0x04,  0xa3,  0x6c,  'd',  'e',  'p',  'e',  'n', 
    'd',  'e',  'n',  'c',  'i',  'e',  's',  0x80, 
    0x64,  'n',  'a',  'm',  'e',  0x6b,  'p',  'l', 
    'u',  'g',  'i',  'n',  'U',  'I',  'a',  'p', 
    'p',  0x67,  'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x65,  '1',  '.',  '0',  '.',  '0', 
    0xff, 
};
QT_MOC_EXPORT_PLUGIN(SamplePlugin, SamplePlugin)

QT_WARNING_POP
QT_END_MOC_NAMESPACE
