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
    const uint offsetsAndSize[106];
    char stringdata0[662];
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
QT_MOC_LITERAL(277, 13), // "GrapTheBottel"
QT_MOC_LITERAL(291, 18), // "SavePathLoaderFile"
QT_MOC_LITERAL(310, 9), // "sFileName"
QT_MOC_LITERAL(320, 14), // "std::vector<Q>"
QT_MOC_LITERAL(335, 10), // "vSolutions"
QT_MOC_LITERAL(346, 14), // "dSimultionTime"
QT_MOC_LITERAL(361, 16), // "CheckForColltion"
QT_MOC_LITERAL(378, 17), // "vPossibleSoultion"
QT_MOC_LITERAL(396, 12), // "bOneSoultion"
QT_MOC_LITERAL(409, 12), // "MoveTheRobot"
QT_MOC_LITERAL(422, 11), // "OpenGripper"
QT_MOC_LITERAL(434, 12), // "CloseGripper"
QT_MOC_LITERAL(447, 12), // "TargetObject"
QT_MOC_LITERAL(460, 13), // "eTargetObject"
QT_MOC_LITERAL(474, 4), // "bTop"
QT_MOC_LITERAL(479, 17), // "getConfigurations"
QT_MOC_LITERAL(497, 8), // "nameGoal"
QT_MOC_LITERAL(506, 7), // "nameTcp"
QT_MOC_LITERAL(514, 29), // "rw::models::SerialDevice::Ptr"
QT_MOC_LITERAL(544, 5), // "robot"
QT_MOC_LITERAL(550, 25), // "rw::models::WorkCell::Ptr"
QT_MOC_LITERAL(576, 2), // "wc"
QT_MOC_LITERAL(579, 6), // "State&"
QT_MOC_LITERAL(586, 17), // "CheckReachability"
QT_MOC_LITERAL(604, 30), // "GetStringGraspPostionAndObject"
QT_MOC_LITERAL(635, 12), // "GraspPostion"
QT_MOC_LITERAL(648, 13) // "eGraspPostion"

    },
    "SamplePlugin\0btnPressed\0\0timer\0getImage\0"
    "get25DImage\0stateChangedListener\0"
    "rw::kinematics::State\0state\0checkCollisions\0"
    "Device::Ptr\0device\0State\0CollisionDetector\0"
    "detector\0Q\0q\0createPathRRTConnect\0"
    "from\0to\0extend\0maxTime\0printProjectionMatrix\0"
    "std::string\0frameName\0GoToHomePostion\0"
    "GrapTheBottel\0SavePathLoaderFile\0"
    "sFileName\0std::vector<Q>\0vSolutions\0"
    "dSimultionTime\0CheckForColltion\0"
    "vPossibleSoultion\0bOneSoultion\0"
    "MoveTheRobot\0OpenGripper\0CloseGripper\0"
    "TargetObject\0eTargetObject\0bTop\0"
    "getConfigurations\0nameGoal\0nameTcp\0"
    "rw::models::SerialDevice::Ptr\0robot\0"
    "rw::models::WorkCell::Ptr\0wc\0State&\0"
    "CheckReachability\0GetStringGraspPostionAndObject\0"
    "GraspPostion\0eGraspPostion"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SamplePlugin[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      20,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,  134,    2, 0x08,    1 /* Private */,
       3,    0,  135,    2, 0x08,    2 /* Private */,
       4,    0,  136,    2, 0x08,    3 /* Private */,
       5,    0,  137,    2, 0x08,    4 /* Private */,
       6,    1,  138,    2, 0x08,    5 /* Private */,
       9,    4,  141,    2, 0x08,    7 /* Private */,
      17,    4,  150,    2, 0x08,   12 /* Private */,
      22,    1,  159,    2, 0x08,   17 /* Private */,
      25,    0,  162,    2, 0x08,   19 /* Private */,
      26,    0,  163,    2, 0x08,   20 /* Private */,
      27,    3,  164,    2, 0x08,   21 /* Private */,
      27,    2,  171,    2, 0x28,   25 /* Private | MethodCloned */,
      32,    2,  176,    2, 0x08,   28 /* Private */,
      32,    1,  181,    2, 0x28,   31 /* Private | MethodCloned */,
      35,    0,  184,    2, 0x08,   33 /* Private */,
      36,    0,  185,    2, 0x08,   34 /* Private */,
      37,    2,  186,    2, 0x08,   35 /* Private */,
      41,    5,  191,    2, 0x08,   38 /* Private */,
      49,    0,  202,    2, 0x08,   44 /* Private */,
      50,    2,  203,    2, 0x08,   45 /* Private */,

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
    QMetaType::Void,
    QMetaType::Int, 0x80000000 | 23, 0x80000000 | 29, QMetaType::Double,   28,   30,   31,
    QMetaType::Int, 0x80000000 | 23, 0x80000000 | 29,   28,   30,
    0x80000000 | 29, 0x80000000 | 29, QMetaType::Bool,   33,   34,
    0x80000000 | 29, 0x80000000 | 29,   33,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 38, QMetaType::Bool,   39,   40,
    0x80000000 | 29, 0x80000000 | 23, 0x80000000 | 23, 0x80000000 | 44, 0x80000000 | 46, 0x80000000 | 48,   42,   43,   45,   47,    8,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 38, 0x80000000 | 51,   39,   52,

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
        case 9: _t->GrapTheBottel(); break;
        case 10: { int _r = _t->SavePathLoaderFile((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::vector<Q>>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 11: { int _r = _t->SavePathLoaderFile((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::vector<Q>>>(_a[2])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 12: { std::vector<Q> _r = _t->CheckForColltion((*reinterpret_cast< std::add_pointer_t<std::vector<Q>>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2])));
            if (_a[0]) *reinterpret_cast< std::vector<Q>*>(_a[0]) = std::move(_r); }  break;
        case 13: { std::vector<Q> _r = _t->CheckForColltion((*reinterpret_cast< std::add_pointer_t<std::vector<Q>>>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<Q>*>(_a[0]) = std::move(_r); }  break;
        case 14: _t->MoveTheRobot(); break;
        case 15: _t->OpenGripper(); break;
        case 16: _t->CloseGripper((*reinterpret_cast< std::add_pointer_t<TargetObject>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2]))); break;
        case 17: { std::vector<Q> _r = _t->getConfigurations((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::string>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<rw::models::SerialDevice::Ptr>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<rw::models::WorkCell::Ptr>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<State&>>(_a[5])));
            if (_a[0]) *reinterpret_cast< std::vector<Q>*>(_a[0]) = std::move(_r); }  break;
        case 18: _t->CheckReachability(); break;
        case 19: _t->GetStringGraspPostionAndObject((*reinterpret_cast< std::add_pointer_t<TargetObject>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<GraspPostion>>(_a[2]))); break;
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
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const rw::kinematics::State &, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<Device::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<const State &, std::false_type>, QtPrivate::TypeAndForceComplete<const CollisionDetector &, std::false_type>, QtPrivate::TypeAndForceComplete<const Q &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<TargetObject, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<Q>, std::false_type>, QtPrivate::TypeAndForceComplete<const std::string, std::false_type>, QtPrivate::TypeAndForceComplete<const std::string, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::SerialDevice::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::WorkCell::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<State &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<TargetObject, std::false_type>, QtPrivate::TypeAndForceComplete<GraspPostion, std::false_type>


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
        if (_id < 20)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 20;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 20)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 20;
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
