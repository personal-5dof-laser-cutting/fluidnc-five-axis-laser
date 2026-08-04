#include "src/Machine/MachineConfig.h"
#include "src/GCode.h"
#include "src/System.h"

// Global variables
Machine::MachineConfig* config   = nullptr;
parser_state_t          gc_state = {};
system_t                sys      = {};

// Firmware function stubs
void     limit_error(size_t, float) {}
void     limit_error() {}
void     ambiguousLimit() {}
void     protocol_disable_steppers() {}
void     set_state(State) {}
bool     state_is(State) { return false; }
float*   get_mpos() { static float m[6] = {}; return m; }
float    vector_distance(float*, float*, size_t) { return 0.0f; }
int32_t  mpos_to_steps(float, size_t) { return 0; }
void     set_motor_steps(size_t, int) {}
void     limits_get_state() {}
float    limitsMaxPosition(size_t) { return 0.0f; }
float    limitsMinPosition(size_t) { return 0.0f; }
bool     mc_move_motors(float*, plan_line_data_t*) { return true; }
bool     atMsgLevel(MsgLevel) { return false; }

// Machine stubs
namespace Machine {
    AxisMask  Axes::homingMask        = 0;
    int       Axes::_numberAxis       = 5;
    Axis*     Axes::_axis[MAX_N_AXIS] = {};
    void      Axes::set_disable(bool) {}
    MotorMask Axes::negLimitMask      = 0;
    MotorMask Axes::posLimitMask      = 0;
    void      Motor::limitOtherAxis(int) {}
    void      Stepping::unlimit(int, int) {}
}

// LogStream
static Channel _dummy_channel("dummy");
LogStream::LogStream(MsgLevel, const char*) : _channel(_dummy_channel) {}
LogStream::~LogStream() {}

// AssertionFailed
std::exception AssertionFailed::create(const char*, const char*, ...) { return std::exception(); }

// Channel vtable stubs
Channel::Channel(const std::string& name, bool) : _name(name) {}
Channel::Channel(const char* name, bool) : _name(name) {}
Channel::Channel(const char* name, int, bool) : _name(name) {}
Error   Channel::pollLine(char*) { return Error::Ok; }
void    Channel::ack(Error) {}
void    Channel::sendLine(MsgLevel, const char*) {}
void    Channel::sendLine(MsgLevel, const std::string*) {}
void    Channel::sendLine(MsgLevel, const std::string&) {}
void    Channel::flushRx() {}
bool    Channel::lineComplete(char*, char) { return false; }
bool    Channel::is_visible(const std::string&, std::string, bool) { return false; }
void    Channel::print_msg(MsgLevel, const char*) {}
void    Channel::autoReport() {}
void    Channel::out(const char*, const char*) {}
void    Channel::out(const std::string&, const char*) {}
void    Channel::out_acked(const std::string&, const char*) {}

gc_modal_t modal_defaults = {};

size_t LogStream::write(uint8_t) { return 0; }
