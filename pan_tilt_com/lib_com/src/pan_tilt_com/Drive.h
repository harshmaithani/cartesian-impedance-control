#ifndef PURE_DRIVE_H
#define PURE_DRIVE_H

#include "UdpTransport.h"

namespace Pure{
    namespace Drive{

/// Enumerates the command modes of a drive.
enum Mode
{            
    Mode_Position = 0,
    Mode_Velocity = 1,
    Mode_Torque = 2
};

/// Enumerates the types of drive.
enum Type
{
    Type_Linear = 0,
    Type_Angular = 1
};

/// Constains the properties of a drive.
struct Properties
{   
    /// Default operation mode of the drive.
    Mode defaultMode;

    /// Type of the drive. The units used in the following fields depend of its value.
    Type type;
    
    /// Maximum position, in radians or meters.
    float maxPosition;

    /// Minimum position, in radians or meters.
    float minPosition;

    /// Maximum speed, in radians/second or meters/second. 
    float maxSpeed;
    
    /// Minimum speed, in radians/second or meters/second.
    float minSpeed;

    /// Maximum acceleration, in radians/second� or meters/second�
    float maxAcceleration;

    /// Maximum torque/force, in Newton.meters or Newton.
    float maxTorque;
    
    /// Minimum torque/force, in Newton.meters or Newton.
    float minTorque;
};

/// Enumerates the status of a drive.
enum Status
{
    Status_Enabled,
    Status_Disabled,
    Status_Error
};

/// Contains the state of a drive.
struct State
{
    /// Constructor.
    State()
    {        
        time = 0;
        target = 0;
        position = 0;
        speed = 0;
        torque = 0;
        status = Status_Disabled;
        mode = Mode_Velocity;
    }
    
    /// Time stamp of the structure.
    unsigned long long time;

    /// Current target applied to the motor control
    float target;

    /// Current position, in radians or meters.
    float position;

    /// Current speed, in radians/second or meters/second. 
    float speed;   

    /// Current torque, in Newton.meter or Newton. 
    float torque;
    
    /// Current operating mode.
    Mode mode;

    /// Current drive status.
    Status status;    
};

/// Contains the command values.
struct Command
{    
    /// Constructor.
    Command()
    {
        mode = Mode_Velocity;
        enable = false;
        target = 0;
    }  

    /// Desired mode of operation.
    Mode mode;

    /// Sets the drive to enable or disable state. If disabled, the behaviour is hardware dependent.
    bool enable;

    /// Target for the selected control mode.
    float target;
};
     
struct Device
{
    Properties properties;
    State state;
    Command command;
};

class Client 
{
public:

    Client(UdpTransport& transport, UInt16 instance);

    bool Receive(UInt32 timeout = 100);

    bool Send();

    std::vector<Device> drives;

private:

    UdpTransport& m_transport;
    UInt16 m_instance;
};

    }
}

#endif
