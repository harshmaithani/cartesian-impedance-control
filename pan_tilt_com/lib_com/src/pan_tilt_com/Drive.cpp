#include "Drive.h"

namespace Pure{
    namespace Drive{


Client::Client(UdpTransport& transport, UInt16 instance) :
    m_transport(transport)
{
    m_instance = instance;

    // We're going to send a GET request to know the number of drives,
    // and have their properties.

    Request request;
    
    request.action = Request::Get;
    request.target = m_instance;

    if(!m_transport.SendRequest(request))
    {
        throw std::runtime_error("Could not GET drive service state");
    }

    if(Response::OK != request.response.result)
    {
        throw std::runtime_error("Request error.");
    }

    // The request completed successfully, now decode the data.

    Int32 index = 0;
    Int32 count = request.response.data.Size() / 30; // 30 is the size for one drive properties.    

    for(Int32 i = 0; i < count; i++)
    {
        // Foreach entry we add an entry in our drives list, and
        // read the drive properties.

        Device device;

        Byte type;
        Byte mode;

        request.response.data 
            >> type
            >> mode 
            >> device.properties.maxPosition
            >> device.properties.minPosition
            >> device.properties.maxSpeed
            >> device.properties.minSpeed
            >> device.properties.maxAcceleration
            >> device.properties.maxTorque
            >> device.properties.minTorque;

        device.properties.type = (Type)type;
        device.properties.defaultMode = (Mode)mode;

        drives.push_back(device);
        
    }

    // To finish, we will subscribe to the notifications,
    // to know the state of the drive.
    
    if(!m_transport.Subscribe(m_instance, 1))
    {
        throw std::runtime_error("Error subscribing to Drive notifications.");
    }

}

bool Client::Receive(UInt32 timeout)
{
    Notification notification;
    
    // Wait for a notification...

    if(!m_transport.ReceiveNotification(m_instance, notification, timeout))
    {        
        return false;
    }

    // ... and dequeue to get the more recent if several are available.
    
    while(m_transport.ReceiveNotification(m_instance, notification, 0)){}


    // Update our drives state.    
    UInt16 target;
    notification.data >> target; 
    
    UInt64 time; 
    notification.data >> time;

    for(Int32 i = 0; i < (Int32)drives.size(); i++)
    {
        Byte mode;
        Byte status;
        
        drives[i].state.time = time;

        notification.data
            >> mode
            >> status 
            >> drives[i].state.target
            >> drives[i].state.position 
            >> drives[i].state.speed 
            >> drives[i].state.torque ;

        drives[i].state.status = (Status)status;
        drives[i].state.mode = (Mode)mode;
    }

    return true;
}

bool Client::Send()
{
    Notification notification;

    // Push the command data for each drive in the notification buffer.
    
    for(Int32 i = 0; i < (Int32)drives.size(); i++)
    {
        Byte enable = drives[i].command.enable ? 1 : 0;
        Byte mode = (Byte)drives[i].command.mode;
        notification.data
            << enable
            << mode
            << drives[i].command.target;            
    }

    notification.target = m_instance;

    // Send it on the network.

    return m_transport.SendNotification(notification);

}


    }
}
