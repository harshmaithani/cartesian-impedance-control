#ifndef PURE_UDPTRANSPORT_H
#define PURE_UDPTRANSPORT_H

// A bunch of stuff for network access ...

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

// ... C++ Standard Template Library items ...

#include <string>
#include <vector>
#include <queue>
#include <list>
#include <map>
#include <iostream>
#include <stdexcept>

// ... and threading primtives from the OS to finish.

#include <pthread.h>
#include <time.h>



// Types definitions matching the PURE protocol specification:

typedef unsigned char       Byte;
typedef short               Int16;
typedef unsigned short      UInt16;
typedef int                 Int32;
typedef unsigned int        UInt32;
typedef long long           Int64;
typedef unsigned long long  UInt64;
typedef float               Float32;
typedef double              Float64;


namespace Pure{

struct Point2D 
{
    Point2D()
    {
        x = 0;
        y = 0;
    }

    Float64 x;
    Float64 y;
};

struct Pose2D
{
    Pose2D()
    {
        orientation = 0;
    }

    Point2D position;
    Float64 orientation;
};

    
class Buffer
{
public:

    static const Int32 MaxSize = 4096;

    Buffer()
    {
        m_start = 0;
        m_end = 0;
    }

    void Clear()
    {
        m_start = 0;
        m_end = 0;
    }

    Int32 Size()
    {
        return m_end - m_start;
    }

    Byte& operator[](UInt32 i)
    {
        if(i > m_end - m_start - 1)
        {
            throw std::runtime_error("Out of bounds.");
        }

        return m_buffer[m_start + i];
    }

    template<class T>
    Buffer& operator<<(const T& val)
    {
        if(sizeof(T) > MaxSize - m_end)
        {
            throw std::runtime_error("Not enough space.");
        }

        memcpy(&m_buffer[m_end], &val, sizeof(T));

        m_end += sizeof(T);

        return *this;
    }

    template<class T>
    Buffer& operator>>(T& val)
    {

        if(sizeof(T) > m_end - m_start)
        {
            throw std::runtime_error("Not enough data.");
        }

        memcpy(&val, &m_buffer[m_start], sizeof(val));

        m_start += sizeof(T);

        return *this;
    }

private:

    UInt32 m_start;
    UInt32 m_end;
    Byte m_buffer[MaxSize];
};

// Describes a response from a PURE Request.
struct Response
{
    // Result of the request.
    Byte result;

    // Optional data.
    Buffer data;

    // Predefined Result codes.
    static const Byte OK =                  0x00;
    static const Byte ServiceNotFound =     0x01;
    static const Byte ActionNotSupported =  0x02;
    static const Byte UnknownAction =       0x03;
    static const Byte InvalidLength =       0x04;
    static const Byte InvalidData =         0x05;
    static const Byte ServiceSpecificBase = 0x10;
};

// Describes a request to a PURE controller.
struct Request
{    

    // Message id.
    Byte id;

    // Action code.
    Byte action;

    // Target service instance.
    UInt16 target;    

    // Request data.
    Buffer data;
 
    // Action codes defined by the protocol.
    static const Byte Get =     0x00;
    static const Byte Query =   0x01;
    static const Byte Replace = 0x02;
    static const Byte Update =  0x03;
    static const Byte Insert =  0x04;    
    static const Byte Delete =  0x05;    
    static const Byte Submit =  0x06;

    // Response from PURE.
    Response response;
};

// Describes a notification message to or from PURE.
struct Notification
{

    // Target service instance.
    UInt16 target;

    // Notification data.
    Buffer data;
};

// Wraps calls to the winsock API.
// Used by the UdpTransport class.
class UdpClient
{
public:

    UdpClient(UInt16 localPort, std::string remoteHost, UInt16 remotePort);    
    
    bool ReadMessage(Byte* buffer, int maxlength, int& length);
    
    bool WriteMessage(const Byte* buffer, int length);    

private:
    
    int m_socket;    

    sockaddr_in m_remoteAddress;
    sockaddr_in m_readAddress;
};

// Implements the base functionalities of the PURE protocol, 
// e.g. requests, notifications, and subscriptions. 
//
// When instantiated, UdpTransport will create a thread
// that will handle incoming UDP messages from PURE.
// It is implemented in UdpTransport.RxHandler.
//
// The public methods give access to the PURE protocol functionalities.
//
// Typically, a client will first execute a GET request using the
// UdpTransport.SendRequest method, to retreive the properties of the 
// requested service. It will then use Subscribe to enable notification
// messages from PURE.
//
// Once this initialization sequence is complete, a client will typically
// call ReceiveNotification to receive sensor data, perform its processing and 
// send commands using SendNotification.
//
// There is one request which is guaranteed to succeed once you have
// the correct IP address and port number; it's a GET request to service
// instance 0.
//
// See the "Protocol->Example" chapter in the "PURE Communication Manual"
// for more details.
//
class UdpTransport
{
    public:

    // Establishes a connection to a PURE controller.

    // The default arguments can be replaced by the data provided
    // in the robot ID card.
    
    UdpTransport(
        std::string host = "192.168.1.2", 
        UInt16 remotePort = 60000, 
        UInt16 localPort = 60000);

    ~UdpTransport();

    // Transmits a request to a PURE controller.

    bool SendRequest(Request& request, UInt32 timeout = 5000);

    bool SendNotification(Notification& notification);

    bool Subscribe(UInt16 target, Byte mode, UInt32 timeout = 5000); 
    
    bool ReceiveNotification(UInt16 target, Notification& notification, UInt32 timeout = 5000);   
    
private:

    pthread_t m_rxThread;

    // Used for Id field in request messages.
    Byte m_id;

    // Used to store a pending request.
    Request* m_pending;  

    // Used by the RX thread to signal a response.    
    pthread_cond_t m_responseCondition;
    pthread_mutex_t m_responseMutex;

    // Used to synchronize access to the state between
    // the RX thread and the caller thread.   
    pthread_mutex_t m_mutex;

    // Holds information relative to a subscription. 
    struct Subscription
    {       
        Subscription()
        {
            pthread_mutex_init(&mutex, NULL);
            pthread_cond_init(&receivedEvent, NULL);
            pthread_mutex_init(&receivedMutex, NULL);
        }
        
        // Incoming notifications are stored in this queue.
        std::queue<Notification> notifications;

        // Used to synchronize access to the queue.        
        pthread_mutex_t mutex;

        // Used to signal the reception of a notification.        
        pthread_cond_t receivedEvent;
        pthread_mutex_t receivedMutex;
    };

    // Stores the subscriptions.
    std::map<UInt16, Subscription*> m_subscriptions;

    // UDP protocol access.
    UdpClient m_client;

    static const Int32 MaxSize = 4096;

    // Use to store transmit data.
    Byte m_txBuffer[MaxSize];

    // Use to store receive data.
    Byte m_rxBuffer[MaxSize];
    int m_rxLength;

    // Message reception routine.(Executed by the rx thread).
    void RxHandler();
    
    static void* ThreadEntry(void* arg);

    
    void ProcessNotification();
    void ProcessResponse();
     
};

}

#endif

