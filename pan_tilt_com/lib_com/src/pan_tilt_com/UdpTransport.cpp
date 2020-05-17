#include "UdpTransport.h"

namespace Pure{

UdpTransport::UdpTransport(
    std::string host, 
    UInt16 remotePort, 
    UInt16 localPort) 
    :
    m_client(localPort, host, remotePort),
    m_pending(NULL),
    m_id(1)
{

    // Create the thread synchronization objects

    pthread_mutex_init(&m_mutex, NULL);
    pthread_cond_init(&m_responseCondition, NULL);
    pthread_mutex_init(&m_responseMutex, NULL);

    // Create and start the incoming messages handling thread

    pthread_create(&m_rxThread, NULL, &ThreadEntry, (void*)this);    
}

void* UdpTransport::ThreadEntry(void* arg)
{       
    UdpTransport* transport = (UdpTransport*)arg;
    
    std::cout << "Starting RxHandler !" << std::endl;    
 
    transport->RxHandler();
    
    return NULL;
}

void UdpTransport::RxHandler()
{
	 //std::cout << "UdpTransport::RxHandler() !" << std::endl;    
	 
    // What we do here is block, waiting for an incoming message.

    while(m_client.ReadMessage(m_rxBuffer, MaxSize, m_rxLength))
    {    
		
		//std::cout << "in while UdpTransport::RxHandler()!" << std::endl;
		    
        if(m_rxLength < 1)
        {
            std::cout << "Message is too short" << std::endl;
            continue;
        }
        
        if(m_rxLength > MaxSize)
        {        
            std::cout << "Message is too long" << std::endl;
            continue;
        }
        
        // We have received a message, so determine what it is...
        
        if(0xFF == m_rxBuffer[0])
        {            
            // ... It's a notification.
            
            if(m_rxLength < 3)
            {
                std::cout << "Notification header is too small" << std::endl;
                continue;
            }
            
            ProcessNotification();            
        }
        else
        {
            // ... It's a response to a request.
            
            if(m_rxLength < 5)
            {
                std::cout << "Response header is too small" << std::endl;
                continue;
            }

            ProcessResponse();            
        }
    }
}

void UdpTransport::ProcessNotification()
{
	//std::cout << "UdpTransport::ProcessNotification() !" << std::endl;
	
    UInt16 target = *((UInt16*)&m_rxBuffer[1]);

    // We have to use a locking mechanism, since the m_subscriptions member can be accessed
    // at any time for writing by another thread which would call the Subscribe method.          

    pthread_mutex_lock(&m_mutex);

    // Find a Subcription with the correct instance.

    std::map<UInt16, Subscription*>::iterator iter = m_subscriptions.find(target);

    if(iter != m_subscriptions.end())
    {
        // We have a match, so build a Notification object from the message...

        Notification notification;
        notification.target = target;

        for(int i = 1; i < m_rxLength; i++)
        {
            notification.data << m_rxBuffer[i];
        }
        
        // ... and queue it for the client.

        // We have to synchronize access to the queue, since it can be read at any time
        // by the thread executing the code in ReceiveNotification.
                        
        pthread_mutex_lock(&iter->second->mutex);

        iter->second->notifications.push(notification);                              
        
        pthread_mutex_unlock(&iter->second->mutex);

        // Signal that we have something in the queue.                
        
        pthread_cond_signal(&iter->second->receivedEvent);
    }   
    
    pthread_mutex_unlock(&m_mutex);
    
}

void UdpTransport::ProcessResponse()
{
	
	//std::cout << "UdpTransport::ProcessResponse() !" << std::endl;
	
    // Extract header information 
    
    char id = m_rxBuffer[0];
    char action = m_rxBuffer[1];
    UInt16 target = *((UInt16*)&m_rxBuffer[2]);
    char result = m_rxBuffer[4];

    // We have to lock access to the state, since we have no guarantee that
    // someone is not currently calling the SendRequest method in another thread.
   
    pthread_mutex_lock(&m_mutex);

    // Check if we have a pending request ...

    if(m_pending != NULL)
    {
        // ... If we have one, check that the header information match the request ...

        if((id == m_pending->id)
            && (action == m_pending->action)
            && (target == m_pending->target))
        {
            // ... if that's the case, fill the request.Response field ...

            m_pending->response.result = result;

            m_pending->response.data.Clear();
            
            for(int i = 5; i < m_rxLength; i++)
            {
                // Push all message bytes in the response data buffer.
                
                m_pending->response.data << m_rxBuffer[i];
            }                    

            m_pending = NULL;

            // ... and signal that we've received it.
            
            pthread_cond_signal(&m_responseCondition);
        }
    }
    
    pthread_mutex_unlock(&m_mutex);    
}

UdpTransport::~UdpTransport()
{    
    
}

bool UdpTransport::SendRequest(Request& request, UInt32 timeout)
{
	
	//std::cout << "UdpTransport::SendRequest() !" << std::endl;
	
    // In this method, we build and send a request packet, and wait for the RX
    // thread to signal a response.


    // We increment the Id field at each request, avoiding 0x00 and 0xFF
    // which are reserved. This helps in checking that a response matches a request.
    
    request.id = ((m_id++) % 253) + 1;
    m_txBuffer[0] = request.id;
    m_txBuffer[1] = request.action;

    *((UInt16*)&m_txBuffer[2]) = request.target;

    for(int i = 0; i < (int)request.data.Size(); i++)
    {
        m_txBuffer[4 + i] = request.data[i];
    }

    // We have to lock 
    
    pthread_mutex_lock(&m_mutex);
    m_pending = &request;
    pthread_mutex_unlock(&m_mutex);
    
    // Send the request on the network ...
    if(!m_client.WriteMessage(m_txBuffer, 4 + request.data.Size()))
    {
        std::cout << "Could not write message..." << std::endl; 
        return false;
    }

    // ... and wait for the RX thread to signal the response, or the timeout to occur.        
    
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);    
    ts.tv_sec += 1;
    //ts.tv_nsec += timeout * 1000000;
    pthread_mutex_lock(&m_responseMutex);
    int result = pthread_cond_timedwait(&m_responseCondition, &m_responseMutex, &ts);
    pthread_mutex_unlock(&m_responseMutex);
    
    // Don't forget to reset our m_pending request field.
    // We have no right to keep a reference to the "request" parameter,
    // since we have no idea about its lifetime...
    
    pthread_mutex_lock(&m_mutex);
    m_pending = NULL;
    pthread_mutex_unlock(&m_mutex);
    
    //std::cout << "End of UdpTransport::SendRequest() !" << ", result = " << result << std::endl;

    return result == 0;
}

bool UdpTransport::SendNotification(Notification& notification)
{
	//std::cout << "UdpTransport::SendNotification !" << std::endl; 
	
    // Build a notification message and send it on the network.

    m_txBuffer[0] = 0xFF;

    *((UInt16*)&m_txBuffer[1]) = notification.target;

    for(int i = 0; i < notification.data.Size(); i++)
    {
        m_txBuffer[3 + i] = notification.data[i];
    }

    return m_client.WriteMessage(m_txBuffer, 3 + notification.data.Size());
}

bool UdpTransport::Subscribe(UInt16 target, Byte mode, UInt32 timeout)
{
    // In this method, we execute an INSERT request on the Notification Manager service,
    // and keep a record of this in m_subscriptions.

    Request request;
    
    request.action = Request::Insert;
    request.target = 1; // Notification Manager is always instance 1.
    
    request.data << target;    
    request.data << mode;
    
    if(!SendRequest(request, timeout))
    {
        return false;
    }

    Subscription* subscription = new Subscription();

    // Add an entry to our subscriptions map.
    // We have to synchronize the access with the RX thread.
   
    pthread_mutex_lock(&m_mutex);
    
    m_subscriptions[target] = subscription;
    
    pthread_mutex_unlock(&m_mutex);  

    return true;

}

bool UdpTransport::ReceiveNotification(UInt16 target, Notification& notification, UInt32 timeout)
{
    // First find the corresponding subscription.   
    
    pthread_mutex_lock(&m_mutex);
    
    std::map<UInt16, Subscription*>::iterator iter = m_subscriptions.find(target);

    Subscription* subscription = NULL;

    if(iter != m_subscriptions.end())
    {
        subscription = iter->second;
    }
    
    pthread_mutex_unlock(&m_mutex);

    if(subscription == NULL)
    {
        return false;
    }

    // Check if there is already a notification in the queue.
    bool available = false;
                
    pthread_mutex_lock(&subscription->mutex);
    
    if(subscription->notifications.size() > 0)
    {
        available = true;
        notification = subscription->notifications.front();
        subscription->notifications.pop();
    }
        
    pthread_mutex_unlock(&subscription->mutex);
      
    if(available)
    {
        return true;
    }

    // If no notification is available, wait to receive one up to timeout.

    if(timeout == 0)
    {
        // No need to make an expensive call.
        return false;
    }
            
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);    
    ts.tv_sec += 1;
    //ts.tv_nsec += timeout * 1000000;
    pthread_mutex_lock(&subscription->receivedMutex);
    pthread_cond_timedwait(&subscription->receivedEvent, &subscription->receivedMutex, &ts);
    pthread_mutex_unlock(&subscription->receivedMutex);
                
    bool result;

    pthread_mutex_lock(&subscription->mutex);

    if(subscription->notifications.size() > 0)
    {
        result = true;

        notification = subscription->notifications.front();
        subscription->notifications.pop();
        
    }    
    else
    {
        result = false;
    }
    
    pthread_mutex_unlock(&subscription->mutex);
    
    return result;
}

UdpClient::UdpClient(UInt16 localPort, std::string remoteHost, UInt16 remotePort)
{
	//std::cout << "UdpClient(UInt16 localPort, std::string remoteHost, UInt16 remotePort) !" << std::endl; 
	//std::cout << "localPort = " << localPort << ", remoteHost = " << remoteHost << ", remotePort = " << remotePort << std::endl;
	
    int error;	

    m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	
	if(m_socket == 0)
	{
		throw std::runtime_error("Could not create socket.");
	}

	sockaddr_in socketAddress;	
	socketAddress.sin_family = AF_INET;
	socketAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    socketAddress.sin_port = htons(localPort);


	error = bind(m_socket, (sockaddr*)&socketAddress, sizeof(socketAddress));
	
	if(error != 0)
	{
		throw std::runtime_error("Could not bind socket.");
	}    	

    m_remoteAddress.sin_family = AF_INET;
    m_remoteAddress.sin_addr.s_addr = inet_addr(remoteHost.c_str());
    m_remoteAddress.sin_port = htons(remotePort);

    m_readAddress.sin_family = AF_INET;
    m_readAddress.sin_addr.s_addr = inet_addr(remoteHost.c_str());
    m_readAddress.sin_port = htons(remotePort);    

}

bool UdpClient::ReadMessage(Byte* buffer, Int32 maxlength, Int32& length)
{
	//std::cout << "UdpClient::ReadMessage(Byte* buffer, Int32 maxlength, Int32& length) !" << std::endl;
	
    socklen_t fromLen = sizeof(m_readAddress);
    int ret = recvfrom(m_socket, buffer, maxlength, 0, (sockaddr*)(&m_readAddress), &fromLen);    
    
    //std::cout << "UdpClient::ReadMessage return length = " << ret << std::endl;
    
    length = ret;
    
    return (length > 0);
}

bool UdpClient::WriteMessage(const Byte* buffer, Int32 length)
{    
	//std::cout << "UdpClient::WriteMessage(const Byte* buffer, Int32 length) !" << std::endl;
	    
    ssize_t s = sendto(m_socket, buffer, length, 0, (sockaddr*)(&m_remoteAddress), sizeof(m_remoteAddress));
 
    return (s == length);
}

}
