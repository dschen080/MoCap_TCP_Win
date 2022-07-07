/*******************************************************************
* Author	: wwyang
* Date		: 2021.11.27
* Copyright : Zhejiang Gongshang University
* Head File :
* Version   : 1.0
*********************************************************************/
// .NAME CMoCapTCPSever

// .SECTION Description
// It is a class that implements a server with TCP stream. In this implementation, we assume that a mocap data
// can be sent via a packet. The server will send a data if available to all of its connecting clients but does 
// not receive message from the clients except the "quit" msg.
// Note that a server can only send and receive a certain type of data which is specified through the template param.

// .SECTION See also
// CMoCapTCPClient

#ifndef _TCPSERVER_H_
#define _TCPSERVER_H_

#include <iostream>
#include <winsock2.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>
#include <map>
#include <atomic>
#include <windows.h>
#include <winsock.h>
#include <ws2tcpip.h>
#include <assert.h>

#include "NetOp.h"

#pragma comment(lib,"ws2_32.lib")

namespace mocap_netop {

template<class DataType_Send, class DataType_Recv>
class CMoCapTCPServer {
public:
	CMoCapTCPServer() = delete;
	explicit CMoCapTCPServer( const std::string &ipAddress, unsigned maxDataSize, unsigned maxConnection = 5 );
	CMoCapTCPServer(const CMoCapTCPServer&) = delete;

	CMoCapTCPServer& operator=(const CMoCapTCPServer&) = delete;

	~CMoCapTCPServer() { Stop(); }

	// Description:
	// Start the server to listen to the ports and communicate with the clients
	// The callback functions are used to handle msgs that are received from or sent to the clients   
	bool Start(void (*send_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>& )=0, void (*recv_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>&)=0);
	
	// Description:
	// Stop the server
	void Stop();
    
    bool IsWorking()
    {
        return _bInWork && (_sockfd_server >= 0);
    }
    
    // Description:
    // Get the data repos of the sever
    Data_Repos<DataType_Send, DataType_Recv>& GetSeverDataRepos()
    {
        return _dataReposForServer;
    }

private:
	// Initlaize the server, including creating sockets, the thread for listening, and etc.
	bool InitializeServer();

	// core of the listening thread
	void DoListening();

	// core of the thread of message sending
	void DoSendMessage();

	// core of the thread of message receiving
	void DoReceiveMessage(unsigned iClientThread);
    
    // set the socket as non-blocking
    int set_nonblocking(SOCKET fd)
    {
        unsigned long long ul=1;
        int ret=ioctlsocket(fd, FIONBIO, (unsigned long *)&ul);
        if(ret == SOCKET_ERROR)
            return -1;
        else return 1;
    }

private:
	std::thread _threadListen; // thread for listening to the connection query
	std::thread _threadSendMsg; // thread for sending messages to all clients
	std::vector< std::shared_ptr<std::thread> > _threadClients; // thread pool for client connections: receiving messages from each client

    SOCKET _sockfd_server=-1; // handle to the server's socket
    std::map<unsigned, int> _threadConnections; // the connection sockid associated to each thread; -1 if no connection assocition in the thread 
    std::atomic_uint _nCurConnection; // number of current client connections
    
    std::mutex _mutex_forCriticalOps; // for the thread-safe ops 
    
    std::atomic_bool _bInWork; 
    
private:
	std::string _ipAddress; // address of the server: ip and port
	void (*_pRecv_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>& )=0;  // callback for receiving a message
	void (*_pSend_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>&) = 0;  // callback for sending a message. Note that it is sent to all clients 
	unsigned _maxConnection = 1; // maximum number of the client connections allowed by the server
    unsigned _maxDataSize;
    SOCKET testser = INVALID_SOCKET;
    
    Data_Repos<DataType_Send, DataType_Recv> _dataReposForServer; // repos for the data have been received or to be sent by the server
};

//////////////////////// Implementation of the template class ///////////////////////////////////////
///
///
template<class DataType_Send, class DataType_Recv>
CMoCapTCPServer<DataType_Send, DataType_Recv>::CMoCapTCPServer(const std::string& ipAddress, unsigned maxDataSize, unsigned maxConnection /*= 5*/)
    : _ipAddress(ipAddress), _maxConnection(maxConnection), _maxDataSize(maxDataSize)
{
    _bInWork = false;
}

template<class DataType_Send, class DataType_Recv>
bool CMoCapTCPServer<DataType_Send, DataType_Recv>::Start(void (*send_msg_callback)(Data_Buffer *, Data_Repos<DataType_Send, DataType_Recv>&), void (*recv_msg_callback)(Data_Buffer *, Data_Repos<DataType_Send, DataType_Recv>&))
{
    if(_bInWork)
        return false;
    
    _pSend_msg_callback = send_msg_callback;
    _pRecv_msg_callback = recv_msg_callback;
    
    // Initialize the server
    return InitializeServer();
}

template<class DataType_Send, class DataType_Recv>
void CMoCapTCPServer<DataType_Send, DataType_Recv>::Stop()
{
    if(!_bInWork) return;

    _bInWork = false;


    //std::cout << "1\n";

    // Stop all threads
    if(_threadListen.joinable()){
        // Create a client to connect the server in case that the accept() function gets stuck in.
        int sockfd_tmp;
        sockfd_tmp = socket(AF_INET, SOCK_STREAM, 0);

        struct sockaddr_in serv_addr;
        memset((char *) &serv_addr, 0, sizeof(serv_addr));

        struct hostent *server;

        // decode the ip and port from _ipAddress (with the format such as: 192.0.1.1:20000)
        auto index = _ipAddress.find_last_of(':');
        std::string address = _ipAddress.substr(0, index).c_str();
        int port = std::stoi(_ipAddress.substr(index+1, _ipAddress.length()-index-1 ));

        server = gethostbyname(address.c_str());

        serv_addr.sin_family = AF_INET;
        memcpy((char *)&serv_addr.sin_addr.s_addr, (char *)server->h_addr, server->h_length);

        serv_addr.sin_port = htons( port );

        connect(sockfd_tmp, (struct sockaddr *) &serv_addr, sizeof(serv_addr));

        _threadListen.join();

        shutdown(sockfd_tmp, 2);
        closesocket(sockfd_tmp);
    }

    //std::cout << "2\n";

    if(_threadSendMsg.joinable())
        _threadSendMsg.join();

    //std::cout << "22\n";

    for(unsigned i = 0; i < _threadClients.size(); i ++){
        //std::cout << "thread: " << i << std::endl;
        if(_threadClients[i]->joinable())
            _threadClients[i]->join();
        //std::cout << "end join" << std::endl;
    }

    std::cout << "222\n";

    // Close all connection sockets that are generated by the listening thread
    for(unsigned i = 0; i < _threadConnections.size(); i ++){
        int iConnection = _threadConnections[i];
        if(iConnection >= 0){
            // send a null message to the client to notify it
            Data_Header data;

            strcpy(data.data_name, "quit") ;
            data.nDataSize = 0;

            send(iConnection, (char*)&data, sizeof(data),0);

            shutdown(iConnection, 2);
            closesocket(iConnection);
        }
    }

    //std::cout << "3\n";

    // Close the server socket
    if(_sockfd_server >= 0){
//            int t=1;
//            int a = ioctl(_sockfd_server, I_SETCLTIME, t);

        shutdown(_sockfd_server, 2);
        int ret = closesocket(_sockfd_server);    // wait until the write queue is clear
        //std::cout << "close: " << ret << " " << a << std::endl;
    }
    _sockfd_server = -1;

    //std::cout << "4\n";

    // Clear other resources
    _threadClients.clear();
    _threadConnections.clear();

    std::cout << "Success on stopping server\n";
}

template<class DataType_Send, class DataType_Recv>
bool CMoCapTCPServer<DataType_Send, DataType_Recv>::InitializeServer()
{
    WORD w_req = MAKEWORD(2, 2);//Version number
        WSADATA wsadata;
        int err;
        err = WSAStartup(w_req, &wsadata);
        if (err != 0) {
            std::cout << "ini socket fail s" << std::endl;
        }
        else {
            //std::cout << "ini socket success s" << std::endl;
            Sleep(1);
        }
        //Check the Version number
        if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
            std::cout << "fail s" << std::endl;
            WSACleanup();
        }
        else {
            //std::cout << "right s" << std::endl;
            Sleep(1);
        }

    // 1. Create a socket as a file: tcp stream
    _sockfd_server = socket(AF_INET, SOCK_STREAM, 0);

    if(_sockfd_server < 0){
        std::cout << "Error on opening socket" << std::endl;
        return false;
    }

    // 2. Bind the socket to the server's port
    struct sockaddr_in serv_addr;

    // clear address structure
    memset((char *) &serv_addr, 0, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;

    // decode the ip and port from _ipAddress (with the format such as: 192.0.1.1:20000)
    auto index = _ipAddress.find_last_of(':');
    if(index != _ipAddress.npos){ // found
        std::string address = _ipAddress.substr(0, index).c_str();
        int port = std::stoi(_ipAddress.substr(index+1, _ipAddress.length()-index-1 ));

        serv_addr.sin_addr.s_addr = inet_addr( address.c_str() );
        serv_addr.sin_port = htons(  port );
    }
    else{
        std::cout << "Error in IP address which should be in the format such as (192.0.1.1:20000)" << std::endl;
        return false;
    }

    // This bind() call will bind  the socket to the current IP address on port
    int ret = bind(_sockfd_server, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    if ( ret < 0){
        std::cout << "Error on binding socket" << std::endl;
        wprintf(L"socket function failed with error: %u\n", WSAGetLastError());
        return false;
    }

    _bInWork = true;

    _threadClients.clear();
    _threadConnections.clear();

    _nCurConnection = 0;
    for(unsigned i = 0; i < _maxConnection; i ++){
        _threadConnections.insert( std::pair<unsigned, int>(i, -1) );
    }

    // 3. Create a new thread for lisenting to the port
    _threadListen = std::thread(&CMoCapTCPServer::DoListening, this);

    // 4. Create a new thread for sending messages if available to all connected clients
    _threadSendMsg = std::thread(&CMoCapTCPServer::DoSendMessage, this);

    // 5. Pre-create a thread pool for client connections
    for(unsigned i = 0; i < _maxConnection; i ++){
        _threadClients.push_back( std::make_shared<std::thread>( &CMoCapTCPServer::DoReceiveMessage, this, i) );
    }

    return true;
}

template<class DataType_Send, class DataType_Recv>
void CMoCapTCPServer<DataType_Send, DataType_Recv>::DoListening()
{
    // This listen() call tells the socket to listen to the incoming connections.
    // The listen() function places all incoming connection into a backlog queue
    // until accept() call accepts the connection.
    // Here, we set the maximum size for the backlog queue to 5.
    listen(_sockfd_server, 5);
    
    // Try to accept the incoming connections
    while(_bInWork){
        // The accept() call actually accepts an incoming connection
       
        if(_nCurConnection == _maxConnection) continue; // no more connections are allowed
       
        // This accept() function will write the connecting client's address info 
        // into the the address structure and the size of that structure is clilen.
        // The accept() returns a new socket file descriptor for the accepted connection.
        // So, the original socket file descriptor can continue to be used 
        // for accepting new connections while the new socker file descriptor is used for
        // communicating with the connected client.
        struct sockaddr_in client_addr;
        socklen_t clilen = sizeof(client_addr);
        
        // waiting until for getting a connection
        // It should be in a block mode so that the server can always listen to the port
        int sockfd_client = accept(_sockfd_server, (struct sockaddr *) &client_addr, &clilen);

        if(sockfd_client < 0){
            std::cout << "Error on accepting the connection from " << inet_ntoa(client_addr.sin_addr) 
                      << "port " << ntohs(client_addr.sin_port) << std::endl;
        }
        else{            
            // Associates this new connection to a free thread
            _nCurConnection ++;
            
            std::unique_lock<std::mutex> lock(_mutex_forCriticalOps);

            for(unsigned i = 0; i < _maxConnection; i ++){
                if(_threadConnections[i] == -1){ // it's a free thread: no client socket is associated
                    _threadConnections[i] = sockfd_client;
                    break;
                }
            }

            lock.unlock();
            
            // set the client socket as non-blocking mode
            set_nonblocking(sockfd_client);
        } 
    }
    
}

template<class DataType_Send, class DataType_Recv>
void CMoCapTCPServer<DataType_Send, DataType_Recv>::DoSendMessage()
{
    static void *pDataBuffer = malloc(sizeof(Data_Header)+_maxDataSize); // reference to a memory for putting data's header and its entity together
    
    // Try to get a message from the callback of sending message
    while(_bInWork){
        if(_pSend_msg_callback != 0){
            Data_Buffer pickData;
            pickData.dataHeader.nMaxDataSize = _maxDataSize;
//            struct timeval tp; //get system time
//            gettimeofday(&tp,NULL);
//            pickData.dataHeader.timestamp = tp.tv_sec*1000+tp.tv_usec/1000;

            pickData.pData = (char*)pDataBuffer+sizeof(Data_Header);
            
            _pSend_msg_callback(&pickData, _dataReposForServer); // pick out a message for the server from somewhere
            
            // If a message available, then send it to all the client connections
            if(pickData.dataHeader.nDataSize != 0){ // server has some message
                // 1. put the header and entity of the message into a continuous memory
                unsigned nHeaderSize = sizeof (pickData.dataHeader), nTotSize = nHeaderSize + pickData.dataHeader.nDataSize;
                
                memcpy(pDataBuffer, &(pickData.dataHeader), nHeaderSize);
                //std::cout<<"nDataSize:"<<pickData.dataHeader.nDataSize<<"\n";

                //memcpy((void *)((char*) pDataBuffer + nHeaderSize), pickData.pData, pickData.dataHeader.nDataSize);
                
                // 2. send the message to all clients
                std::unique_lock<std::mutex> lock(_mutex_forCriticalOps);
                
                for(unsigned i = 0; i < _threadConnections.size(); i ++){
                    if(_threadConnections[i] != -1){ // 
                        char checkAlive;
                        int nbyte = recv(_threadConnections[i], &checkAlive, 1, MSG_PEEK); // test if client connect is alive
                        if(nbyte!=0){  //connection alive
                            auto n = send(_threadConnections[i], (char*)pDataBuffer, nTotSize,0);
                            if (n < 0)
                                std::cout << "ERROR on writing to socket: " << i << std::endl;
                        }
                        else{  // connection failed
                            std::cout << "connection lost\n";
                            
                            shutdown(_threadConnections[i], 2);
                            closesocket(_threadConnections[i]);
                            
                            _nCurConnection--;
                            _threadConnections[i] = -1;

                        }
                    }
                }

                lock.unlock();
            }
        }
    }
}

template<class DataType_Send, class DataType_Recv>
void CMoCapTCPServer<DataType_Send, DataType_Recv>::DoReceiveMessage(unsigned iClientThread)
{
    // Try to receive a message from the client connection 
    static void *pDataBuffer = malloc(sizeof(Data_Header)+_maxDataSize);
    
    while(_bInWork){
        std::unique_lock<std::mutex> lock(_mutex_forCriticalOps);
        
        int iConnection = _threadConnections[iClientThread];
        
        lock.unlock();
        
        if(iConnection < 0) continue;
        
        //std::cout << "begin recv" << std::endl;
        
        // First, read its head
        unsigned nHeadSize = sizeof(Data_Header);
        int n = recv(iConnection, (char*)pDataBuffer, nHeadSize,0);

        //std::cout<<"recvsize:"<<n<<"\n";
    
        // If a message available, then send it to the callback of receiving  message
        // do nothing, just handle the disconnect message
        if (n == (int)nHeadSize){
            
            //std::cout << "recv1" << std::endl;
            
            Data_Header *pHeadInfo = static_cast<Data_Header*>(pDataBuffer);
            //std::cout<<"RecvnDataSize:"<<pHeadInfo->nDataSize<<"\n";

            if(strcmp(pHeadInfo->data_name, "quit") == 0){
                // quit the connection and detach it from the thread
                shutdown(iConnection, 2);
                closesocket(iConnection);
                
                std::unique_lock<std::mutex> lock(_mutex_forCriticalOps);
                
                _threadConnections[iClientThread] = -1;
                
                lock.unlock();
                
            }
            else{
                assert(strcmp(pHeadInfo->data_name, "mocap") == 0);
                //std::cout << "recv2" << std::endl;
                
                // Second, read out the data entity
                if(pHeadInfo->nDataSize > 0){
                    int n = recv(iConnection, (char *)pDataBuffer+nHeadSize, pHeadInfo->nDataSize, 0);
    
                    //int move = -10;
                    //memcpy(&move,(char *)pDataBuffer+nHeadSize,sizeof(int));
                    //std::cout<<"actionReceived:"<<move<<"\n";
                    
                    if(n == (int)pHeadInfo->nDataSize){
                        Data_Buffer data;
                        
                        data.dataHeader = *pHeadInfo;
                        data.pData = (char *)pDataBuffer + nHeadSize;
                        
                        if(_pRecv_msg_callback != 0){
                            _pRecv_msg_callback(&data, _dataReposForServer);
                        } 
                    } 
                }
            }
        }
        
        //std::cout << "end recv" << std::endl;
    
    } 
}

} // namespace: mocap_netop


#endif // !_TCPSERVER_H_

