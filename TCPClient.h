/*******************************************************************
* Author	: wwyang
* Date		: 2021.11.28
* Copyright : Zhejiang Gongshang University
* Head File :
* Version   : 1.0
*********************************************************************/
// .NAME CMoCapTCPClient

// .SECTION Description
// It is a class that implements a client with TCP stream. In this implementation, we assume that a packet message contains
// a full mocap data. 
// Note that a client can only send and receive a certain type of data which is specified through the template param.

// .SECTION See also
// CMoCapTCPServer

#ifndef TCPCLIENT_H
#define TCPCLIENT_H

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
class CMoCapTCPClient {
public:
	CMoCapTCPClient() = delete;
	explicit CMoCapTCPClient( const std::string &severAddressPort, unsigned maxDataSize);
	CMoCapTCPClient(const CMoCapTCPClient&) = delete;

	CMoCapTCPClient& operator=(const CMoCapTCPClient&) = delete;

	~CMoCapTCPClient() { Disconnect(); }

	// Description:
	// Connect the server for communication
	// The callback functions are used to handle msgs that are received from or sent to the sever   
	bool Connect(void (*send_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>& )=0, void (*recv_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>&)=0);
	
	// Description:
	// Disconnect from the server
	void Disconnect();
    
    bool IsWorking()
    {
        return _bInWork && (_sockfd_client >= 0);
    }
    
    // Description:
    // Get the data repos of the client
    Data_Repos<DataType_Send, DataType_Recv>& GetClientDataRepos()
    {
        return _dataReposForClient;
    }
    
private:
    // core of the thread of message sending
	void DoSendMessage();

	// core of the thread of message receiving
	void DoReceiveMessage();
    
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
    std::atomic_bool _bInWork;
    
    int _sockfd_client=-1; // handle to the client's socket
    std::thread _threadRecvMsg; // thread for receiving messages from the server
	std::thread _threadSendMsg; // thread for sending messages to the server
    
private:
    std::string _serverIPAddress; // address of the server: ip and port
	void (*_pRecv_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>&)=0;  // callback for receiving a message
	void (*_pSend_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>&) = 0;  // callback for sending a message 
    unsigned _maxDataSize;
    
    Data_Repos<DataType_Send, DataType_Recv> _dataReposForClient; // repos for the data have been received or to be sent by the client
};

//////////////////////// Implementation of the template class ///////////////////////////////////////
///
///
template <class DataType_Send, class DataType_Recv>
CMoCapTCPClient<DataType_Send, DataType_Recv>::CMoCapTCPClient( const std::string &serverAddressPort, unsigned maxDataSize )
    : _serverIPAddress(serverAddressPort), _maxDataSize(maxDataSize)
{
    _bInWork = false;
}

template <class DataType_Send, class DataType_Recv>
bool CMoCapTCPClient<DataType_Send, DataType_Recv>::Connect(void (*send_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>&)/*=0*/, void (*recv_msg_callback)(Data_Buffer*, Data_Repos<DataType_Send, DataType_Recv>& )/*=0*/)
{
    if(_bInWork)
        Disconnect();
    
    _pSend_msg_callback = send_msg_callback;
    _pRecv_msg_callback = recv_msg_callback;


    WORD w_req = MAKEWORD(2, 2);//Version number
        WSADATA wsadata;
        int err;
        err = WSAStartup(w_req, &wsadata);
        if (err != 0) {
            std::cout << "ini socket fail c" << std::endl;
        }
        else {
            //std::cout << "ini socket success c" << std::endl;
            Sleep(1);
        }
        //Check the Version number
        if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
            std::cout << "fail c" << std::endl;
            WSACleanup();
        }
        else {
            //std::cout << "right c" << std::endl;
            Sleep(1);
        }
    
    // Connect to the server
    
    // 1. Create a socket as a file: tcp stream
    _sockfd_client = socket(AF_INET, SOCK_STREAM, 0);
    
    if(_sockfd_client < 0){
        std::cout << "Error on opening socket" << std::endl;
        return false;
    }
    
    // 2. Fill the server's information and connect it
    struct sockaddr_in serv_addr;
    
    // clear address structure
    memset((char *) &serv_addr, 0, sizeof(serv_addr));
    
    serv_addr.sin_family = AF_INET;  
    
    // decode the ip and port from _ipAddress (with the format such as: 192.0.1.1:20000)
    auto index = _serverIPAddress.find_last_of(':');
    if(index != _serverIPAddress.npos){ // found
        std::string address = _serverIPAddress.substr(0, index).c_str();
        int port = std::stoi(_serverIPAddress.substr(index+1, _serverIPAddress.length()-index-1 ));
        
        struct hostent *server;
        server = gethostbyname(address.c_str());
        if (server == NULL) {
            std::cout << "ERROR, no such host\n";
            
            shutdown(_sockfd_client, 2);
            closesocket(_sockfd_client);
            return false;
        }
        
        memcpy((char *)&serv_addr.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
        serv_addr.sin_port = htons(  port ); 
    }
    else{
        std::cout << "Error in IP address which should be in the format such as (192.0.1.1:20000)" << std::endl;
        
        shutdown(_sockfd_client, 2);
        closesocket(_sockfd_client);
        return false;
    }
    
    // Connect the server
    if (connect(_sockfd_client, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0){ 
        std::cout << "ERROR connecting\n";
        
        shutdown(_sockfd_client, 2);
        closesocket(_sockfd_client);
        return false;
    }
    
    // set the client socket as non-blocking mode
    set_nonblocking(_sockfd_client);
    
    _bInWork = true;
    
    // 3. Create a new session for receving message from the server
    _threadRecvMsg = std::thread(&CMoCapTCPClient::DoReceiveMessage, this);
    
    // 4. Create a new session for sending messages if available to the server
    _threadSendMsg = std::thread(&CMoCapTCPClient::DoSendMessage, this);

    return true;  
}

template <class DataType_Send, class DataType_Recv>
void CMoCapTCPClient<DataType_Send, DataType_Recv>::Disconnect()
{
    if(!_bInWork) return;
    
    _bInWork = false;
    
    // Stop all threads
    if(_threadSendMsg.joinable())
        _threadSendMsg.join();
    if(_threadRecvMsg.joinable())
        _threadRecvMsg.join();
    
    // Close the connection socket
    if(_sockfd_client >= 0){
        // First, send a null message to the server to notify it            
        Data_Header data;
        
        strcpy(data.data_name, "quit");
        data.nDataSize = 0;
                   
        send(_sockfd_client, (char*)&data, sizeof(data),0);
        
        shutdown(_sockfd_client, 2);
        closesocket(_sockfd_client);
        
        _sockfd_client = -1;
    }
    
    std::cout << "Success on stopping client\n";
}

template <class DataType_Send, class DataType_Recv>
void CMoCapTCPClient<DataType_Send, DataType_Recv>::DoSendMessage()
{
    static void *pDataBuffer = malloc(sizeof(Data_Header)+_maxDataSize); // reference to a memory for putting data's header and its entity together
    
    // Try to get a message from the callback of sending message
    while(_bInWork && _sockfd_client >= 0){
        if(_pSend_msg_callback != 0){
            Data_Buffer pickData;
            pickData.dataHeader.nMaxDataSize = _maxDataSize;
            pickData.pData = (char*)pDataBuffer+sizeof(Data_Header);
            
            _pSend_msg_callback(&pickData, _dataReposForClient); // pick out a message for the server from somewhere
            
            // If a message available, then send it to the server
            if(pickData.dataHeader.nDataSize != 0){ // it has some message                
                // 1. put the header and entity of the message into a continuous memory
                unsigned nHeaderSize = sizeof (pickData.dataHeader), nTotSize = nHeaderSize + pickData.dataHeader.nDataSize;
                
                memcpy(pDataBuffer, &(pickData.dataHeader), nHeaderSize);
                
                // 2. send the message to the server
                int n = send(_sockfd_client, (char*)pDataBuffer, nTotSize,0);
                if (n < 0) 
                    std::cout << "ERROR on writing to socket\n";
            }
        }
    }
    
    return;
}

template <class DataType_Send, class DataType_Recv>
void CMoCapTCPClient<DataType_Send, DataType_Recv>::DoReceiveMessage()
{
    // Try to receive a message from the client connection
    // Default to receive the skeleton data
    unsigned nHeadSize = sizeof(Data_Header);
    static void *pDataBuffer = malloc(nHeadSize+_maxDataSize);
            
    while(_bInWork  && _sockfd_client >= 0){
        
        // First, read out the header
        int n = recv(_sockfd_client, (char*)pDataBuffer, nHeadSize,0);
        
        if(n == (int)nHeadSize){ // It's a data
            Data_Header *pHeader = static_cast<Data_Header*>(pDataBuffer);
            
            if(strcmp(pHeader->data_name, "quit") == 0){
                // quit the connection
                
                shutdown(_sockfd_client, 2);
                closesocket(_sockfd_client);
                
                _sockfd_client = -1;
                
                //std::cout << "Client: receive server quit command\n";
            }
            else{
                assert(strcmp(pHeader->data_name, "mocap") == 0);
                
                if(pHeader->nDataSize > 0){
                    // Second, read out the data entity
                    int n = recv(_sockfd_client, (char *)pDataBuffer+nHeadSize, pHeader->nDataSize,0);
    
                    if(n == (int)pHeader->nDataSize){
                        Data_Buffer data;
                        
                        data.dataHeader = *pHeader;
                        data.pData = (char *)pDataBuffer + nHeadSize;
    
                        float f;
                        memcpy(&f,data.pData,sizeof(float));
                        
                        if(_pRecv_msg_callback != 0){
                            _pRecv_msg_callback(&data, _dataReposForClient);
                        } 
                        
                        //std::cout << "recv: " << n << std::endl;
                    } 
                }
            }
        }
    }
   
    return;
}

} // namespace: mocap_netop



#endif // TCPCLIENT_H
