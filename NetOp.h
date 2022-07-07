/*******************************************************************
* Author	: wwyang
* Date		: 2021.11.27
* Copyright : Zhejiang Gongshang University
* Head File :
* Version   : 1.0
*********************************************************************/
// .NAME Data_Header/Data_Buffer

// .SECTION Description
// Here provides some data structure for the network operations

// .SECTION See also
// CMoCapTCPClient, CMoCapTCPSever


#ifndef _NETOP_H_
#define _NETOP_H_

#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <queue>
#include <iostream>;

namespace mocap_netop {

	// something fundamental is here

    // Data in a packet 
    struct Data_Header{ // Note that the length of the header should be fixed, i.e., without dyanmic memory in it
        // What is the data: 8 bytes for easy memory alignment
		char data_name[8]; // avaialble names: "quit", "mocap"

		// Timestamp for the data in the connection
		uint64_t timestamp;
        
        // Data size
        unsigned nDataSize=0, nMaxDataSize=0;
    };

	struct Data_Buffer{
		Data_Header dataHeader;

		// Pointer to the data entity
		void* pData = 0;
	};
    
    // Repos for the data to be sent or have been received by a server or client
    template<class DataType_Send, class DataType_Recv>
    class Data_Repos{
    public:
        ~Data_Repos(){ DestroyRepos(); }
        
        std::shared_ptr<DataType_Send> PopData_SendQueue()
        {
            std::unique_lock<std::mutex> lock(_forSafeDataOp);
            
            if(_queueDataToSend.empty()){
                return std::shared_ptr<DataType_Send>(); // empty pointer
            }
            else{
                //std::cout<<"SendQueue size:"<<_queueDataToSend.size()<<"\n";
            }
            
            std::shared_ptr<DataType_Send> data = _queueDataToSend.front();
            _queueDataToSend.pop();
            
            return data;
        }
        std::shared_ptr<DataType_Recv> PopData_RecvQueue()
        {
            std::unique_lock<std::mutex> lock(_forSafeDataOp);
            
            if(_queueDataReceived.empty()){
                return std::shared_ptr<DataType_Recv>(); // empty pointer
            }
            else{
            }
            
            std::shared_ptr<DataType_Recv> data = _queueDataReceived.front();
            _queueDataReceived.pop();
            
            return data;
        }
        void PushData_SendQueue( const std::shared_ptr<DataType_Send> &data)
        {
            std::unique_lock<std::mutex> lock(_forSafeDataOp);
            
            _queueDataToSend.push(data);
        }
        void PushData_RecvQueue( const std::shared_ptr<DataType_Recv> &data)
        {
            std::unique_lock<std::mutex> lock(_forSafeDataOp);
            
            _queueDataReceived.push(data);
        }
        
        void DestroyRepos()
        {
            std::unique_lock<std::mutex> lock(_forSafeDataOp);
            
            while(!_queueDataToSend.empty()){
                _queueDataToSend.pop();
            }
            
            while(!_queueDataReceived.empty()){
                _queueDataReceived.pop();
            }
        }
        
    private:
        std::mutex _forSafeDataOp; // manipulate the data in a thread-safe manner
        std::queue< std::shared_ptr< DataType_Send > > _queueDataToSend; // data to be sent to server/clients
        std::queue< std::shared_ptr<DataType_Recv> > _queueDataReceived; // data received from the server/client
    };
}

#endif // !_NETOP_H_

