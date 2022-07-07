/*******************************************************************
* Author	: wwyang
* Date		: 2021.11.28
* Copyright : Zhejiang Gongshang University
* Head File :
* Version   : 1.0
*********************************************************************/
// .NAME Data_Repos

// .SECTION Description
// It is a class that implements a data type which will be transmitted between the server and the clients. 
// Meanwhile, the corresponding callback functions are created for the server and clients to tell them how to
// transform between the data type and the format of the network packet.  

// .SECTION See also
// CMoCapTCPServer CMoCapTCPClient

#ifndef MOCAP_DATA_H
#define MOCAP_DATA_H

#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <queue>

#include "NetOp.h"

#define JOINT_NUMBER 17

////////////////////////////////////////////////////////////////
/// The data type of Data_MoCap: represents a frame of 3D poses
///
struct Data_MoCap_Send{
    uint64_t timestamp;
    
    // data of skeletons at one frame
    struct Joint{
        float x, y, z;
    };

    struct Pose{
        unsigned long long ID;
        Joint joints[JOINT_NUMBER];
    };
    
    struct PoseAction{
        unsigned long long poseID;
        int action;
    };
    
    std::vector<Pose> poses; // 3D poses 
    std::vector<PoseAction> actions; // recognized action type of each pose  
};

struct Data_MoCap_Recv{
    struct PoseAction{
        unsigned long long poseID;
        int action;
    };
    
    std::vector<PoseAction> actions; // recognized action type of each pose  
};

// The following callbacks are used for the server and client with the Data_MoCap data to transform one data in their repos into
// the format of packet or vice versa.
// -- for server
void sendmsg_callback_mocap_server(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Send, Data_MoCap_Recv> &dataReposForServer);
void recvmsg_callback_mocap_server(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Send, Data_MoCap_Recv> &dataReposForServer);

// -- for client
void sendmsg_callback_mocap_client_actionRecog(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Recv, Data_MoCap_Send> &dataReposForClient);
void recvmsg_callback_mocap_client_actionRecog(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Recv, Data_MoCap_Send> &dataReposForClient);
void recvmsg_callback_mocap_client_contentRender(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Recv, Data_MoCap_Send> &dataReposForClient);

////////////////////////////////////////////////////////////////
/// Other data types.... 
/// Note that one type should be associated with a sendmsg callback and a recvmsg callback.

#endif // MOCAP_DATA_H
