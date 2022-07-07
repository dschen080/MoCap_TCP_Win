#include "MoCap_Data.h"
#include "NetOp.h"

#include <string.h>
#include <assert.h>
#include <iostream>

// callback for the server
void sendmsg_callback_mocap_server(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Send, Data_MoCap_Recv> &dataReposForServerClient)
{
    // try to grab a mocap data that is to be sent from the server's repos
    std::shared_ptr<Data_MoCap_Send> data = dataReposForServerClient.PopData_SendQueue();
    
    if(data){ // not empty
        Data_MoCap_Send *pData = data.get();
        
        strcpy(pDataBuffer->dataHeader.data_name, "mocap");
        pDataBuffer->dataHeader.timestamp = pData->timestamp;
        
        // Pack the mocap data into several packets
        // In our implementation, we assume that a data can be sent in a packet
        unsigned nDataSize = 0;
        
        // data format: (number of poses: uint, 4 bytes); (pose1, pose2, ...); (number of action: uint, 4 bytes); (action1, action2, ..) 
        // format of pose: (poseID: ulong long, 8 bytes); (joint1, joint2,..,joint17)
        // format of joint: (x: float, 4 bytes), (y: float, 4 bytes), (z: float, 4 bytes)
        // format of action: (poseID: ulong long, 8 bytes); (action type: int, 4 bytes) 
        
        // 1. data of 3D poses
        // (a) number of poses
        unsigned nPose = pData->poses.size();
        memcpy( (char *)pDataBuffer->pData + nDataSize, &nPose, 4);
        nDataSize += 4;
        
        // (b) information of each pose
        for(const auto &pose : pData->poses){ // copy each pose
            // id
            memcpy( (char *)pDataBuffer->pData + nDataSize, &pose.ID, 8);
            nDataSize += 8;
            
            // joints
            unsigned jointSize = sizeof (Data_MoCap_Send::Joint); assert(jointSize == 12);
            unsigned nJointSize = sizeof (pose.joints); assert(nJointSize == jointSize*JOINT_NUMBER);
            memcpy( (char *)pDataBuffer->pData + nDataSize, pose.joints, nJointSize);
            nDataSize += nJointSize;
        }
        
        // 2. data of Pose actions
        // (a) number of actions
        unsigned nAction = pData->actions.size();
        memcpy( (char *)pDataBuffer->pData + nDataSize, &nAction, 4);
        nDataSize += 4;

        // (b) information of each action
        for(auto &action : pData->actions){
            // id
            memcpy( (char *)pDataBuffer->pData + nDataSize, &(action.poseID), 8);
            nDataSize += 8;
            
            // action type
            memcpy((char *)pDataBuffer->pData + nDataSize, &(action.action), 4);
            nDataSize += 4;
        }
        
        assert(nDataSize <= pDataBuffer->dataHeader.nMaxDataSize);
        pDataBuffer->dataHeader.nDataSize = nDataSize;
        std::cout<<"nDataSize:"<<nDataSize<<"\n";
    }
}

void recvmsg_callback_mocap_server(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Send, Data_MoCap_Recv> &dataReposForServerClient)
{
    // try to recieve a mocap data and save it to the client's repos
    assert(pDataBuffer->pData != 0);

    // Create a data from the buffer
    std::shared_ptr<Data_MoCap_Recv> data = std::make_shared<Data_MoCap_Recv>();

    // Read out the mocap data from several packets
    // In our implementation, we assume that a buffer contains a mocap data
    
    // data format: (number of action: int, 4 bytes); (action1, action2, ..)
    // format of action: (poseID: ulong long, 8 bytes); (action type: int, 4 bytes) 
    unsigned nDataSize = pDataBuffer->dataHeader.nDataSize, nCurDataSize = 0;

    // (a) number of actions
    unsigned nAction;
    memcpy(&nAction, (char *)pDataBuffer->pData + nCurDataSize, 4);
    nCurDataSize += 4;
    
    data->actions.resize(nAction);
    
    // (b) each action
    for(auto &action : data->actions){
        // id
        memcpy(&(action.poseID), (char *)pDataBuffer->pData + nCurDataSize, 8);
        nCurDataSize += 8;
        
        // action type
        memcpy(&(action.action), (char *)pDataBuffer->pData + nCurDataSize,  4);
        nCurDataSize += 4;
        
        
        //std::cout<<"action:" << action.poseID << ", " << action.action<< "\n";
    }
    
    assert(nCurDataSize == nDataSize);

    dataReposForServerClient.PushData_RecvQueue( data );
}

// callback for the client 1
void sendmsg_callback_mocap_client_actionRecog(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Recv, Data_MoCap_Send> &dataReposForClient)
{
    // Send the action recognition result to the server
    
    // try to grab a mocap data that is to be sent from the client's repos
    std::shared_ptr<Data_MoCap_Recv> data = dataReposForClient.PopData_SendQueue();
    
    if(data){ // not empty
        Data_MoCap_Recv *pData = data.get();
        
        strcpy(pDataBuffer->dataHeader.data_name, "mocap");
        pDataBuffer->dataHeader.timestamp = 1000; // a random number
        
        // Pack the mocap data into several packets
        // In our implementation, we assume that a data can be sent in a packet
        unsigned nDataSize = 0;

        // data format: (number of action: int, 4 bytes); (action1, action2, ..)
        // format of action: (poseID: ulong long, 8 bytes); (action type: int, 4 bytes) 
        
        // 1. data of Pose actions
        // (a) number of actions
        unsigned nAction = pData->actions.size();
        memcpy( (char *)pDataBuffer->pData + nDataSize, &nAction, 4);
        nDataSize += 4;

        // (b) information of each action
        for(auto &action : pData->actions){
            // id
            memcpy( (char *)pDataBuffer->pData + nDataSize, &(action.poseID), 8);
            nDataSize += 8;
            
            // action type
            memcpy((char *)pDataBuffer->pData + nDataSize, &(action.action), 4);
            nDataSize += 4;
        }
        
        assert(nDataSize <= pDataBuffer->dataHeader.nMaxDataSize);
        pDataBuffer->dataHeader.nDataSize = nDataSize;
    }        
}

void recvmsg_callback_mocap_client_actionRecog(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Recv, Data_MoCap_Send> &dataReposForClient)
{
    // Only receive the mocap data of 3d poses from the server
    
    // try to recieve a mocap data and save it to the client's repos
    assert(pDataBuffer->pData != 0);

    // Create a data from the buffer
    std::shared_ptr<Data_MoCap_Send> data = std::make_shared<Data_MoCap_Send>();

    // Read out the mocap data from several packets
    // In our implementation, we assume that a buffer contains a mocap data
    
    // data format: (number of poses: uint, 4 bytes); (pose1, pose2, ...); (number of action: uint, 4 bytes); (action1, action2, ..) 
    // format of pose: (poseID: ulong long, 8 bytes); (joint1, joint2,..,joint17)
    // format of joint: (x: float, 4 bytes), (y: float, 4 bytes), (z: float, 4 bytes)
    // format of action: (poseID: ulong long, 8 bytes); (action type: int, 4 bytes) 
    unsigned nDataSize = pDataBuffer->dataHeader.nDataSize, nCurDataSize = 0;
    
    data->timestamp = pDataBuffer->dataHeader.timestamp;
    
    // 1. data of 3D poses
    // (a) number of poses
    unsigned nPose;
    memcpy(&nPose,  (char *)pDataBuffer->pData + nCurDataSize, 4);
    nCurDataSize += 4;
    
    data->poses.resize(nPose);
    
    // (b) information of each pose
    for(auto &pose : data->poses){ // copy each pose
        // id
        memcpy(&(pose.ID), (char *)pDataBuffer->pData + nCurDataSize, 8);
        nCurDataSize += 8;
        
        // joints
        unsigned jointSize = sizeof (Data_MoCap_Send::Joint); assert(jointSize == 12);
        unsigned nJointSize = sizeof (pose.joints); assert(nJointSize == jointSize*JOINT_NUMBER);
        memcpy(pose.joints, (char *)pDataBuffer->pData + nCurDataSize, nJointSize);
        nCurDataSize += nJointSize;
    }

    dataReposForClient.PushData_RecvQueue( data );
}

void recvmsg_callback_mocap_client_contentRender(mocap_netop::Data_Buffer* pDataBuffer, mocap_netop::Data_Repos<Data_MoCap_Recv, Data_MoCap_Send> &dataReposForClient)
{
    // Receive all of the mocap data (3d poses + pose action) from the server
    
    // try to recieve a mocap data and save it to the client's repos
    assert(pDataBuffer->pData != 0);

    // Create a data from the buffer
    std::shared_ptr<Data_MoCap_Send> data = std::make_shared<Data_MoCap_Send>();

    // Read out the mocap data from several packets
    // In our implementation, we assume that a buffer contains a mocap data
    
    // data format: (number of poses: uint, 4 bytes); (pose1, pose2, ...); (number of action: uint, 4 bytes); (action1, action2, ..) 
    // format of pose: (poseID: ulong long, 8 bytes); (joint1, joint2,..,joint17)
    // format of joint: (x: float, 4 bytes), (y: float, 4 bytes), (z: float, 4 bytes)
    // format of action: (poseID: ulong long, 8 bytes); (action type: int, 4 bytes) 
    unsigned nDataSize = pDataBuffer->dataHeader.nDataSize, nCurDataSize = 0;
    
    data->timestamp = pDataBuffer->dataHeader.timestamp;
    
    // 1. data of 3D poses
    // (a) number of poses
    unsigned nPose;
    memcpy(&nPose,  (char *)pDataBuffer->pData + nCurDataSize, 4);
    nCurDataSize += 4;
    
    data->poses.resize(nPose);
    
    // (b) information of each pose
    for(auto &pose : data->poses){ // copy each pose
        // id
        memcpy(&(pose.ID), (char *)pDataBuffer->pData + nCurDataSize, 8);
        nCurDataSize += 8;
        
        // joints
        unsigned jointSize = sizeof (Data_MoCap_Send::Joint); assert(jointSize == 12);
        unsigned nJointSize = sizeof (pose.joints); assert(nJointSize == jointSize*JOINT_NUMBER);
        memcpy(pose.joints, (char *)pDataBuffer->pData + nCurDataSize, nJointSize);
        nCurDataSize += nJointSize;
    }
    
    // 2. data of Pose actions
    // (a) number of actions
    unsigned nAction;
    memcpy(&nAction, (char *)pDataBuffer->pData + nCurDataSize, 4);
    nCurDataSize += 4;
    
    data->actions.resize(nAction);
    
    // (b) each action
    for(auto &action : data->actions){
        // id
        memcpy(&(action.poseID), (char *)pDataBuffer->pData + nCurDataSize, 8);
        nCurDataSize += 8;
        
        // action type
        memcpy(&(action.action), (char *)pDataBuffer->pData + nCurDataSize,  4);
        nCurDataSize += 4;
        
        //std::cout<<"action:" << action.poseID << ", " << action.action<< "\n";
    }
    
    assert(nCurDataSize == nDataSize);

    dataReposForClient.PushData_RecvQueue( data );
}
