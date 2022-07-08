
#include <iostream>

#include "TCPServer.h"
#include "TCPClient.h"
#include "MoCap_Data.h"

//// Here is where the server works
//void Server_Work(mocap_netop::CMoCapTCPServer<Data_MoCap> &server)
//{
//    // Simulate the running of the server
//    // We read 100 frames of poses (each frame has several poses) from a file and send them one by one to the clients
//    FILE *fp = fopen("skeletons.txt", "r");
    
//    for(unsigned i = 0; i < 100; i ++){ // for each frame
//        if(!server.IsWorking()){
//            std::cout << "Sever is not working!\n";
//            break;
//        }
        
//        std::shared_ptr<Data_MoCap> dataEntity = std::make_shared< Data_MoCap >(); // memory for the data to be sent
        
//        // To fill the data
//        Data_MoCap &dataFrame = *dataEntity; 
//        unsigned nPose, nJoint;
//        char line[128];
        
//        dataFrame.timestamp = i;
        
//        // number of poses and joints
//        fscanf(fp, "%d%d", &nPose, &nJoint);
//        fgets(line, 128, fp); // eat out the line
        
//        // each pose
//        dataFrame.joints.resize(nPose);
//        for(unsigned j = 0; j < nPose; j ++){
//            std::vector<Data_MoCap::Joint> &poseJoint = dataFrame.joints[j];
//            poseJoint.resize(nJoint);
//            for(unsigned k = 0; k < nJoint; k ++){
//                Data_MoCap::Joint &joint = poseJoint[k];
//                fscanf(fp, "%f%f%f", &joint.x, &joint.y, &joint.z);
//            }
            
//            // eat out the line: one pose one line
//            fgets(line, 128, fp);
//        }
        
//        // Push the frame into the server's repo and 
//        // it will be automatically sent by the server to all the clients
//        server.GetSeverDataRepos().PushData_SendQueue( dataEntity );  
//    }
    
//    fclose(fp);
//}

// Here is where the client works
void Client_Work(mocap_netop::CMoCapTCPClient<Data_MoCap_Recv, Data_MoCap_Send> &client)
{
    // Read out the poses that are received by the clients 
    unsigned nRecvPose = 0;
    FILE *fp = fopen("recv_pose.txt", "w"),
            *fp_action = fopen("recv_action.txt", "w"),
            *fp_action_send = fopen("send_action.txt", "w");
    
    int nIterate = 0;
    
    while(client.IsWorking()){
        // Obtain a pose from its recv repo
        auto dataFrame = client.GetClientDataRepos().PopData_RecvQueue();
    
        if(dataFrame){
            nRecvPose ++;
            
            // Write the data into files
            // Frame data
            fprintf(fp, "Frame: %d\n", dataFrame->timestamp);
            
            for(const auto &pose : dataFrame->poses){
                fprintf(fp, "Pose: %d\n", pose.ID);
                
                for(unsigned k = 0; k < JOINT_NUMBER; k ++){
                    fprintf(fp, "(%.3f, %.3f, %.3f)\n", pose.joints[k].x, 
                                 pose.joints[k].y, pose.joints[k].z);
                }
            }
            
            // Action data
            fprintf(fp_action, "A received msg for actions: %d\n", dataFrame->actions.size()); 
            
            for(const auto &action : dataFrame->actions){
                fprintf(fp_action, "Action: (%d, %d)\n", action.poseID, action.action);
            }
            
            ++nIterate;
            
            if(nIterate%5==0){
                // Send an action to server
                std::shared_ptr<Data_MoCap_Recv> dataEntity = std::make_shared< Data_MoCap_Recv >(); // memory for the data to be sent
                        
                // To fill the data
                Data_MoCap_Recv &dataFrame = *dataEntity;
                Data_MoCap_Recv::PoseAction poseAction;
                poseAction.poseID = nIterate-1;
                poseAction.action = nIterate;
                dataFrame.actions.emplace_back( poseAction );
                
                client.GetClientDataRepos().PushData_SendQueue( dataEntity );
                
                // Action data
                fprintf(fp_action_send, "A received msg for actions: %d\n", dataFrame.actions.size()); 
                
                for(const auto &action : dataFrame.actions){
                    fprintf(fp_action_send, "Action: (%d, %d)\n", action.poseID, action.action);
                }
            }
        }
    }
    
    fclose(fp);
    fclose(fp_action);
    fclose(fp_action_send);
    
    std::cout << "Finish working" << std::endl;
}

int main(int argc, char *argv[])
{    
    std::cout << "Begin.....!\n";
    
//    // In our implementation, we assume that a frame data can be contained in a packet
//    unsigned nPoseSize = sizeof (Data_MoCap::Joint) * JOINT_NUMBER;
//    unsigned nMaxPose = 10; // maximum of 10 poses is allowed in a frame data
//    unsigned maxPacketSize;
    
//    maxPacketSize = nPoseSize * nMaxPose;
    
//    // Simulation of server
//    mocap_netop::CMoCapTCPServer<Data_MoCap> server("127.0.0.1:5003", maxPacketSize, 5); // max 5 client connections
//    auto waitfor = std::chrono::milliseconds(20000) + std::chrono::high_resolution_clock::now(); // wait for 20s
//    std::cout << "waiting to start the server.....\n";
//    while(!server.Start(sendmsg_callback_mocap, recvmsg_callback_mocap)) // Note that it should wait for a few seconds if the server restarts on a same port 
//    {
//        if(std::chrono::high_resolution_clock::now() > waitfor){
//            std::cout << "Fail to open the server!\n";
//            break;
//        }

//        sleep(2);
//    }
    
    // Simulation of client
    mocap_netop::CMoCapTCPClient<Data_MoCap_Recv, Data_MoCap_Send> client("192.168.31.143:5003", 10000);
    client.Connect(sendmsg_callback_mocap_client_actionRecog, recvmsg_callback_mocap_client_actionRecog);
    
//    // Construct messages which will be sent by the server 
//    Server_Work(server);
    
    // Wait for the server and client to send and receive the constructed messages.
    sleep(1);
    
    // Check the messages that have been recieved by the clients
    Client_Work(client); 
    
//    // Stop server/client
//    server.Stop();

    client.Disconnect();
    
    std::cout << "Finish......!\n";
    
    return 0;
}


