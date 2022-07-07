
#include <iostream>

#include "TCPServer.h"
#include "TCPClient.h"
#include "MoCap_Data.h"

//// Here is where the server works
void Server_Work(mocap_netop::CMoCapTCPServer<Data_MoCap_Send,Data_MoCap_Recv> &server)
{
    // Simulate the running of the server
    // We read 100 frames of poses (each frame has several poses) from a file and send them one by one to the clients
    FILE *fp = fopen("skeletons.txt", "r");
    
    for(unsigned i = 0; i < 100; i ++){ // for each frame
        Sleep(20);
        std::cout<<"i:"<<i<<"\n";

        if(!server.IsWorking()){
            std::cout << "Server is not working!\n";
            break;
        }
        
        std::shared_ptr<Data_MoCap_Send> dataEntity = std::make_shared< Data_MoCap_Send >(); // memory for the data to be sent
        
        // To fill the data
        Data_MoCap_Send &dataFrame = *dataEntity;
        unsigned nPose, nJoint;
        char line[128];
        
        dataFrame.timestamp = i;
        
        // number of poses and joints
        fscanf(fp, "%d%d", &nPose, &nJoint);
        fgets(line, 128, fp); // eat out the line
        
        // each pose
        dataFrame.poses.resize(nPose);
        for(unsigned j = 0; j < nPose; j ++){
            Data_MoCap_Send::Pose &pose = dataFrame.poses[j];
            pose.ID = j+1; //fixed value for easy testing
            for(unsigned k = 0; k < JOINT_NUMBER; k ++){
                Data_MoCap_Send::Joint &joint = pose.joints[k];
                fscanf(fp, "%f%f%f", &joint.x, &joint.y, &joint.z);
            }
            
            // eat out the line: one pose one line
            fgets(line, 128, fp);
        }

        //get the action data from the RecvQueue and merge into the dataFrame
        std::shared_ptr<Data_MoCap_Recv> dataEntityRecv = server.GetSeverDataRepos().PopData_RecvQueue();
        Data_MoCap_Recv &dataFrameRecv = *dataEntityRecv;
        if(dataEntityRecv==nullptr){
            dataFrame.actions.resize(0);
        }
        else{
            dataFrame.actions.resize(dataFrameRecv.actions.size());
            for(unsigned j=0; j<dataFrameRecv.actions.size(); j++){
                dataFrame.actions[j].poseID = dataFrameRecv.actions[j].poseID;
                dataFrame.actions[j].action = dataFrameRecv.actions[j].action;
            }
        }
        
        // Push the frame into the server's repo and
        // it will be automatically sent by the server to all the clients
        server.GetSeverDataRepos().PushData_SendQueue( dataEntity );
        if(i==99){
            i-=100;
        }
    }
    
    fclose(fp);
}

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
            
            if(nIterate%2==0){
                // Send an action to server
                std::shared_ptr<Data_MoCap_Recv> dataEntity = std::make_shared< Data_MoCap_Recv >(); // memory for the data to be sent
                        
                // To fill the data
                Data_MoCap_Recv &dataFrame = *dataEntity;
                Data_MoCap_Recv::PoseAction poseAction;
                poseAction.poseID = 1;
                poseAction.action = rand()%100;
                dataFrame.actions.emplace_back( poseAction );

                Data_MoCap_Recv::PoseAction poseAction1;
                poseAction.poseID = 2;
                poseAction.action = rand()%100;
                dataFrame.actions.emplace_back( poseAction1 );

                Data_MoCap_Recv::PoseAction poseAction2;
                poseAction.poseID = 3;
                poseAction.action = rand()%100;
                dataFrame.actions.emplace_back( poseAction2 );

                Data_MoCap_Recv::PoseAction poseAction3;
                poseAction.poseID = 4;
                poseAction.action = rand()%100;
                dataFrame.actions.emplace_back( poseAction3 );
                
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

    setbuf(stdout,NULL);
    std::cout << "Begin.....!\n";
    
//    // In our implementation, we assume that a frame data can be contained in a packet
//    unsigned nPoseSize = sizeof (Data_MoCap::Joint) * JOINT_NUMBER;
//    unsigned nMaxPose = 10; // maximum of 10 poses is allowed in a frame data
//    unsigned maxPacketSize;
    
//    maxPacketSize = nPoseSize * nMaxPose;
    
//    // Simulation of server
    mocap_netop::CMoCapTCPServer<Data_MoCap_Send, Data_MoCap_Recv> server("127.0.0.1:5003", 10000, 5); // max 5 client connections
    auto waitfor = std::chrono::milliseconds(2000) + std::chrono::high_resolution_clock::now(); // wait for 20s
    std::cout << "waiting to start the server.....\n";
    while(!server.Start(sendmsg_callback_mocap_server, recvmsg_callback_mocap_server)) // Note that it should wait for a few seconds if the server restarts on a same port
    {
        if(std::chrono::high_resolution_clock::now() > waitfor){
            std::cout << "Fail to open the server!\n";
            break;
        }
//        sleep(2);
    }
    // Simulation of client
    mocap_netop::CMoCapTCPClient<Data_MoCap_Recv, Data_MoCap_Send> client("127.0.0.1:5003", 10000);
    client.Connect(sendmsg_callback_mocap_client_actionRecog, recvmsg_callback_mocap_client_actionRecog);
    
//    // Construct messages which will be sent by the server

    std::thread server_thread(Server_Work,std::ref(server));
    //while(true){
    //    Server_Work(server);
    
    // Wait for the server and client to send and receive the constructed messages.
    //sleep(1);
    
    // Check the messages that have been recieved by the clients
    Client_Work(client);
    //}
    
//    // Stop server/client
//    server.Stop();

    client.Disconnect();
    
    std::cout << "Finish......!\n";
    
    return 0;
}


