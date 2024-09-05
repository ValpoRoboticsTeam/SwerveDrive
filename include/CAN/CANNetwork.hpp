// CANNetwork.hpp
// Contains code for maintaining a CAN Network in the frc style where
// a heartbeat signal is sent within a maximum period of 100ms. Different
// can devices are added to this network and data is sent to each of these
// instances. Each CANDevice handles its own data sending and data parsing,
// but this network handles the heartbeat and distribution of frames for
// each device to parse. Only periodic frames are distributed
//
// Author: Aiden Carney

#ifndef CANNETWORK_HPP
#define CANNETWORK_HPP

#include <mutex>
#include <vector>

#include "CANConnection.hpp"
#include "CANDevice.hpp"


class CANNetwork {
    private:
        CANConnection *conn;              // the can connection point
        std::vector<CANDevice*> devices;  // the devices registered on the network
        std::mutex deviceMutex;           // mutex to make reads and writes to devices vector thread-safe

        std::thread t;   // the thread where data is distributed to be parsed
        bool runThread;  // if the mainloop should be running or not
        bool runHearbeat;


        // Contains the loop that runs on a thread and sends heartbeat signals and 
        // distributes data to each can device to be handled
        // 
        // Params:
        //    None
        // Return:
        //    None
        void _mainloop();


    public:
        // ctor for CANNetwork. Requires a connection object because this class is 
        // useless without a valid connection
        //
        // Params:
        //    newConn - a reference to the connection object
        // Return:
        //    The new instance
        CANNetwork(CANConnection& newConn);


        // Sets a new connection for the network
        //
        // Params:
        //    newConn - a reference to the connection object
        // Return:
        //    None
        void setConnection(CANConnection& newConn) {conn = &newConn;}


        // Stops the network thread by clearing the flag
        // 
        // Params:
        //    None
        // Return:
        //    None
        void stopReading() {runThread = false;}


        // Starts the network thread by setting the flag
        // 
        // Params:
        //    None
        // Return:
        //    None
        void startReading() {runThread = true;}


        // Stops the heartbeat from being sent by clearing the flag
        // 
        // Params:
        //    None
        // Return:
        //    None
        void stopHeartbeat() {runHearbeat = false;}

        // Starts the hearbeat being sent by setting the flag
        // 
        // Params:
        //    None
        // Return:
        //    None
        void startHearbeat() {runHearbeat = true;}


        // Adds a new device to the list of registered devices
        // if it is not already there
        // 
        // Params:
        //    newDevice - a reference to a CANDevice object
        // Return:
        //    int - 0 if success, -1 if can id is already present
        int addDevice(CANDevice& newDevice);
};



#endif
