// CANConnection.hpp
// Contains class that handles thread-safe communication with
// a can network
//
// Author: Aiden Carney

#ifndef CANCONNECTION_HPP
#define CANCONNECTION_HPP


#include <linux/can.h>

#include <mutex>
#include <thread>
#include <list>


#define PACKET_LENGTH 8  // number of bytes in each packet. This should always be 8

class CANConnection {
    private:
        bool connOpen;           // flag to know if the connection is currently open
        int sockfd;              // the socket where the CAN network is open
        std::mutex connMutex;   // mutex to make writes to CAN bus thread-safe

        std::list<struct can_frame> frameQueue;  // the data structure that holds all incoming messages
        int messagesToRead = 10;                 // the number of messages to read at one time in the read thread
        bool runReadThread;                      // boolean to determine whether to read messages to the vector
        std::mutex queueMutex;                   // mutex to make reads and writes to message vector thread-safe
        std::thread readThread;                  // the thread that reads from the socket

        // Should be the target of a detached thread. Runs forever continuously
        // reading frames from the CAN bus and storing them to be handled
        // efficiently later
        //
        // Params:
        //    None
        // Return:
        //    None
        void _readThread();


    public:

        // Default ctor for CANConnection
        // Sets the connection open flag to false
        //
        // Params:
        //    None
        // Return:
        //    the new CANConnection instance
        CANConnection();

        // ctor for CANConnection that attempts to open a new connection 
        // to the can bus
        // 
        // Params:
        //    interface_name - string with what the can interface is named ("can0", "vcan0", etc)
        // Return:
        //    the new CANConnection instance
        CANConnection(const char* interface_name);


        // Attempts to open and bind to the socket for the can network
        // 
        // Params:
        //    interface_name - string with what the can interface is named ("can0", "vcan0", etc)
        // Return:
        //    None
        int openConnection(const char* interface_name);


        // Writes a command to the can bus. Generates the frame based
        // on the id and the data. Thread safe
        //
        // Params:
        //    canId  - the 32 bit integer that is formatted for its intended target. Note: the 
        //             first 3 MSB's are not sent and are used for configuration
        //    data   - array of bytes that will be sent in the can frame. Up to documentation of
        //             CAN receiver as to what this should contain
        //    nBytes - the number of bytes to send in the frame. Same as length of data. Must be on
        //             the interval [0, PACKET_LENGTH]
        // Return:
        //    int -   0 if the operation was successful, -1 if connection not open, -2 if some other handled error
        int writeFrame(uint32_t canId, uint8_t data[], int nBytes);


        // Attempts to read the next frame from the CAN network that was stored in the message
        // queue. Returns 0 if data was read
        //
        // Params:
        //    *canId - a pointer to a 32 bit integer where the can frame id will be placed
        //    data   - an array of bytes for the data to be placed if a frame is read
        // Return:
        //    int - 0 if successful, -1 if connection not open, -2 if no data, -3 unknown error
        int readNextFrame(uint32_t *canId, uint8_t data[PACKET_LENGTH]);


        // Looks through the message queue looking for a specific can id format. Each
        // canId will be masked and checked to be equal to the specifier. If this
        // check returns true, this frame will be placed in the other parameters
        //
        // Params:
        //    bitmask   - the bitmask to use on the can frame to check for what it contains efficiently
        //    specifier - what the canId needs to look like after it has been masked
        // Params:
        //    *canId - a pointer to a 32 bit integer where the can frame id will be placed
        //    data   - an array of bytes for the data to be placed if a frame is read
        // Return:
        //    int - 0 if successful, -1 if connection not open, -2 if no data, -3 unknown error
        int readNextFrameIf(uint32_t bitmask, uint32_t specifier, uint32_t *canId, uint8_t data[PACKET_LENGTH]);


        // Stops the read thread by clearing the flag
        // 
        // Params:
        //    None
        // Return:
        //    None
        void stopReading() {runReadThread = false;}


        // Starts the read thread by setting the flag
        // 
        // Params:
        //    None
        // Return:
        //    None
        void startReading() {runReadThread = true;}


        // Sets a new number of messages to read on each iteration of
        // the read thread
        // 
        // Params:
        //    newN - the new number of messages to read at a time
        // Return:
        //    None
        void setMessagesToRead(int newN) {messagesToRead = newN;}


};

#endif
