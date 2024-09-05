#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>

#include <chrono>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
//#include <iostream>

#include "CAN/CANConnection.hpp"
#include "util/misc.hpp"


// Sets the connection open flag to false
CANConnection::CANConnection() {
    connOpen = false;
    runReadThread = true;

    readThread = std::thread(&CANConnection::_readThread, this);
}


// the connection open flag will be set by the function call
CANConnection::CANConnection(const char* interface_name) {
    connOpen = false;
    openConnection(interface_name);
    runReadThread = true;

    readThread = std::thread(&CANConnection::_readThread, this);
}


// uses an infinite loop that delays for a period after every execution. Attempts to read
// messagesToRead number of frames, if no data is available this thread will sleep.
// Accesses the messages queue with a mutex so that it is thread safe
void CANConnection::_readThread() {
    while(true) {
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // sleep for half a milli second

        if(!runReadThread) continue;  // make sure we should be reading
        if(!connOpen) continue;       // make sure connection is opened properly

        struct can_frame frames[messagesToRead];  // attempt to read data
        int messagesRead = 0;
        while(messagesRead < messagesToRead) {
            int nbytes;
            struct can_frame frame;
            nbytes = read(sockfd, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {  // don't show error on terminal because the no-block flag is set meaning there might not always be data
                break;  // exit for loop
            }

            frames[messagesRead] = frame;
            messagesRead++;
        }

        // write the message to the queue
        const std::lock_guard<std::mutex> lock(queueMutex);
        for(int i=0; i<messagesRead; i++) {
            frameQueue.push_back(frames[i]);
        }


        // lock gets released because it goes out of scope here
    }
}


// Uses the linux built in api for interacting with sockets.
// Will not raise any exceptions. If exceptions occur, the terminal
// will show it, but execution will continue. Opens socket with the
// no-block flag so that any read operations will return immediately
int CANConnection::openConnection(const char* interface_name) {
    if ((sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {  // might be AF_CAN
       perror("Socket Failed");
       connOpen = false;

       return -1;
    }

    struct timeval timeout;      
    timeout.tv_sec = 0;
    timeout.tv_usec = 5000;  // 5ms max delay for messages

    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout) < 0) {
        perror("setsockopt failed\n");
    }

    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof timeout) < 0) {
        perror("setsockopt failed\n");
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_name);

    if(ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        connOpen = false;

        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        connOpen = false;

        return -1;
    }

    connOpen = true;

    return 0;
}


// Aquires the mutex in an exception safe way and writes
// a command to the can bus if it is properly set up
// Make sure "canX" txqueuelen is set to be greater than 10.
// 1000 is a good number. This ensures that no fails to write
// due to lack of buffer space will occur
int CANConnection::writeFrame(uint32_t canId, uint8_t data[], int nBytes) {
    if(!connOpen) return -1;  // make sure connection is opened properly
    if(nBytes < 0 || nBytes > PACKET_LENGTH) return -4;  // only accept valid packet sizes

    struct can_frame frame;
    frame.can_id = canId;
    frame.can_dlc = nBytes;
    for(int i = 0; i < nBytes; i++) {  // copy data to frame
        frame.data[i] = data[i];
    }
    for(int i = nBytes; i < PACKET_LENGTH; i++) {  // fill rest of frame with 0s
        frame.data[i] = 0;
    }

    const std::lock_guard<std::mutex> lock(connMutex);  // grab the mutex and send the data on its way
    if (write(sockfd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("CAN Failed to write: ");
        return -2;
    }

    return 0;
}


// Reads a can frame and stores the data in the supplied parameters
int CANConnection::readNextFrame(uint32_t *canId, uint8_t data[PACKET_LENGTH]) {
    if(!connOpen) return -1;  // make sure connection is opened properly
    if(frameQueue.empty()) return -2;  // no data


    struct can_frame frame;  // get the first element and remove it
    try {
        const std::lock_guard<std::mutex> lock(queueMutex);
        frame = frameQueue.front();
        frameQueue.pop_front();
    } catch(...) {
        perror("Reading from Message Queue Failed");  // exception caught, log it and return
        return -3;
    }

    *canId = frame.can_id;                        // copy over frame data
    for(int i = 0; i < PACKET_LENGTH; i++) {
        data[i] = frame.data[i];
    }

    return 0;  // success
}


int CANConnection::readNextFrameIf(uint32_t bitmask, uint32_t specifier, uint32_t *canId, uint8_t data[PACKET_LENGTH]) {
    if(!connOpen) return -1;  // make sure connection is opened properly
    if(frameQueue.empty()) return -2;  // no data


    struct can_frame frame;  // get the first element and remove it
    try {
        const std::lock_guard<std::mutex> lock(queueMutex);

        std::list<struct can_frame>::iterator it = frameQueue.begin();

        
        while(it != frameQueue.end()) {
            uint32_t maskedId = it->can_id & bitmask;
            if(maskedId == specifier) {
                *canId = it->can_id;                        // copy over frame data
                for(int i = 0; i < PACKET_LENGTH; i++) {
                    data[i] = it->data[i];
                }
                frameQueue.erase(it);

                return 0;  // exit immediately, nothing else to do
            }

            it++;  // move to next element
        }

        
    } catch(...) {
        perror("Reading from Message Queue Failed");  // exception caught, log it and return
        return -3;
    }


    return -2;  // nothing found, would have returned earlier if something was found


}
