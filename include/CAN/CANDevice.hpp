// CanDevice.hpp
// Contains a template class for creating and using a device on the can network
//
// Author: Aiden Carney

#ifndef CANDEVICE_HPP
#define CANDEVICE_HPP

#include <stdint.h>

#include "CANConnection.hpp"


class CANDevice {
    protected:
        uint8_t deviceId;            // the device can id 
        CANConnection *conn = NULL;  // the can network connection instance, default to null

    public: 
        // Virtual Destructor.
        virtual ~CANDevice() {};

        // Takes an incoming CAN Frame and responds accordingly. Must be
        // overriden. This method should likely be used in a loop so that
        // all frames can be read without blocking.
        //
        // Params:
        //    canFrameId - the integer containing 29 bits that correspond to the 
        //                 id of the can frame
        //    data       - the data to be parsed
        // Return:
        //    None
        virtual void _parseIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]) = 0;


        // Returns the can id of the device
        //
        // Params:
        //    None
        // Return:
        //    uint8_t - the device id for this instance
        uint8_t getDeviceId() const {return deviceId;}


        // Sets a new can id for the device
        // 
        // Params:
        //    newId - the new id for the device
        // Return:
        //    None
        void setDeviceId(uint8_t newId) {deviceId = newId;}


        // Sets a new connection for the CAN device
        // 
        // Params:
        //    newConn - a reference to the connection object to use
        // Return:
        //    None
        void setConnection(CANConnection& newConn) {conn = &newConn;}
};


#endif
