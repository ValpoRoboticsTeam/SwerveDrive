#include <chrono>
#include <iostream>
#include <cstring>
#include <cmath>

#include "CAN/can_utils.hpp"
#include "CAN/SparkMaxMC.hpp"


// intializes members to either null or 0
SparkMaxMC::SparkMaxMC() {
    conn = NULL;
    deviceId = 0;

    encoderMode = 0;
    isReversed = false;
    encoderIsReversed = false;
    encoderOffset = 0;
}


// initializes members to supplied values
SparkMaxMC::SparkMaxMC(CANConnection& connection, int canDeviceId) {
    conn = &connection;
    deviceId = canDeviceId;

    encoderMode = 0;
    isReversed = false;
    encoderIsReversed = false;
    encoderOffset = 0;
}


SparkMaxMC::~SparkMaxMC() {
    dutyCycleSet(0);
}


// Creates the struct of parameters and then gets the 
// corresponding integer using a method from can_utils
uint32_t SparkMaxMC::getCanFrameId(int apiClass, int apiIndex) {
    can_id_params params = {2, 5, apiClass, apiIndex, deviceId};
    return genCanFrameID(&params);
}


// use can utils to generate the setpoint frame based on the documentation
void SparkMaxMC::getSetpointFrame(uint8_t bytes[8], float setpoint, int16_t arbFF, uint8_t pidSlot) {
    memset(bytes, 0, 8 * sizeof(uint8_t));       // clear data

    floatToBytes(setpoint, bytes, 4);            // fill first 4 bytes with the setpoint
    
    uint8_t arbFFBytes[2];
    memset(arbFFBytes, 0, 2 * sizeof(uint8_t));
    int64ToBytes(arbFF, arbFFBytes, 2);
    bytes[4] = arbFFBytes[0];                    // next 2 bytes are arbFF
    bytes[5] = arbFFBytes[1];

    bytes[6] = pidSlot;                          // byte 7 is for pid slot and arb units, but only changing pid slot

    // ignore last byte, it can stay 0
}


void SparkMaxMC::debugIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]) {
    can_id_params params;
    decodeCanFrameID(canFrameId, &params);

    printf("Decoding of message %x:\n", canFrameId);
    printf("    Device Type:   %d\n", params.deviceType);
    printf("    Manufacturer:  %d\n", params.manufacturer);
    printf("    API Class:     %d\n", params.apiClass);
    printf("    API Index:     %d\n", params.apiIndex);
    printf("    Device Number: %d\n", params.deviceNumber);
    printf("    ");
    for(int i = 0; i < PACKET_LENGTH; i++) {
        printf("%d ", data[i]);
    }
    printf("\n");
}


// Takes an incoming CAN Frame and responds accordingly.
void SparkMaxMC::_parseIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]) {
    can_id_params params;
    decodeCanFrameID(canFrameId, &params);

    if(params.apiClass == 6) {  // 6 corresponds to periodic data being sent back
        switch(params.apiIndex) {
            case 0: { // periodic status 0 - applied output, faults, sticky faults, is follower
                uint8_t appliedBytes[2] = {data[0], data[1]};
                uint8_t faultBytes[2] = {data[2], data[3]};
                uint8_t stickyFaultBytes[2] = {data[4], data[5]};

                appliedOutput = bytesToSignedInt64(appliedBytes, 2) / 32767.0;  // divide by int16 max to scale to [-1, 1]
                faults = bytesToUnsignedInt64(faultBytes, 2);
                stickyFaults = bytesToUnsignedInt64(stickyFaultBytes, 2);
                isFollower = data[7];

                break;
            }

            case 1: {// periodic status 1 - velocity, temperature, voltage, current
                uint8_t velocityBytes[4] = {data[0], data[1], data[2], data[3]};
                _internalEncoderVelocity_rpm = bytesToFloat(velocityBytes, 4);
                temperature_c = data[4];

                // TODO: read voltage and current

                break;
            }

            case 2: { // periodic status 2 - motor position
                uint8_t positionBytes[4] = {data[0], data[1], data[2], data[3]};
                _internalEncoderPosition = bytesToFloat(positionBytes, 4);
                //std::cout << "Motor " << unsigned(deviceId) << " position: " << position << "\n";
                break;
            }

            case 4: { // periodic status 4 - alternate encoder velocity, position
                uint8_t velocityBytes[4] = {data[0], data[1], data[2], data[3]};
                uint8_t positionBytes[4] = {data[4], data[5], data[6], data[7]};
                _altEncoderVelocity_rpm = bytesToFloat(velocityBytes, 4);
                _altEncoderPosition = bytesToFloat(positionBytes, 4);
                break;
            }

            // don't read periodic 3, 5, and 6 because we don't use them


        }
    }
}





/*****************************************************************************************/
/*                                                                                       */
/*                              Firmware Functions                                       */
/*                                                                                       */
/*****************************************************************************************/


// (Config Factory Defaults) Makes simple api call to device
int SparkMaxMC::setToFactoryDefaults() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(7, 4);             // api class and api index

    uint8_t bytes[5] = {0, 0, 0, 0, sparkmax_bool};   // 5 bytes from documentation

    return conn->writeFrame(canId, bytes, 5);
}


// (Config Burn Flash) Makes simple api call to device  
int SparkMaxMC::burnFlash() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(7, 2);  // api class and api index

    uint8_t bytes[2] = {0xA3, 0x3A};       // 2 bytes from documentation

    return conn->writeFrame(canId, bytes, 2);
}


// (Clear Faults) sends api command to clear faults
int SparkMaxMC::clearStickyFaults() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(6, 14);  // api class and api index

    return conn->writeFrame(canId, NULL, 0);
}


// (Periodic Status X) makes API call to change how fast data comes in
int SparkMaxMC::setPeriodicRate(int frameNumber, uint16_t rate) {
    if(frameNumber < 0 || frameNumber > 6) return -4;  // invalid frame choice
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(6, frameNumber);  // api class and api index

    uint8_t bytes[2];  // takes a 2 byte unsigned integer
    floatToBytes(rate, bytes, 2);

    return conn->writeFrame(canId, bytes, 2);  // sends 2 bytes for the new period
}





/*****************************************************************************************/
/*                                                                                       */
/*                                Motion Functions                                       */
/*                                                                                       */
/*****************************************************************************************/


// (Duty Cycle Set) sets the target duty cycle
int SparkMaxMC::dutyCycleSet(float percent, int slot /*0*/) {
    if(conn == NULL) return -1;
    if(percent > 1 || percent < -1) return -4;  // invalid parameters

    uint32_t canId = getCanFrameId(0, 2);  // api class and api index

    uint8_t bytes[8];  // takes a 4 byte floating point number
    getSetpointFrame(bytes, percent, 0, slot);

    return conn->writeFrame(canId, bytes, 8);
}


// (Speed Set) sets the target velocity of the motor in RPM. 
int SparkMaxMC::velocitySet(float targetRPM, int slot /*0*/) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(1, 2);  // api class and api index

    uint8_t bytes[8];  // takes a 4 byte floating point number
    getSetpointFrame(bytes, targetRPM, 0, slot);

    return conn->writeFrame(canId, bytes, 8);
}


// (Smart Velocity Set) sets the target velocity of the motor in RPM. 
// Honors the max acceleration and max velocity from smart motion 
// parameters at the firmware level
int SparkMaxMC::smartVelocitySet(float targetRPM, int slot /*0*/) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(1, 3);  // api class and api index

    uint8_t bytes[8];  // takes a 4 byte floating point number
    getSetpointFrame(bytes, targetRPM, 0, slot);

    return conn->writeFrame(canId, bytes, 8);
}


// (Voltage Set) Sets the closed loop speed controller where the
// target voltage is in volts  
int SparkMaxMC::voltageSet(float targetVoltage, int slot /*0*/) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(4, 2);  // api class and api index

    uint8_t bytes[8];  // takes a 4 byte floating point number
    getSetpointFrame(bytes, targetVoltage, 0, slot);

    return conn->writeFrame(canId, bytes, 8);
}


// (Position Set) Sets the closed loop speed controller where the
// target position is in rotations      
int SparkMaxMC::absPositionSet(float targetRotations, int slot /*0*/) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(3, 2);  // api class and api index

    uint8_t bytes[8];  // takes a 4 byte floating point number
    getSetpointFrame(bytes, targetRotations, 0, slot);

    return conn->writeFrame(canId, bytes, 8);
}


// (Smart Motion Set) Sets the closed loop smart motion 
// controller where the target position is in rotations 
int SparkMaxMC::smartAbsPositionSet(float targetRotations, int slot /*0*/) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(5, 2);  // api class and api index

    uint8_t bytes[8];  // takes a 4 byte floating point number
    getSetpointFrame(bytes, targetRotations, 0, slot);

    return conn->writeFrame(canId, bytes, 8);
}


// calculates where to move to and then moves there
int SparkMaxMC::moveToAngle(float angle_rad, bool smart /*true*/, int slot /*0*/) {
    float currentPosition = getPosition();
    int numRevolutions = (int)currentPosition;

    float currentAngle = (currentPosition - numRevolutions) * 2 * M_PI;
    float diffAngle = angle_rad - currentAngle;

    float desiredPosition = numRevolutions + (diffAngle / (2 * M_PI)) + encoderOffset;

    if(smart) {
        return smartAbsPositionSet(desiredPosition);
    } else {
        return absPositionSet(desiredPosition);
    }
}





/*****************************************************************************************/
/*                                                                                       */
/*                       Parameter Manipulation Functions                                */
/*                                                                                       */
/*****************************************************************************************/


// (Parameter Access) writes the frame that updates a parameter on the spark max
int SparkMaxMC::setGenericParameter(E_SPARKMAX_PARAM param, uint8_t packet[PACKET_LENGTH]) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(48, 0);   // api class and api index
    canId |= (param << DEVICE_NUMBER_BITS);  // from docs, api section of id is or'd with the desired parameter

    return conn->writeFrame(canId, packet, 5);
}


// (Parameter Access) sends a frame to request the data and then polls the connection
// response queue until found or timeout is hit
int SparkMaxMC::readGenericParameter(E_SPARKMAX_PARAM param, uint8_t response[PACKET_LENGTH], int timeout_ms /*100*/) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(48, 0);   // api class and api index
    canId |= (param << DEVICE_NUMBER_BITS);  // from docs, api section of id is or'd with the desired parameter

    uint8_t bytes[0];
    conn->writeFrame(canId, bytes, 0);  // send 0 length message to view the parameter

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = begin;

    uint32_t bitmask = 0xFFFFFFFF;            // response can id should be the same so search for 
    uint32_t specifier = canId & bitmask;     // only that
    uint32_t responseId = 0;

    int dataRead = conn->readNextFrameIf(bitmask, specifier, &responseId, response);
    while(std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() < timeout_ms && dataRead != 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(400)); // wait for a bit before re-trying
        dataRead = conn->readNextFrameIf(bitmask, specifier, &responseId, response);

        end = std::chrono::steady_clock::now();
    }

    return dataRead;
}





/*****************************************************************************************/
/*                                                                                       */
/*                             Motor Config Functions                                    */
/*                                                                                       */
/*****************************************************************************************/


// makes parameter update call to set the motor reversed setting
int SparkMaxMC::setMotorReversed(bool reverse) {
    uint8_t setReverse = 0;
    if(reverse) setReverse = 1;

    uint8_t data[5] = {setReverse, 0, 0, 0, sparkmax_bool};

    int response = setGenericParameter(kInverted, data);
    if(response == 0) {
        isReversed = reverse;
    } 

    return response;
}


// makes parameter update call to set idle mode
int SparkMaxMC::setIdleMode(uint8_t newIdleMode) {
    uint8_t data[5] = {newIdleMode, 0, 0, 0, sparkmax_uint};

    return setGenericParameter(kIdleMode, data);
}





/*****************************************************************************************/
/*                                                                                       */
/*                             Encoder Config Functions                                  */
/*                                                                                       */
/*****************************************************************************************/


// makes parameter update call to set the encoder mode
int SparkMaxMC::setAltEncoderMode(bool alternate) {
    uint8_t useAlt = 0;
    if(alternate) useAlt = 1;

    uint8_t data[5] = {useAlt, 0, 0, 0, sparkmax_uint};

    int response = setGenericParameter(kDataPortConfig, data);
    if(response == 0) {
        encoderMode = useAlt;
    } 

    return response;

}


// Makes parameter set call to set the number of ticks per revolution
// for the encoder. Chooses the internal encoder or the alternate
// encoder based on the current encoder mode
int SparkMaxMC::setTicksPerEncoderRevolution(unsigned int ticks) {
    uint8_t kTicksData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint64ToBytes(ticks, kTicksData, 4);
    kTicksData[4] = sparkmax_uint;

    if(encoderMode == 0) {
        return setGenericParameter(kEncoderCountsPerRev, kTicksData);
    } else if(encoderMode == 1) {
        return setGenericParameter(kAltEncoderCountsPerRev, kTicksData);
    } else {
        return -4;  // invalid parameters
    }
}


// Sets whether the alternate encoder should be reversed or not
int SparkMaxMC::setAltEncoderReversed(bool reverse) {
    uint8_t setReverse = 0;
    if(reverse) setReverse = 1;

    uint8_t data[5] = {setReverse, 0, 0, 0, sparkmax_bool};

    int response = setGenericParameter(kAltEncoderInverted, data);
    if(response == 0) {
        encoderIsReversed = reverse;
    } 

    return response;
}





/*****************************************************************************************/
/*                                                                                       */
/*                                PID Config Functions                                   */
/*                                                                                       */
/*****************************************************************************************/


// makes call to parameter set function
int SparkMaxMC::setkP(float kP, int slot /*0*/) {
    uint8_t kP_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytes(kP, kP_data, 4);
    kP_data[4] = sparkmax_float32;
    switch(slot) {
        case 0:  return setGenericParameter(kP_0, kP_data);
        case 1:  return setGenericParameter(kP_1, kP_data);
        case 2:  return setGenericParameter(kP_2, kP_data);
        case 3:  return setGenericParameter(kP_3, kP_data);
        default: return -1;
    }
}


// makes call to parameter set function
int SparkMaxMC::setkI(float kI, int slot /*0*/) {
    uint8_t kI_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytes(kI, kI_data, 4);
    kI_data[4] = sparkmax_float32;
    switch(slot) {
        case 0:  return setGenericParameter(kI_0, kI_data);
        case 1:  return setGenericParameter(kI_1, kI_data);
        case 2:  return setGenericParameter(kI_2, kI_data);
        case 3:  return setGenericParameter(kI_3, kI_data);
        default: return -1;
    }
}


// makes call to parameter set function
int SparkMaxMC::setkD(float kD, int slot /*0*/) {
    uint8_t kD_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytes(kD, kD_data, 4);
    kD_data[4] = sparkmax_float32;
    switch(slot) {
        case 0:  return setGenericParameter(kD_0, kD_data);
        case 1:  return setGenericParameter(kD_1, kD_data);
        case 2:  return setGenericParameter(kD_2, kD_data);
        case 3:  return setGenericParameter(kD_3, kD_data);
        default: return -1;
    }
}


// makes call to parameter set function
int SparkMaxMC::setkF(float kF, int slot /*0*/) {
    uint8_t kF_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytes(kF, kF_data, 4);
    kF_data[4] = sparkmax_float32;
    switch(slot) {
        case 0:  return setGenericParameter(kF_0, kF_data);
        case 1:  return setGenericParameter(kF_1, kF_data);
        case 2:  return setGenericParameter(kF_2, kF_data);
        case 3:  return setGenericParameter(kF_3, kF_data);
        default: return -1;
    }
}


// makes call to all of the individual set functions, this
// is purely for convenience. Return value is sum of all 
// return values from each parameter set
int SparkMaxMC::setPIDF(float kP, float kI, float kD, float kF, int slot /*0*/) {
    int response = 0;
    response += setkP(kP, slot);
    response += setkI(kI, slot);
    response += setkD(kD, slot);
    response += setkF(kF, slot);
    return response;
}





/*****************************************************************************************/
/*                                                                                       */
/*                                Encoder Functions                                      */
/*                                                                                       */
/*****************************************************************************************/


// Sets the new offset position of the encoder
void SparkMaxMC::tareEncoder() {
    if(encoderMode == 0) {
        encoderOffset = _internalEncoderPosition;
    } else if(encoderMode == 1) {
        encoderOffset = _altEncoderPosition;
    }
}


// Chooses which velocity to return based on the encoder mode
float SparkMaxMC::getVelocity() {
    if(encoderMode == 0) {
        return gearRatio * _internalEncoderVelocity_rpm;
    } else if(encoderMode == 1) {
        return gearRatio * _altEncoderVelocity_rpm;
    } else {
        return 0;
    }
}


// Chooses which position to return based on the encoder mode.
// Subtracts the encoder offset to respect zeroing
float SparkMaxMC::getPosition() {
    if(encoderMode == 0) {
        return gearRatio * (_internalEncoderPosition - encoderOffset);
    } else if(encoderMode == 1) {
        return gearRatio * (_altEncoderPosition - encoderOffset);
    } else {
        return 0;
    }
}


// chooses which value to return based on encoder mode
float SparkMaxMC::getAbsPosition() {
    if(encoderMode == 0) {
        return _internalEncoderPosition;
    } else if(encoderMode == 1) {
        return _altEncoderPosition;
    } else {
        return 0;
    }
}


// calculates the current angle
float SparkMaxMC::getAngle_rad() {
    float position = getPosition();
    float revolution = position - (int)position;  // get the decimal number of revolutions (ignore number of times around)
    return revolution * 2 * M_PI;                 // angle = revolutions * 2pi rad/revolution
}





/*****************************************************************************************/
/*                                                                                       */
/*                          Parameter Access Functions                                   */
/*                                                                                       */
/*****************************************************************************************/


// makes call to read parameter for each of the individual 
// pid constants
int SparkMaxMC::getFullPIDF(pidf_constants& constants, int slot /*0*/) {
    if(slot < 0 || slot > 3) return -4;  // not sent, invalid slot

    int response = 0;

    // array for the data to be read into
    uint8_t response_data[8];
    float values[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    const int timeout = 50;  // don't wait longer than 50ms for each call

    E_SPARKMAX_PARAM paramsToRead[8];
    paramsToRead[0] = static_cast<E_SPARKMAX_PARAM>(kP_0 + (8 * slot));  // constants are all right next to each other
    paramsToRead[1] = static_cast<E_SPARKMAX_PARAM>(kI_0 + (8 * slot));  // with an offset of 8 for each group
    paramsToRead[2] = static_cast<E_SPARKMAX_PARAM>(kD_0 + (8 * slot));
    paramsToRead[3] = static_cast<E_SPARKMAX_PARAM>(kF_0 + (8 * slot));
    paramsToRead[4] = static_cast<E_SPARKMAX_PARAM>(kIZone_0 + (8 * slot));
    paramsToRead[5] = static_cast<E_SPARKMAX_PARAM>(kDFilter_0 + (8 * slot));
    paramsToRead[6] = static_cast<E_SPARKMAX_PARAM>(kOutputMin_0 + (8 * slot));
    paramsToRead[7] = static_cast<E_SPARKMAX_PARAM>(kOutputMax_0 + (8 * slot));

    for(int i = 0; i < 8; i++) {
        memset(response_data, 0, sizeof(response_data));  // clear data array to all 0 before reading again

        if(readGenericParameter(paramsToRead[i], response_data, 50) == 0) {  // read the parameter
            values[i] = bytesToFloat(response_data, 4);
        } else {
            response -= 1;
        }
    }

    constants.kP = values[0];
    constants.kI = values[1];
    constants.kD = values[2];
    constants.kF = values[3];
    constants.kIZone = values[4];
    constants.kDFilter = values[5];
    constants.kOutputMin = values[6];
    constants.kOutputMax = values[7];


    return response;
}





/*****************************************************************************************/
/*                                                                                       */
/*                                Debug Functions                                        */
/*                                                                                       */
/*****************************************************************************************/


// prints faults in a human readable format. There are 16 possible
// faults. These were determined from the REV Hardware client, so 
// it is possible that they are not accurate (they were not documented
// anywhere else)
void SparkMaxMC::printFaults(uint16_t faultString) {
    const char* faultStrings[16] = {
        "Brownout", "Over Current", "Watchdog Reset", "Motor Type",
        "Sensor Fault", "Stall", "EEPROM", "CAN TX",
        "CAN RX", "Has Reset", "Gate Driver Fault", "Hardware Fault",
        "Soft Limit Forward", "Soft Limit Reverse", "Hard Limit Forward", "Hard Limit Reverse"
    };
    printf("Fault Decoding for %d:\n", faultString);
    for(int i = 0; i < 16; i++) {
        int bit = faultString & 1;
        printf("    %s: %d\n", faultStrings[i], bit);

        faultString >>= 1;  // shift to next bit
    }

    printf("\n");
}


// makes api call to get pid info and then prints using cout
void SparkMaxMC::printPID(int slot) {
    pidf_constants constants;
    int response = getFullPIDF(constants, slot);

    std::cout << "PID Constants for slot " << slot << "\n";
    std::cout << "    Response:   " << response << "\n";
    std::cout << "    kP:         " << constants.kP << "\n";
    std::cout << "    kI:         " << constants.kI << "\n";
    std::cout << "    kD:         " << constants.kD << "\n";
    std::cout << "    kF:         " << constants.kF << "\n";
    std::cout << "    kI Zone:    " << constants.kIZone << "\n";
    std::cout << "    kD Filter:  " << constants.kDFilter << "\n";
    std::cout << "    kOutputMin: " << constants.kOutputMin << "\n";
    std::cout << "    kOutputMax: " << constants.kOutputMax << "\n";
}


// (Identify) Causes the motor controller LED to flash
// rapidly so it can be identified 
int SparkMaxMC::identify() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(7, 6);  // api class and api index

    uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // just send empty message

    return conn->writeFrame(canId, bytes, 0);
}
