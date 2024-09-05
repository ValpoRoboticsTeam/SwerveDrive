// SparkMaxMC.hpp
// Contains class for dealing with a Spark Max Motor Controller based
// on their provided documentation
//
// Author: Aiden Carney

#ifndef SPARKMAXMC_HPP
#define SPARKMAXMC_HPP


#include "CANConnection.hpp"
#include "CANDevice.hpp"
#include "util/PIDController.hpp"


// each parameter has a type associated with it that corresponds to an
// integer value, these are not documented anywhere else and were 
// found from trial and error. These are needed because to update 
// parameters, one of the bytes needs to be the type.
enum E_SPARKMAX_PARAM_TYPE {
    sparkmax_uint = 1,
    sparkmax_float32,
    sparkmax_bool
};

// the valid parameters on the sparkmax that can be viewed and
// changed. These are available in more detail on Rev Robotics
// website: https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters
enum E_SPARKMAX_PARAM {
    kCanID = 0,
    kInputMode,
    kMotorType,
    kCommAdvance,
    kSensorType,
    kCtrlType,
    kIdleMode,
    kInputDeadband,
    kFeedbackSensorPID0,
    kFeedbackSensorPID1,
    kPolePairs,
    kCurrentChop,
    kCurrentChopCycles,
    kP_0,
    kI_0,
    kD_0,
    kF_0,
    kIZone_0,
    kDFilter_0,
    kOutputMin_0,
    kOutputMax_0,
    kP_1,
    kI_1,
    kD_1,
    kF_1,
    kIZone_1,
    kDFilter_1,
    kOutputMin_1,
    kOutputMax_1,
    kP_2,
    kI_2,
    kD_2,
    kF_2,
    kIZone_2,
    kDFilter_2,
    kOutputMin_2,
    kOutputMax_2,
    kP_3,
    kI_3,
    kD_3,
    kF_3,
    kIZone_3,
    kDFilter_3,
    kOutputMin_3,
    kOutputMax_3,
    kInverted,
    kOutputRatio,
    kSerialNumberLow,
    kSerialNumberMid,
    kSerialNumberHigh,
    kLimitSwitchFwdPolarity,
    kLimitSwitchRevPolarity,
    kHardLimitFwdEn,
    kHardLimitRevEn,
    kSoftLimitFwdEn,
    kSoftLimitRevEn,
    kRampRate,
    kFollowerID,
    kFollowerConfig,
    kSmartCurrentStallLimit,
    kSmartCurrentFreeLimit,
    kSmartCurrentConfig,
    kSmartCurrentReserved,
    kMotorKv,
    kMotorR,
    kMotorL,
    kMotorRsvd1,
    kMotorRsvd2,
    kMotorRsvd3,
    kEncoderCountsPerRev,
    kEncoderAverageDepth,
    kEncoderSampleDelta,
    kEncoderInverted,
    kEncoderRsvd1,
    kClosedLoopVoltageMode,
    kCompensatedNominalVoltage,
    kSmartMotionMaxVelocity_0,
    kSmartMotionMaxAccel_0,
    kSmartMotionMinVelOutput_0,
    kSmartMotionAllowedClosedLoopError_0,
    kSmartMotionAccelStrategy_0,
    kSmartMotionMaxVelocity_1,
    kSmartMotionMaxAccel_1,
    kSmartMotionMinVelOutput_1,
    kSmartMotionAllowedClosedLoopError_1,
    kSmartMotionAccelStrategy_1,
    kSmartMotionMaxVelocity_2,
    kSmartMotionMaxAccel_2,
    kSmartMotionMinVelOutput_2,
    kSmartMotionAllowedClosedLoopError_2,
    kSmartMotionAccelStrategy_2,
    kSmartMotionMaxVelocity_3,
    kSmartMotionMaxAccel_3,
    kSmartMotionMinVelOutput_3,
    kSmartMotionAllowedClosedLoopError_3,
    kSmartMotionAccelStrategy_3,
    kIMaxAccum_0,
    kSlot3Placeholder1_0,
    kSlot3Placeholder2_0,
    kSlot3Placeholder3_0,
    kIMaxAccum_1,
    kSlot3Placeholder1_1,
    kSlot3Placeholder2_1,
    kSlot3Placeholder3_1,
    kIMaxAccum_2,
    kSlot3Placeholder1_2,
    kSlot3Placeholder2_2,
    kSlot3Placeholder3_2,
    kIMaxAccum_3,
    kSlot3Placeholder1_3,
    kSlot3Placeholder2_3,
    kSlot3Placeholder3_3,
    kPositionConversionFactor,
    kVelocityConversionFactor,
    kClosedLoopRampRate,
    kSoftLimitFwd,
    kSoftLimitRev,
    kSoftLimitRsvd0,
    kSoftLimitRsvd1,
    kAnalogPositionConversion,
    kAnalogVelocityConversion,
    kAnalogAverageDepth,
    kAnalogSensorMode,
    kAnalogInverted,
    kAnalogSampleDelta,
    kAnalogRsvd0,
    kAnalogRsvd1,
    kDataPortConfig,
    kAltEncoderCountsPerRev,
    kAltEncoderAverageDepth,
    kAltEncoderSampleDelta,
    kAltEncoderInverted,
    kAltEncoderPositionFactor,
    kAltEncoderVelocityFactor,
    kAltEncoderRsvd0,
    kAltEncoderRsvd1,
    kExtFFGain0,
    kExtFFGain1,
    kExtFFGain2,
    kExtFFGain3,
    kExtFFReserved0,
    kExtFFReserved1
};


class SparkMaxMC : public CANDevice {
    private:
        // method for generating the can frame ID for a spark max
        // motor controller based on its api class and index
        //
        // Params:
        //    apiClass - the API Class integer from the spark max docs
        //    apiIndex - the API Index integer from the spark max docs
        // Return:
        //    uint32_t - the can frame id
        uint32_t getCanFrameId(int apiClass, int apiIndex);


        // Fills in the data for a setpoint frame - i.e. motion commands
        //
        // Params:
        //    data     - where to write the bytes to
        //    setpoint - the setpoint
        //    arbFF    - an arbitrary amount of voltage to add to the motor output in volts (multiplied by 0.0009765625)
        //    pidSlot  - which pid slot to use
        // Return:
        //    None
        void getSetpointFrame(uint8_t bytes[8], float setpoint, int16_t arbFF, uint8_t pidSlot);


        // prints tothe terminal the corresponding id information based on the frc
        // protocal for a given frame
        //
        // Params:
        //    canFrameId - the id field of the can frame
        //    data       - array that contains the data sent with the frame
        // Return:
        //    None
        void debugIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]);


        float appliedOutput = 0;  // ranges from [-1, 1]
        int faults = 0;
        int stickyFaults = 0;
        bool isFollower = false;
        int temperature_c = 0;    // motor temperature in degrees c

        float busVoltage = 0;  // the voltage drop of the motor
        float busCurrent = 0;  // the current supplied to the motor

        float _internalEncoderVelocity_rpm = 0;  // the velocity of the internal encoder in rpm
        float _internalEncoderPosition = 0;      // the position of the built-in encoder in revolutions (not scaled or offset)
        float _altEncoderVelocity_rpm = 0;       // the velocity of the alternate encoder in rpm
        float _altEncoderPosition = 0;           // the position of the alternate encoder in revolutions (not scaled or offset)

        int encoderMode;         // 0 for built-in, 1 for alternate
        bool isReversed;         // if the motor is reversed or not
        bool encoderIsReversed;  // if the encoder is reversed
        float encoderOffset;     // used for zeroing the encoder (either for internal or alternate
                                 // encoder based on which encoder mode is on)
        float gearRatio = 1.0;   // multiplied by position and velocity so they correspond to the output
                                 // of the mechanism and not the motor


    public:
        // default ctor. Does not initialize anything about the device and this
        // should be done later if the instance is to be used successfully
        //
        // Params:
        //    None
        // Return:
        //    The new instance
        SparkMaxMC();


        // ctor. Initializes connection and the device id
        //
        // Params:
        //    connection  - a reference to the connection instance for the CAN network
        //    canDeviceId - the id to use for this motor controller. Can be set using
        //                  the REV Hardware client
        // Return:
        //    The new instance
        SparkMaxMC(CANConnection& connection, int canDeviceId);


        // destructor - makes sure motors are off
        ~SparkMaxMC();




        // Takes an incoming CAN Frame and responds accordingly.
        //
        // Params:
        //    canFrameId - the integer containing 29 bits that correspond to the 
        //                 id of the can frame
        //    data       - the data to be parsed
        // Return:
        //    None
        void _parseIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]) override;




        /*****************************************************************************************/
        /*                                                                                       */
        /*                              Firmware Functions                                       */
        /*                                                                                       */
        /*****************************************************************************************/

        // (Config Factory Defaults) Sets all parameters to factory defaults. This is useful for 
        // starting the motor controller from a known state. If this method is to work properly,
        // it should be called before any other motor methods and before any heartbeat is sent
        //
        // Params:
        //    None
        // Return:
        //    int - if the command was sent successfully   
        int setToFactoryDefaults();


        // (Config Burn Flash) Burns to flash all the parameters currently set that have been 
        // changed. This is useful for if the motor controller brown's out that it doesn't 
        // need to be reconfigured
        //
        // Params:
        //    None
        // Return:
        //    int - if the command was sent successfully   
        int burnFlash();


        // (Clear Faults) Clears the sticky faults set by the motor controller.
        //
        // Params:
        //    None
        // Return:
        //    int - if the command was sent successfully   
        int clearStickyFaults();


        // (Periodic Status X) Sets the rate at which data is sent from the motor 
        // controller for one of the periodic frames. This is helpful to decrease 
        // traffic on the CAN Network
        //
        // Params:
        //    frameNumber - which periodic frame to change the rate of
        //    rate        - delay between frames in ms (period)
        // Return:
        //    int  - 0 if response was received
        int setPeriodicRate(int frameNumber, uint16_t rate);




        /*****************************************************************************************/
        /*                                                                                       */
        /*                                Motion Functions                                       */
        /*                                                                                       */
        /*****************************************************************************************/

        // (Duty Cycle Set) Sets the pwm duty cycle of the motor controller.
        //
        // Params:
        //    percent - value between [-1, 1] corresponding to the requested duty cycle
        //    slot    - the PID slot to use
        // Return:
        //    int - if the command was sent successfully
        int dutyCycleSet(float percent, int slot=0);


        // (Speed Set) sets the target velocity of the motor in RPM. 
        //
        // Params:
        //    targetRPM - the new target speed in RPM
        //    slot    - the PID slot to use
        // Return:
        //    int - if the command was sent successfully
        int velocitySet(float targetRPM, int slot=0);


        // (Smart Velocity Set) sets the target velocity of the motor in RPM. 
        // Honors the max acceleration and max velocity from smart motion 
        // parameters at the firmware level
        //
        // Params:
        //    targetRPM - the new target speed in RPM
        //    slot    - the PID slot to use
        // Return:
        //    int - if the command was sent successfully
        int smartVelocitySet(float targetRPM, int slot=0);


        // (Voltage Set) Sets the closed loop speed controller where the
        // target voltage is in volts
        //
        // Params:
        //    targetVoltage: - the target voltage in units of volts
        //    slot    - the PID slot to use
        // Return:
        //    int - if the command was sent successfully     
        int voltageSet(float targetVoltage, int slot=0);


        // (Position Set) Sets the closed loop speed controller where the
        // target position is in rotations. Moves to the absolute position
        // without respecting any previous zeroing of the encoder.
        // NOTE: this api call appears to not function properly on the sparkmax.
        // DON'T USE UNTIL FIRMWARE FIXED
        //
        // Params:
        //    targetRotations: - the target position in units of rotations
        //    slot    - the PID slot to use
        // Return:
        //    int - if the command was sent successfully       
        int absPositionSet(float targetRotations, int slot=0);


        // (Smart Motion Set) Sets the closed loop smart motion controller 
        // where the target position is in rotations Moves to the absolute 
        // position without respecting any previous zeroing of the encoder
        // NOTE: this api call appears to not function properly on the sparkmax.
        // DON'T USE UNTIL FIRMWARE FIXED
        //
        // Params:
        //    targetRotations: - the target position in rotations
        //    slot    - the PID slot to use
        // Return:
        //    int - if the command was sent successfully  
        int smartAbsPositionSet(float targetRotations, int slot=0);


        // moves to an angle based on the current motor position. Uses one 
        // of the absPositionSet methods but calculates the position to move
        // to that is the least amount of travel. Respects gear ratio
        // NOTE: this method relies on api calls that appear to not function 
        // properly on the sparkmax. DON'T USE UNTIL FIRMWARE FIXED
        //
        // Params:
        //    angle_rad - the angle to move to in radians
        //    smart     - use the smart motion function that respects ramp rates or not
        //    slot    - the PID slot to use
        // Return:
        //    int - if the command was sent successfully  
        int moveToAngle(float angle_rad, bool smart=true, int slot=0);




        /*****************************************************************************************/
        /*                                                                                       */
        /*                       Parameter Manipulation Functions                                */
        /*                                                                                       */
        /*****************************************************************************************/

        // (Parameter Access) Sets a generic parameter on the sparkmax. Must supply a the
        // correct packet structure
        // 
        // Params:
        //    param  - the parameter to write, of enumerated type
        //    packet - what to send to the sparkmax. Up to the caller
        //             to determine how this should be interpreted (read the docs)
        // Return:
        //    int - if the command was sent successfully
        int setGenericParameter(E_SPARKMAX_PARAM param, uint8_t packet[PACKET_LENGTH]);


        // (Parameter Access) reads a parameter from the sparkmax and places the output value into 
        // the array. Returns 0 if found a response within the maximum allowed
        // time. This function has an optional timeout because this operation
        // could spend a lot of time holding a mutex somewhere if it doesn't get a 
        // response immediately. 
        //
        // Params:
        //    param      - the parameter to read of enumerated type
        //    response   - where to place the response if any. Up to the caller
        //                 to determine how this should be interpreted (read the docs)
        //    timeout_ms - (optional) max time to spend in this function
        // Return:
        //    int  - 0 if response was received
        int readGenericParameter(E_SPARKMAX_PARAM param, uint8_t response[PACKET_LENGTH], int timeout_ms=100);




        /*****************************************************************************************/
        /*                                                                                       */
        /*                             Motor Config Functions                                    */
        /*                                                                                       */
        /*****************************************************************************************/

        // Sets whether the motor should be reversed or not
        //
        // Params:
        //    reverse - should the motor be reversed
        // Return:
        //    int - if the command was sent successfully
        int setMotorReversed(bool reverse);


        // sets the idle mode for the sparkmax to either brake or coast
        //
        // Params:
        //    newIdleMode - 0 for coast, 1 for brake
        // Return:
        //    int - if the command was sent successfully        
        int setIdleMode(uint8_t newIdleMode);



        /*****************************************************************************************/
        /*                                                                                       */
        /*                             Encoder Config Functions                                  */
        /*                                                                                       */
        /*****************************************************************************************/

        // Sets the encoder mode for the motor controller. The two options are either
        // the built-in encoder or the alternate encoder
        //
        // Params:
        //    alternate - if the encoder mode should be alternate or internal
        // Return:
        //    int - if the command was sent successfully
        int setAltEncoderMode(bool alternate);


        // Sets a scalar to be multiplied by all data coming from the encoder (velocity, position,
        // etc.) to factor things like gear ratios or unit conversions. This value is strictly
        // multiplied so a gear reduction should supply a number less than 1.
        // 
        // Params:
        //    ratio - the scalar to multiply all terms used in the motor controller by
        // Return:
        //    None
        void setGearRatio(float ratio) {gearRatio = ratio;}


        // Makes parameter set call to set the number of ticks per revolution
        // for the encoder. Updates either the internal encoder or the alternate
        // encoder based on the current mode. This number should be based on the
        // specifications of the encoder and not any gear ratio
        //
        // Params:
        //    ticks - the new number of ticks per revolution
        // Return:
        //    int - if the command was sent successfully
        int setTicksPerEncoderRevolution(unsigned int ticks);


        // Sets whether the alternate encoder should be reversed or not
        // 
        // Params:
        //    reverse - should the alternate encoder be reversed
        // Return:
        //    int - if the command was sent successfully
        int setAltEncoderReversed(bool reverse);




        /*****************************************************************************************/
        /*                                                                                       */
        /*                                PID Config Functions                                   */
        /*                                                                                       */
        /*****************************************************************************************/

        // Sets the P constant on the motor controller for a given slot (default 0)
        //
        // Params:
        //    kP   - the new value for kP
        //    slot - the pid slot (0, 1, 2, 3)
        // Return:
        //    int - if the command was sent successfully
        int setkP(float kP, int slot=0);


        // Sets the I constant on the motor controller for a given slot (default 0)
        //
        // Params:
        //    kI   - the new value for kI
        //    slot - the pid slot (0, 1, 2, 3)
        // Return:
        //    int - if the command was sent successfully
        int setkI(float kI, int slot=0);


        // Sets the D constant on the motor controller for a given slot (default 0)
        //
        // Params:
        //    kD   - the new value for kD
        //    slot - the pid slot (0, 1, 2, 3)
        // Return:
        //    int - if the command was sent successfully
        int setkD(float kD, int slot=0);


        // Sets the feed-forward constant on the motor controller for a given slot (default 0)
        //
        // Params:
        //    kF   - the new value for kF
        //    slot - the pid slot (0, 1, 2, 3)
        // Return:
        //    int - if the command was sent successfully
        int setkF(float kF, int slot=0);


        // updates all the PIDF parameters for the motor controller for
        // the given slot
        // 
        // Params:
        //    kP   - the new proportional constant
        //    kI   - the new integral constant
        //    kD   - the new derivative constant
        //    kF   - the new feed-forward constant
        //    slot - the pid slot (0, 1, 2, 3)
        // Return:
        //    int - if the command was sent successfully   
        int setPIDF(float kP, float kI, float kD, float kF, int slot=0);




        /*****************************************************************************************/
        /*                                                                                       */
        /*                                Encoder Functions                                      */
        /*                                                                                       */
        /*****************************************************************************************/


        // (Telemetry Update Mechanical Position Enoder Port) Zeros
        // the encoder value by updating the position held by the 
        // motor controller.
        //
        // Params:
        //    None
        // Return:
        //    None
        void tareEncoder();


        // Sets the tare position of the encoder to a custom value rather 
        // than the current position. This is useful for restoring a
        // previous state
        //
        // Params:
        //    newOffset - the new tare position in rotations
        // Return:
        //    None
        void setTarePosition(float newOffset) {encoderOffset = newOffset;}


        // Gets the current velocity of the encoder (either alternate or internal
        // based on the mode) in RPM
        //
        // Params:
        //    None
        // Return:
        //    float - the velocity of the encoder in RPM
        float getVelocity();


        // Gets the current position of the encoder. Note: this is not the
        // raw value received from the motor controller, an offset is
        // subtracted from it to allow for zeroing the encoder. Returns
        // the encoder position based on the set encoder mode
        //
        // Params:
        //    None
        // Return:
        //    float - the current encoder position
        float getPosition();


        // Gets the current absolute position of the encoder
        //
        // Params:
        //    None
        // Return:
        //    the current encoder value
        float getAbsPosition();


        // Gets the current angle of the encoder on interval [0, 2pi]
        // while respecting the gear ratio and the encoder offset
        // 
        // Params:
        //    None
        // Return:
        //    float - the current angle in radians
        float getAngle_rad();
        



        /*****************************************************************************************/
        /*                                                                                       */
        /*                          Parameter Access Functions                                   */
        /*                                                                                       */
        /*****************************************************************************************/


        // reads the pid constants used by the motor controller for given
        // slot. Stores it in a reference struct, returns 0 if all
        // messages were read successfully
        //
        // Params:
        //    constants - a reference to the structure where the data should be read
        //    slot      - which pid constants slot to read (default 0)
        // Return:
        //    int - if the command was sent successfully          
        int getFullPIDF(pidf_constants& constants, int slot=0);




        /*****************************************************************************************/
        /*                                                                                       */
        /*                                    Getters                                            */
        /*                                                                                       */
        /*****************************************************************************************/

        // Gets the last applied output the motor uses from the 
        // periodic data sent by the motor controller
        //
        // Params:
        //    None
        // Return:
        //    float - the current applied output the motor uses on interval [-1, 1]
        float getAppliedOutput() {return appliedOutput;}


        // Gets the current faults set by the motor
        //
        // Params:
        //    None
        // Return:
        //    int - the current faults
        int getFaults() {return faults;}

        
        // Gets the current sticky faults set by the motor
        //
        // Params:
        //    None
        // Return:
        //    int - the current sticky faults
        int getStickyFaults() {return stickyFaults;}


        // Returns if the motor is a follower. This is periodic data
        // sent back that is made available 
        // 
        // Params:
        //    None
        // Return:
        //    bool - is the motor following another motor
        bool getIsFollower() {return isFollower;}


        // Returns the current temperature of the motor in degrees C
        //
        // Params: 
        //    None
        // Return:
        //    int - the temperature of the motor in degrees C
        int getTemperature() {return temperature_c;}


        // Returns the input voltage to the controller
        //
        // Params:
        //    None
        // Return:
        //    float - the voltage in units V
        float getBusVoltage() {return busVoltage;}


        // Returns the raw phase current of the motor in A
        //
        // Params:
        //    None
        // Return:
        //    float - the current in amps
        float getBusCurrent() {return busCurrent;}


        // returns the current encoder mode. 1 for alternate, 0 for internal
        //
        // Params:
        //    None
        // Return:
        //    int - the encoder mode used by the motor controller
        int getEncoderMode () {return encoderMode;}       


        // if the motor is reversed or not
        // Params:
        //    None
        // Return:
        //    bool - is the motor reversed
        bool getIsReversed() {return isReversed;}


        // is the encoder reversed. Only meaningful for the alternate encoder because
        // the internal encoder can never be out of phase with the motor motion (this
        // is detected automatically by the motor controller)
        //
        // Params:
        //    None
        // Return:
        //    bool - is the encoder reversed
        bool getEncoderIsReversed() {return encoderIsReversed;}


        // returns the current encoder offset used when taring the encoders
        // 
        // Params:
        //    None
        // Return:
        //    float - the current encoder offset
        float getEncoderOffset() {return encoderOffset;}


        // returns the current gear ratio being used with the encoders
        //
        // Params:
        //    None
        // Return:
        //    float - the current gear ratio
        float getGearRatio() {return gearRatio;}


        /*****************************************************************************************/
        /*                                                                                       */
        /*                                Debug Functions                                        */
        /*                                                                                       */
        /*****************************************************************************************/

        // Prints the faults to stdout based on the fault
        // specifier string
        //
        // Params:
        //    faultString - the integer that holds the current faults
        // Return:
        //    None
        void printFaults(uint16_t faultString);


        // Prints the PID constants to stdout for debugging
        //
        // Params:
        //    slot - which PID slot to print
        // Return:
        //    None
        void printPID(int slot);


        // (Identify) Causes the motor controller LED to flash
        // rapidly so it can be identified
        //
        // Params:
        //    None:
        // Return;
        //    int - if the command was sent successfully  
        int identify();


};




#endif
