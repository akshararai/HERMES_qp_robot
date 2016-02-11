#ifndef MS_3DMGX3_25_HH
#define MS_3DMGX3_25_HH

class microstrain_3dmgx3_25 {

 public:

    //! Enumeration of possible IMU commands
    enum cmd {
      CMD_RAW                      =  0xC1,
      CMD_ACCEL_ANGRATE            =  0xC2,
      CMD_DELVEL_DELANG            =  0xC3,
      CMD_CONTINUOUS               =  0xC4,
      CMD_ORIENT                   =  0xC5,
      CMD_ATT_UPDATE               =  0xC6,
      CMD_MAG_VEC                  =  0xC7,
      CMD_ACCEL_ANGRATE_ORIENT     =  0xC8,
      CMD_WRITE_ACCEL_BIAS         =  0xC9,
      CMD_WRITE_GYRO_BIAS          =  0xCA,
      CMD_ACCEL_ANGRATE_MAG        =  0xCB,
      CMD_ACCEL_ANGRATE_MAG_ORIENT =  0xCC,
      CMD_CAPTURE_GYRO_BIAS        =  0xCD,
      CMD_EULER                    =  0xCE,
      CMD_EULER_ANGRATE            =  0xCF,
      CMD_TEMPERATURES             =  0xD1,
      CMD_GYROSTAB_ANGRATE_MAG     =  0xD2,
      CMD_DELVEL_DELANG_MAG        =  0xD3,
      CMD_QUATERNION               =  0xDF,
      CMD_TIMESTAMP                =  0xD7,
      CMD_REALIGN_UP_NORTH         =  0xDD,
      CMD_COMMUNICATION_SETTINGS   =  0xD9,
      CMD_SAMPLING_SETTINGS        =  0xDB,
      CMD_DEV_ID_STR               =  0xEA,
      CMD_STOP_CONTINUOUS          =  0xFA,
      CMD_FIRMWARE_NUMBER          =  0xE9
    };

    static const double GRAV_CONST = 9.80665; // gravity constant
    static const int DATA_RATE_DECIM = 1; // 1000 hz

    microstrain_3dmgx3_25(bool is_45); // constructor
    ~microstrain_3dmgx3_25(void); // destructor

    bool openPort(const char *port_name); // opens port, sets communication and sampling settings
    bool closePort(void); // closes port

    bool switchMode(bool to_25);

    int read_with_poll(int fd, uint8_t* buf, size_t count); // read using poll
    bool send(uint8_t* cmd, int cmd_len); // sends a command
    bool receive(uint8_t command, uint8_t* rep, int rep_len); // receives a response
    bool transact(uint8_t* cmd, int cmd_len, uint8_t* rep, int rep_len); // sends a command and then receives a response

    bool setCommunicationSettings(void); // set communication settings
    bool setSamplingSettings(void); // set sampling settings
    bool initTimestamp(void); // set timestamp to zero
    bool setContinuous(cmd command); // set to continuous (streaming) mode
    void stopContinuous(void); // stop streaming mode

    void receiveAccelAngrate(double* accel, double* angrate, double &timestamp); // receive raw accel/angrate

 private:

    int fd_;
    bool is_45_;
    bool continuous_;

};

#endif
