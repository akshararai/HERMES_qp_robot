#ifndef IMU_INTERFACE_H_
#define IMU_INTERFACE_H_
 
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include <rtdm/rtserial.h>
#include <native/task.h>

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "SL.h"
#include "SL_rt_mutex.h"


#define CMD_AC_AN			0xc2
#define  CMD_AC_AN_LEN	        	1
#define  RPLY_AC_AN_LEN  		31

#define CMD_CONT_MODE			0xc4
#define  CMD_CONT_MODE_LEN		4
#define  CONT_MODE_CONF1		0xc1
#define  CONT_MODE_CONF2		0x29
#define  RPLY_CONT_MODE_LEN		8

#define CMD_SAMP_SETTINGS               0xdb
#define  CMD_SAMP_SETTINGS_LEN          20
#define  SAMP_SETTINGS_CONF1            0xa8
#define  SAMP_SETTINGS_CONF2            0xb9
#define  RPLY_SAMP_SETTINGS_LEN         19

#define CMD_COMM_SETTINGS               0xd9
#define  CMD_COMM_SETTINGS_LEN          11
#define  COMM_SETTINGS_CONF1            0xc3
#define  COMM_SETTINGS_CONF2            0x55
#define  RPLY_COMM_SETTINGS_LEN         10

#define CMD_STOP_CONT			0xfa
#define  CMD_STOP_CONT_LEN		3
#define  STOP_CONT_CONF1		0x75
#define  STOP_CONT_CONF2		0xb4

namespace hermes_communication_tools {

class ImuInterface {
public:
    ImuInterface();
    virtual ~ImuInterface();

    bool initializeInSeparateThread();
    bool readDataThreadSafe(double* accel, double* angrate, double& timer);
    bool initialize();
    bool readData(double* accel, double* angrate);

    struct{
      char* keyword_;
      int priority_;
      int stacksize_;
      int cpu_id_;
      int delay_ns_;
    } imu_comm_xeno_info_;

private:
    int fd_;
    ssize_t res_;
    uint8_t buffer_[100];
    static const int max_realign_trials_ = 2;
    static const uint8_t cmd_streamed_ = CMD_AC_AN;
    static const size_t cmd_streamed_len_ = RPLY_AC_AN_LEN;
    struct {
    	double lin_acc_[3];
    	double ang_vel_[3];
    	double timer_;
    } imu_msg_;
    boost::shared_ptr<boost::thread> imu_comm_thread_;
    sl_rt_mutex mutex_;
    bool stop_imu_comm_;

    bool openPort();
    bool configureIMU();
    bool streamData();
    bool stopStream();
    bool stopImuComm();
    bool readMissalignedMsgFromDevice();
    bool writeToDevice(int len);
    bool readFromDevice(uint8_t command, int len);
    static inline unsigned short bswap_16(unsigned short x);
    static inline unsigned int bswap_32(unsigned int x);
    static bool isChecksumCorrect(uint8_t* rep, int rep_len);
};
}

#endif
