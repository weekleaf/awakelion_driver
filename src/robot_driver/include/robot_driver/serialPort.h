//
// Created by bismarck on 12/8/22.
//

#ifndef MSG_SERIALIZE_SERIALPORT_H
#define MSG_SERIALIZE_SERIALPORT_H

#include <thread>
#include <serial/serial.h>

#include "msg_serialize.h"
#include <iostream>
#define UNPACK_DATA_BUFFSIZE 200           //接收自主控板信息的缓冲区大小
#define PACK_DATA_BUFFSIZE   150            //发送到主控板信息的缓冲区大小

class serialPort {
private:
    serial::Serial serial;
    std::thread listenerThread;

    int fd;

    vision_rx_data     stm2pc_mesg;
    game_robot_HP_t    game_robot_HP;
    robot_judge1_data_t robot_judge1_data;
    
    uint8_t tx_buf[PACK_DATA_BUFFSIZE];
    uint8_t buf[UNPACK_DATA_BUFFSIZE];

    int _data_len = sizeof(vision_rx_data); 
    uint8_t* packData(uint16_t cmd_id, uint8_t *p_data, uint16_t len,uint8_t *tx_buf);

public:
    enum Color
    {
        red = 0,
        blue = 1
    };
    Color current_robot_color;
    
    uint8_t enemy_HP_Hero;
    uint8_t enemy_HP_Infansty1;
    uint8_t enemy_HP_Infansty2;
    uint8_t enemy_HP_Sentry;
    uint8_t enemy_HP_Base;
    std::string name;
    serialPort() = default;
    explicit serialPort(std::string _name);
    ~serialPort();

    bool init();
    void pc_data_handler(uint8_t *p_frame, vision_rx_data &msg_data,robot_judge1_data_t &judge1_data,game_robot_HP_t &robot_HP);
    void writeData(vision_tx_data &msg_data);
    void readData(vision_rx_data &msg_data,robot_judge1_data_t &judge1_data,game_robot_HP_t &robot_HP);
    void unpackData(uint8_t *data, int size);
    void JudgeDate_Processing(robot_judge1_data_t &judge1_data,game_robot_HP_t &robot_HP);

    bool reading(unsigned char *msg_data, uint16_t &size);
};

#endif //RM2022ENGINEER_SERIALPORT_H
