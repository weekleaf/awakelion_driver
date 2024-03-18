//
// Created by bismarck on 12/8/22.
//

#include <iostream>
#include <utility>
#include <sys/ioctl.h>  /* BSD and Linux */
#include <string.h>

#include <ros/ros.h>

#include "robot_driver//serialPort.h"
#include "robot_driver//msg_serialize.h"
#include "robot_driver/msgSerializer/check.h"

using std::cout;
using std::endl;

#define UNPACK_DATA_BUFFSIZE 200           //接收自主控板信息的缓冲区大小
#define PACK_DATA_BUFFSIZE   150            //发送到主控板信息的缓冲区大小
serialPort::serialPort(std::string _name) {
    cout<<"init!"<<endl;
    name = std::move(_name);
    init();
    cout<<"init success!"<<endl;
}

serialPort::~serialPort() {
    if (serial.isOpen()) {
        serial.close();
    }
}

/*判断串口是否初始化成功*/
bool serialPort::init() {
    if (name.empty()) {
        serial.setPort("/dev/ttyACM0"); ///dev/ttyACM0
        serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial.setTimeout(to);
        serial.open();
        cout << "SYSTEM USB NOT DETECTED" << endl;
        if (!serial.isOpen()) {
            serial.setPort("/dev/ttyACM0");  ///dev/ttyACM0
            serial.open();
        }
    } else {
        cout << "usb name is" << name << endl;
        serial.setPort(name);
        serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial.setTimeout(to);
        serial.open();
    }

    //检测串口是否已经打开，并给出提示信息
    if (serial.isOpen()) {
        cout << "Serial Port initialized!" << endl;
        return true;
    } else {
        cout << "Unable to open port " << endl;
        return false;
    }
}

/*数据打包加校验位*/
uint8_t* serialPort::packData(uint16_t cmd_id, uint8_t *p_data, uint16_t len,uint8_t *tx_buf)
{

    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN-1;
    frame_header_t *p_header = (frame_header_t*)tx_buf;

    p_header->sof = UP_REG_ID;
    memcpy(&tx_buf[1], (uint8_t*)&len, 1);
    p_header->seq++;

    memcpy(&tx_buf[HEADER_LEN-1], (uint8_t*)&cmd_id, CMD_LEN);
    append_crc8_check_sum(tx_buf, HEADER_LEN-1);
    memcpy(&tx_buf[HEADER_LEN + CMD_LEN-1], p_data, len);
    append_crc16_check_sum(tx_buf, frame_length);

    return tx_buf;
}

/*对接收数据进行处理*/
void serialPort::pc_data_handler(uint8_t *p_frame, vision_rx_data &pc_send_mesg)
{
  frame_header_t *p_header = (frame_header_t *)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);
  uint8_t data_length = p_frame[1]; 
  unsigned char cmd_id = p_frame[5]; 
  uint8_t *data_addr = p_frame + HEADER_LEN + CMD_LEN - 1;
  memcpy(&pc_send_mesg, data_addr, data_length);
        /*查看数据是否解包成功*/
    // ROS_INFO("pc_send_mesg.robot_pitch[0]=%f",pc_send_mesg.robot_pitch);
    // ROS_INFO("pc_send_mesg.robot_yaw[0]=%f",pc_send_mesg.robot_yaw);
    // ROS_INFO("pc_send_mesg.time_stamp[0]=%f",pc_send_mesg.time_stamp);
  return;
}

/*对数据进行解包*/
void serialPort::unpackData(uint8_t *data, int size)
{
    static uint8_t * protocol_packet =NULL;
    int i = 0;
    int start = 0;
    uint32_t datalen  = 0;
    if(size < 9){
        return;
    }
    for(;;)
    {
        if(start >= size -5) {
            return;
        }
        if(data[start] == 0XA0 ){
            break;
        }
        start++;
    }
    datalen = (uint32_t)data[1+start] + 9;//22  
    protocol_packet = (uint8_t *)malloc(sizeof(uint8_t)*datalen);
    for(i =0; i< (int)datalen ; i++){
        protocol_packet[i] = data[start+i];
                    /*查看原始数据*/
    //    std::cout<<i<<":";  
    //    printf("%X ",protocol_packet[i]);
    }
    if( verify_crc8_check_sum(protocol_packet, sizeof(frame_header_t)-1) !=true ){
        return;
    }
    if(verify_crc16_check_sum(protocol_packet, datalen) !=true){
        return;
    }
    pc_data_handler(protocol_packet,stm2pc_mesg);
    
    free(protocol_packet);
    protocol_packet = NULL;
    start +=datalen;

    if (start < size){
    unpackData(&data[ start], size - start);
    }
}

/*对数据进行读取*/
void serialPort::readData(vision_rx_data &pc_send_mesg)
{
    uint16_t _data_len_write = UNPACK_DATA_BUFFSIZE;
    uint8_t data[UNPACK_DATA_BUFFSIZE]={0};
    uint16_t size = serial.read(data, _data_len_write);
                /*查看原始数据*/
    // for(int a =0; a< (int)_data_len_write ; a++){
    //    std::cout<<"data["<<a<<"]:";  
    //    printf("%X ",data[a]);
    // }
    unpackData(data,size);
    memcpy(&pc_send_mesg, &stm2pc_mesg, sizeof(vision_rx_data));
}

void serialPort::writeData(vision_tx_data &pc_recv_mesg)
{
    uint8_t tx_buf[50] = {0};

    packData(GIMBAL_CTRL_ID, (uint8_t *)&pc_recv_mesg ,sizeof(pc_recv_mesg), tx_buf); //GIMBAL_DATA_ID
        /*查看数据是否成功写入*/
    //    for(int i=0;i<sizeof(tx_buf);i++) printf("tx_buf[%d]=%x ",i,tx_buf[i]);

    serial.write(tx_buf, sizeof(tx_buf)); 
}

