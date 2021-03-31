#ifndef __ANO_TECH_H_
#define  __ANO_TECH_H_

typedef unsigned char u8;
typedef int s16;

void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun,u8*data,u8 len);
//void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
//void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 data_7,s16 data_8,s16 data_9);


u8 data_to_send[50];
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )

void usart1_send_char(u8 c)
{
  Serial.write(c);
}

void usart1_niming_report(u8 fun, u8*data, u8 len)
{
  u8 send_buf[32];
  u8 i;
  if (len>28)return;  //最多28字节数据 
  send_buf[len + 3] = 0;  //校验数置零
  send_buf[0] = 0X88; //帧头
  send_buf[1] = fun;  //功能字
  send_buf[2] = len;  //数据长度
  for (i = 0;i<len;i++)send_buf[3 + i] = data[i];     //复制数据
  for (i = 0;i<len + 3;i++)send_buf[len + 3] += send_buf[i];  //计算校验和 
  for (i = 0;i<len + 4;i++)usart1_send_char(send_buf[i]); //发送数据到串口1 
}

void ANO_DT_Send_Data(u8 *data, u8 len)
{
  u8 i;
  if (len>28)return;  //最多28字节数据 
  for (i = 0;i<len;i++)usart1_send_char(data[i]); //发送数据到串口1
}

void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 data_7,s16 data_8,s16 data_9)
{
  u8 _cnt=0;
  s16 _temp;
  u8 sum = 0;
  u8 i;
  
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x02;
  data_to_send[_cnt++]=0;
  
  _temp = a_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_z;  
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = g_x;  
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_y;  
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_z;  
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = data_7; 
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = data_8; 
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = data_9; 
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  
  
  data_to_send[3] = _cnt-4;
  
  
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++] = sum;
  
  ANO_DT_Send_Data(data_to_send, _cnt);
}


#endif
