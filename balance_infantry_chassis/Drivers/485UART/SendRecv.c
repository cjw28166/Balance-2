#include "SendRecv.h"

uint8_t SendData[34],RecvData[78];
uint8_t array4[4],array2[2];
uint32_t test;
uint32_t crcdata[7];
void modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 34;
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x0;
    motor_s->motor_send_data.Mdata.mode = motor_s->mode;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x0;
    motor_s->motor_send_data.Mdata.reserved = 0x0;
    motor_s->motor_send_data.Mdata.Modify.L = 0;
    motor_s->motor_send_data.Mdata.T = motor_s->T*256;
    motor_s->motor_send_data.Mdata.W = motor_s->W*128;
    motor_s->motor_send_data.Mdata.Pos = (float)((motor_s->Pos/6.2832)*16384.0);
    motor_s->motor_send_data.Mdata.K_P = motor_s->K_P*2048;
    motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*1024;
    
    motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
    motor_s->motor_send_data.CRCdata.u32 = crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
}
bool extract_data(MOTOR_recv* motor_r)
{
    if(motor_r->motor_recv_data.CRCdata.u32 !=
        crc32_core((uint32_t*)(&(motor_r->motor_recv_data)), 18)){
        motor_r->correct = false;
        return motor_r->correct;
    }else{
        motor_r->motor_id = motor_r->motor_recv_data.head.motorID;
        motor_r->mode = motor_r->motor_recv_data.Mdata.mode;
        motor_r->Temp = motor_r->motor_recv_data.Mdata.Temp;
        motor_r->MError = motor_r->motor_recv_data.Mdata.MError;
        motor_r->T = (float)(((float)motor_r->motor_recv_data.Mdata.T) / 256);
        motor_r->W = (float)(((float)motor_r->motor_recv_data.Mdata.W) / 128);
        motor_r->LW = motor_r->motor_recv_data.Mdata.LW;

        motor_r->Acc = (int)motor_r->motor_recv_data.Mdata.Acc;
        motor_r->Pos = 6.2832*((float)motor_r->motor_recv_data.Mdata.Pos) / 16384;
        
        motor_r->gyro[0] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[0]) * 0.00107993176);
        motor_r->gyro[1] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[1]) * 0.00107993176);
        motor_r->gyro[2] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[2]) * 0.00107993176);
        
        motor_r->acc[0] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[0]) * 0.0023911132);
        motor_r->acc[1] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[1]) * 0.0023911132);
        motor_r->acc[2] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[2]) * 0.0023911132);

        motor_r->correct = true;
        return motor_r->correct;
    }
}
void DataSend(MOTOR_send *motor)
{
		for(int i=0;i<34;i++)
				SendData[i]=0;
		SendData[0]=0xFE;
		SendData[1]=0xEE;
		SendData[2]=motor->id;
		SendData[4]=motor->mode;
		int16_ttouint8_t(SendData+12,(int16_t)(motor->T*256),true);
		int16_ttouint8_t(SendData+14,(int16_t)(motor->W*128),true);	
		int t=(int)((motor->Pos*16384)/2/3.14159);
		inttouint8_t(SendData+16,t,true);
		int16_ttouint8_t(SendData+20,(int16_t)(motor->K_P*2048),true);
		int16_ttouint8_t(SendData+22,(int16_t)(motor->K_W*1024),true);	

		uint32_t cr = crc32_core((uint32_t*)SendData,7);
		memcpy(&SendData[30], &cr, sizeof(cr));
//		uint32_ttouint8_t(SendData+30,cr,true);
		

		
}
bool DataRecv(MOTOR_recv *motor)
{
		
		
		
		
		uint32_t crcdata[18];
		for(int i=0;i<18;i++)
		{
				crcdata[i]=uint8touint32_t(RecvData+i*4,true);
		}
		uint32_t cr=crc32_core(crcdata,18),cr0;
		cr0=uint8touint32_t(RecvData+74,true);
		if(cr0==cr)
		{
				motor->motor_id=RecvData[2];
				motor->mode=RecvData[4];
				motor->Temp=RecvData[6];
				motor->MError=RecvData[7];
				motor->T=(RecvData[12]|(RecvData[13]<<8))/256.0;
				motor->W=(RecvData[14]|(RecvData[15]<<8))/128.0;
				motor->LW=uint8tofloat(RecvData+16,true);
				motor->Acc=(int)uint8toint16_t(RecvData+26,true);
				motor->Pos=uint8tofloat(RecvData+30,true)*2*3.14159/16384;
				for(int i=0;i<3;i++)
						motor->gyro[i]=(float)uint8toint16_t(RecvData+38+2*i,true)*2000*3.14159/16384.0/360.0;
				for(int i=0;i<3;i++)
						motor->acc[i]=(float)uint8toint16_t(RecvData+44+2*i,true)*9.80665/4096.0;	
				return true;
		}
		return false;
}
