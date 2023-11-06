#include "serialPort/SerialPort.h"
#include <csignal>
#include "math.h"
#define Orgin_angle1 6+3.14159*9
#define Orgin_angle2 2-3.14159*9
#define Length 150
#define l1 155
#define l2 280
double Cos(double a,double b,double c)
{
    return (a*a+b*b-c*c)/(2.0*a*b);
}
double calc_left(double x,double y)
{
    return acos(Cos(l1,sqrt(y*y+(Length/2.0+x)*(Length/2.0+x)),l2))+atan2(y,Length/2.0+x);
}
double calc_right(double x,double y)
{
    return acos(Cos(l1,sqrt(y*y+(Length/2.0-x)*(Length/2.0-x)),l2))+atan2(y,Length/2.0-x);
}

int main(){
    // set the serial port name
    SerialPort serial("/dev/ttyUSB0");

    // send message struct
    MOTOR_send motor_run1,motor_run2, motor_stop;
    // receive message struct
    MOTOR_recv motor_r1,motor_r2;

    // set the id of motor
    motor_run1.id = 0;
    // set the motor type, A1 or B1
    motor_run1.motorType = MotorType::A1;
    motor_run1.mode = 10;
    motor_run1.T = 0.0;
    motor_run1.W = 0.0;
    motor_run1.Pos = 0.0;
    motor_run1.K_P = 0.1;
    motor_run1.K_W = 0.0;
    motor_run2.id = 1;
    motor_run2.motorType = MotorType::A1;
    motor_run2.mode = 10;
    motor_run2.T = 0.0;
    motor_run2.W = 0.0;
    motor_run2.Pos = 15.0;
    motor_run2.K_P = 0.1;
    motor_run2.K_W = 0.0;


    motor_r1.motorType = motor_run1.motorType;
    motor_r2.motorType = motor_run2.motorType;
    // encode data into motor commands
    modify_data(&motor_run1);
    modify_data(&motor_run2);
    double angle=0,r=20,x,y;
    
    // turn for 3 second
    for(int i=1;i<=10000;i++) 
    {
        angle+=3.14159/180;
        x=sin(angle)*50;y=cos(angle)*50+250;
        std::cout<<calc_left(x,y)<<" "<<calc_right(x,y)<<std::endl;
        motor_run1.Pos=Orgin_angle1-(float)calc_left(x,y)*9;
        motor_run2.Pos=Orgin_angle2+(float)calc_right(x,y)*9;
        std::cout << "Pos1: " << motor_run1.Pos << std::endl;
        std::cout << "Pos2: " << motor_run2.Pos << std::endl<<std::endl;
        serial.sendRecv(&motor_run1, &motor_r1);
        serial.sendRecv(&motor_run2, &motor_r2);
        extract_data(&motor_r1);extract_data(&motor_r2);
        usleep(1000);
    }
    motor_run1.mode=0;motor_run2.mode=0;
    serial.sendRecv(&motor_run1, &motor_r1);
    serial.sendRecv(&motor_run2, &motor_r2);
    // stop the motor

    return 0;
}
