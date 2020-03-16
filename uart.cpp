
#include "uart.h"
#include <stdio.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART
#include <string>
#include <iostream>

using namespace std;


ModeSetting MS;

int main(int argc, char *argv[]) {

    /*
     * ELAI 目标跟踪上位机示例程序 : 读取跟踪器数据输出流
     *
     * [ uart.h ] -> 串口配置:
     * fid = open(uart_target, O_RDWR | O_NOCTTY ); // 阻塞式接收模式
     * ...
     * port_options.c_cc[VMIN]  = VMINX; // Read at least 1 character
     * port_options.c_cc[VTIME] = 0;           // Wait indefinetly
     *
     */

    Uart u;
    int i;

    /*
     * 每条发送指令由字节 '@' 开始, '#' 结束
     * 包含起始结束符号一共12字节
     */
    unsigned char m[20] = "@0000000000#";

    OutputPack OP;
    ModeSetting MS;

    bool exit_ = false;

    system("clear");

    while (!exit_)
    {
        /// 编码一个空指令并且发送 (使用空指令进行数据查询)
        MS.reset_command();
        MS.encode();

        for(int i=0;i<10;i++)m[i+1] = MS.IP.pack_field[i];
        m[0] = '@';
        m[11] = '#';

        usleep(1000);

        u.sendUart(m);
        cout<<"================= 指令已发送 ================="<<endl;

        /*
         * 每条接收指令由字节数据位开始, '#' 结束
         * 包含起始结束符号一共11字节
         */
        u.readUart();
        for(int i=0;i<10 && u.serial_message[i]!='#';i++) {
            OP.pack_field[i] = u.serial_message[i];
        }

        /*
         * 一般输出以及物理意义:
         * Message Received:x  y  width  height  1
         * - 其中, (x,y)代表目标在画面中的包围框的左上角坐标, (width,height)代表包围框的尺寸, 最后一位0/1分别代表跟踪器的[丢失/锁定]状态
         * - 我们使用图像左上角为原点的坐标系, 水平向右为X轴正方向, 垂直向下为Y轴正方向
         * 例如:
         * Message Received:288  218  100  100  1
         */

        for(int i=0;i<5;i++) {
            cout<<OP.data_field[i]<<"  ";
        }
        cout<<endl<<"================= 数据已接收 ================="<<endl;
    }

    u.closeUart();
    return 0;
}






