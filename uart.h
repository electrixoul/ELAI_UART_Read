
#ifndef _UART_H
#define _UART_H

#include <stdio.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART
#include <string>
#include <iostream>

// Define Constants
const char *uart_target = "/dev/ttyUSB0";
#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     BAUDRATE       B115200

class Uart {
private:
  /* data */
  int fid;
public:
  unsigned char serial_message[NSERIAL_CHAR];


  Uart ();
  void sendUart(unsigned char *msg);
  void readUart();
  void closeUart();

};

Uart :: Uart (){
    int ii, jj, kk;

    // SETUP SERIAL WORLD

    struct termios  port_options;   // Create the structure

    tcgetattr(fid, &port_options);	// Get the current attributes of the Serial port


    //------------------------------------------------
    //  OPEN THE UART
    //------------------------------------------------
    // The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR   - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //	    O_NDELAY / O_NONBLOCK (same function)
    //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //				   immediately with a failure status if the output can't be written immediately.
    //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
    //	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

    fid = open(uart_target, O_RDWR | O_NOCTTY );

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(1000000);  // 1 sec delay

    if (fid == -1)
    {
//        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //------------------------------------------------
    // CONFIGURE THE UART
    //------------------------------------------------
    // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
    //	Baud rate:
    //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
    //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000,
    //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE: - CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD  - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)

    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size
    port_options.c_cflag |=  CS8;               // Set the data bits = 8
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
    port_options.c_oflag &= ~OPOST;                           // No Output Processing

    port_options.c_lflag = 0;               //  enable raw input instead of canonical,

    port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VTIME] = 0;           // Wait indefinetly

    cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed
    cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed

    // Set the attributes to the termios structure
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0 )
    {
        printf("\nERROR in Setting port attributes");
    }
    else
    {
        printf("\nSERIAL Port Good to Go.\n");
    }

    // Flush Buffers
    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(500000);   // 0.5 sec delay


}

void Uart :: sendUart(unsigned char *msg){

    tcflush(fid, TCIOFLUSH);

    //--------------------------------------------------------------
    // TRANSMITTING BYTES
    //--------------------------------------------------------------
    unsigned char tx_buffer[20];
    unsigned char *p_tx_buffer;

    p_tx_buffer = &tx_buffer[0];

    // so that i have the number of bytes to write
    // by doing p_tx - tx
    for (int i = 0; i < 20; i++) {
        *p_tx_buffer++ = msg[i];
    }
    //printf("%x%x%x%x%x\n",p_tx_buffer[0],p_tx_buffer[1],p_tx_buffer[2],p_tx_buffer[3],p_tx_buffer[4]);
//    printf("fid 1=%d\n", fid );

    if (fid != -1)
    {
        int count = write(fid, &tx_buffer[0], 20);		//Filestream, bytes to write, number of bytes to write

        usleep(1000);   // .001 sec delay

//        printf("Count = %d\n", count);

//        if (count < 0)  printf("UART TX error\n");
    }

    usleep(1000);  // 1 sec delay


}

void Uart :: readUart(){

    //--------------------------------------------------------------
    // RECEIVING BYTES - AND BUILD MESSAGE RECEIVED
    //--------------------------------------------------------------
    unsigned char rx_buffer[VMINX];
    bool          pickup = true;
    int ii;
    int           rx_length;
    int           nread = 0;


    // TODO
    usleep(1000);   // .001 sec delay

//    printf("Ready to receive message.\n");

    for (ii=0; ii<NSERIAL_CHAR; ii++)  serial_message[ii]=' ';


    while (pickup && fid != -1)
    {
        usleep(50);
        nread++;

        rx_length = read(fid, (void*)rx_buffer, VMINX);   // Filestream, buffer to store in, number of bytes to read (max)

//        printf("Event %d, rx_length=%d, Read=%s\n",  nread, rx_length, rx_buffer );

        if (rx_length < 0)
        {
            //An error occured (will occur if there are no bytes)
        }

        if (rx_length == 0)
        {
            //No data waiting
        }

        if (rx_length>=0)
        {
            if (nread<=NSERIAL_CHAR){
                serial_message[nread-1] = rx_buffer[0];   // Build message 1 character at a time
                //   printf("%x ",serial_message[nread-1]);
            }

            if (rx_buffer[0]=='#')   pickup=false;                               // # symbol is terminator
        }
    }

    printf("\nMessage Received:");
    tcflush(fid, TCIOFLUSH);

}

void Uart :: closeUart(){
    //-------------------------------------------
    //  CLOSE THE SERIAL PORT
    //-------------------------------------------
    close(fid);
}



union OutputPack{

    /*
        unsigned short int left_up_x;
        unsigned short int left_up_y;
        unsigned short int roi_width;
        unsigned short int roi_height;
        unsigned short int machine_state;
    */
    unsigned short int data_field[5];
    unsigned char pack_field[10];

};

union InputPack{

    /*
        unsigned short int compact_state;
        unsigned short int left_up_x;
        unsigned short int left_up_y;
        unsigned short int roi_width;
        unsigned short int roi_height;
    */
    unsigned short int data_field[5];
    unsigned char pack_field[10];
};


class ModeSetting{
public:
    ModeSetting()
    {

    }
public:
    bool NaiveMode = true;
    bool NaiveMode_ClickSelection = true;
    bool ped_or_car = 0;
    bool NoRecover = true;
    bool centerSelection = true;
    bool reset = false;
    bool trigger = false;
    bool power_off = false;

    int left_up_x = 0;
    int left_up_y = 0;
    int roi_width = 0;
    int roi_height = 0;

    InputPack IP;

public:
    void encode()
    {
        // 10000000
        unsigned char h_NaiveMode = true;
        // 01000000
        unsigned char h_NaiveMode_ClickSelection = false;
        // 00100000
        unsigned char h_ped_or_car = 0;
        // 00010000
        unsigned char h_NoRecover = false;
        // 00001000
        unsigned char h_centerSelection = false;
        // 00000100
        unsigned char h_reset = false;
        // 00000010
        unsigned char h_trigger = false;
        // 00000001
        unsigned char h_power_off = false;

        if(NaiveMode)
            h_NaiveMode = 0x80;
        else
            h_NaiveMode = 0x00;

        if(NaiveMode_ClickSelection)
            h_NaiveMode_ClickSelection = 0x40;
        else
            h_NaiveMode_ClickSelection = 0x00;

        if(ped_or_car)
            h_ped_or_car = 0x20;
        else
            h_ped_or_car = 0x00;

        if(NoRecover)
            h_NoRecover = 0x10;
        else
            h_NoRecover = 0x00;

        if(centerSelection)
            h_centerSelection = 0x08;
        else
            h_centerSelection = 0x00;

        if(reset)
            h_reset = 0x04;
        else
            h_reset = 0x00;

        if(trigger)
            h_trigger = 0x02;
        else
            h_trigger = 0x00;

        if(power_off)
            h_power_off = 0x01;
        else
            h_power_off = 0x00;

        unsigned char command_c = ( h_NaiveMode | h_NaiveMode_ClickSelection | h_ped_or_car | h_NoRecover | h_centerSelection | h_reset | h_trigger | h_power_off );

        IP.data_field[0] = command_c;
        IP.data_field[1] = left_up_x;
        IP.data_field[2] = left_up_y;
        IP.data_field[3] = roi_width;
        IP.data_field[4] = roi_height;

    }
    void decode()
    {
        unsigned short int command_i = IP.data_field[0];
        unsigned char command_c = command_i;

        if( (command_c & 0x80) == 0x00)
            NaiveMode = false;
        else
            NaiveMode = true;

        if( (command_c & 0x40) == 0x00)
            NaiveMode_ClickSelection = false;
        else
            NaiveMode_ClickSelection = true;

        if( (command_c & 0x20) == 0x00)
            ped_or_car = false;
        else
            ped_or_car = true;

        if( (command_c & 0x10) == 0x00)
            NoRecover = false;
        else
            NoRecover = true;

        if( (command_c & 0x08) == 0x00)
            centerSelection = false;
        else
            centerSelection = true;

        if( (command_c & 0x04) == 0x00)
            reset = false;
        else
            reset = true;

        if( (command_c & 0x02) == 0x00)
            trigger = false;
        else
            trigger = true;

        if( (command_c & 0x01) == 0x00)
            power_off = false;
        else
            power_off = true;

        left_up_x = IP.data_field[1];
        left_up_y = IP.data_field[2];
        roi_width = IP.data_field[3];
        roi_height = IP.data_field[4];
    }

    void set_input(unsigned char* serial_command)
    {
        for(int i=0;i<10;i++)
        {
            IP.pack_field[i] = serial_command[i];
        }
    }

    void reset_command()
    {
        for(int i=0;i<10;i++)
        {
            IP.pack_field[i] = 0;
        }
    }
};

#endif
