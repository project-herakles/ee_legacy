#ifndef MESSENGER_H_
#define MESSENGER_H_

#include <stdint.h>
#include <iostream>
#include "termios.h"
#include <fcntl.h>
#include <unistd.h>
#include <thread> // multithread
#include <chrono> //high-precision clock;
#include <vector>
#include <mutex>

using namespace std;


#define PORT_NAME  "/dev/ttyUSB0"
#define FLAG O_RDWR

class Messenger
{
private:
	// predefined constants governing serial protocol, see serial protocol for details
	static const uint8_t TX_SOF = 0xCE;
	static const uint8_t RX_SOF = 0xCE;
	static const int RX_FRAMELENGTH = 10;
	static const uint8_t MOVE_CMD = 0x01;
	static const uint8_t GIMBAL_CMD = 0x02;
	static const uint8_t SHOOT_CMD = 0x03;

	static const int HEADER_FRAMELENGTH = 3;
	static const int TX_MOVE_LENGTH = 1;
	static const int TX_GIMBAL_LENGTH = 8;
	static const int TX_SHOOT_LENGTH = 1;

	// intermidiate variables
	int fd; // file descriptor for serial
	int flag; // flags for serial
	struct termios config;
	int counter; // count the number of bytes to send
	uint8_t tx_buffer[128];
	uint8_t rx_buffer[128];
	int tx_counter;
	int rx_counter;
	// Thread-related variables
	bool SendFlag; // true for main thread wants to send data
	bool IsFinished;// true for main thread wants to terminate thread
	std::vector<std::thread> th; // thread; 
	std::mutex Tx_mtx;           // mutex for critical section
	std::mutex Rx_mtx;
	char port_name[128];

	// private functions
	void _serialSetup(void); // setup serial with baudrate 115200
	// Thread-related functions
	void ThrTx();
	void ThrRx();
	void _receive(int size);
	void _receive(void);
public:
	Messenger(void);
	Messenger(const char* port_name,int flag);
	~Messenger();
	void Initiate(void);
	// Basic functions to send or receive
	void send(uint8_t * tx,int size);
	int available(void);
	void retrieve(uint8_t *rx,int size);
	Messenger & operator<<(uint8_t byte);
};

#endif
