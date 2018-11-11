#include "Messenger.h"
#include "termios.h"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <thread> // multithread
#include <chrono> //high-precision clock;
#include <mutex> // thread protect
#include <string.h>

using namespace std;

//------------Initializer & Destructor-------------//
//Default initializer, open ttySAC0 with READ&WRITE. Baudrate=115200

Messenger::Messenger(void)
{
	strcpy(this->port_name,PORT_NAME);
	this->flag = FLAG;	
}

Messenger::Messenger(const char* port_name,int flag)
{	
	strcpy(this->port_name,port_name);
	this->flag = flag;	
}

Messenger::~Messenger()
{
	IsFinished = true;
	for(auto& t: th)
	{
		if(t.joinable())
			t.join();
	}
	cout << "Thread terminated." << endl;
}

void Messenger::Initiate(void)
{ 
	using std::cout;
	fd = open(port_name,flag);
	cout << "fd = " << fd << endl;
	if(fd==-1)
	{
		cout << "Error in opening the port.\n";
	}
	else
	{
		cout << "Port " << port_name << " successfully opened" << endl;
	}
	_serialSetup();
	// setup two thread for input and output
	IsFinished = false;
	tx_counter = 0;
	rx_counter = 0;
	th.push_back(std::thread(&Messenger::ThrRx, this));
	th.push_back(std::thread(&Messenger::ThrTx, this));
}

//------------private functions-------------//
void Messenger::_serialSetup(void)
{
	using std::cout; 
	if(!isatty(fd)) { cout <<"... error handling ..."<<endl; }
	if(tcgetattr(fd, &config) < 0) { cout <<"... error handling ..."<<endl; }
	 //
	 // Input flags - Turn off input processing
	 //
	 // convert break to null byte, no CR to NL translation,
	 // no NL to CR translation, don't mark parity errors or breaks
	 // no input parity check, don't strip high bit off,
	 // no XON/XOFF software flow control
	 //
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	 //
	 // Output flags - Turn off output processing
	 //
	 // no CR to NL translation, no NL to CR-NL translation,
	 // no NL to CR translation, no column 0 CR suppression,
	 // no Ctrl-D suppression, no fill characters, no case mapping,
	 // no local output processing
	 //
	 // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	 //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	config.c_oflag = 0;

	 //
	 // No line processing
	 //
	 // echo off, echo newline off, canonical mode off, 
	 // extended input processing off, signal chars off
	 //
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	 //
	 // Turn off character processing
	 //
	 // clear current char size mask, no parity checking,
	 // no output processing, force 8 bit input
	 //
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= (CS8);

	 //
	 // One input byte is enough to return from read()
	 // Inter-character timer off
	 //
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 0;

	 //
	 // Communication speed (simple version, using the predefined
	 // constants)
	 //
	if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
	    cout <<"... error handling ..."<<endl;
	}
	 //
	 // Finally, apply the configuration
	 //
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0) { cout <<"... error handling ..."<<endl; }
	cout << "serial set up completed" << endl;
}

// receive function in BLOCKING mode
void Messenger::_receive(int size)
{
	Rx_mtx.lock();
	int tmp = read(fd,rx_buffer,size);
	cout << tmp << " Bytes read" << endl;
	Rx_mtx.unlock();
}

// receive function in NON_BLOCKING mode
void Messenger::_receive(void)
{
	Rx_mtx.lock();	
	int cnt = read(fd,rx_buffer+rx_counter,100);
	if(cnt > 0)
	{
		cout << (int)cnt << " bytes " << " read " << endl;
		rx_counter += cnt;
	}
	Rx_mtx.unlock();
}

//------------public functions-------------//
void Messenger::send(uint8_t * tx,int size)
{
	Tx_mtx.lock();
	for(int i=tx_counter;i<tx_counter+size;i++)
		tx_buffer[i] = tx[i];
	tx_counter += size;
	Tx_mtx.unlock();
}
// return the available number of data in the rx_buffer
int Messenger::available(void)
{
	return rx_counter;
}
// load content into rx directly
void Messenger::retrieve(uint8_t *rx, int size)
{
	Rx_mtx.lock();
	if(rx_counter < size)
	{
		for(int i=0;i<rx_counter;i++)
			rx[i] = rx_buffer[i];
		rx_counter = 0;
	}
	else
	{
		for(int i=rx_counter-size;i<rx_counter;i++)
			rx[i] = rx_buffer[i];
		rx_counter -= size;
	}
	Rx_mtx.unlock();
}


// Basic functions to send array or byte
Messenger & Messenger::operator<<(uint8_t byte)
{
	uint8_t tmp[1] = {byte};
	send(tmp,1);
}

// Interaction between classes, send plan through serial


void Messenger::ThrTx()
{
	cout << "Send thread starts." << endl;
	tcflush(fd,TCOFLUSH);
	while (!IsFinished) // job to be done
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1)); // limit check frequency
		if(tx_counter!=0)
		{
			Tx_mtx.lock();
			// make sure data is set properly before sent. 
			int tmp = write(fd,tx_buffer,tx_counter);
			cout << "Sent " << tmp << " bytes\n";
			// cout << "sending " ;
			// for(int i=0;i<tmp;i++)
			// 	cout << (uint8_t)tx_buffer[i];
			tx_counter -= tmp;
			Tx_mtx.unlock();
		}
	}
	cout << "ThrTx() terminates." << endl;
}

void Messenger::ThrRx()
{      
	cout << "Receive thread starts." << endl;
	tcflush(fd,TCIFLUSH);
	while (!IsFinished) // job to be done
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		if(flag & O_NONBLOCK)	// NON-BLOCKING MODE
		{
			_receive();
			//cout << "In NON_BLOCKING mode..." << endl;
		}
		else
		{
			_receive(RX_FRAMELENGTH); // BLOCKING MODE
			//cout << "In BLOCKING mode..." << endl;
		}
	}
	cout << "ThrRx() terminates." << endl;
}
