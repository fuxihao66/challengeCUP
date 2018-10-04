#ifndef CLIENT_H
#define CLIENT_H
#include <winsock2.h> 
#include <pcl/io/boost.h>
#include <QWidget>
#pragma comment(lib,"ws2_32.lib")  
class Client : public QWidget
{
	Q_OBJECT
private:
	void connectToHost();
public:
	Client();
	~Client();
	boost::signals2::connection registerCallback(const boost::function<void()> & callback);
	void start();
	SOCKET sclient;
	sockaddr_in sin;
	int len;
protected:
	boost::signals2::signal<void()> * cloudSignal;
};

inline int bytesToInt(char * stream, int size = 4) {
	int addr = stream[0] & 0xFF;
	addr |= ((stream[1] << 8) & 0xFF00);
	addr |= ((stream[2] << 16) & 0xFF0000);
	addr |= ((stream[3] << 24) & 0xFF000000);
	return addr;
}

#endif // CLIENT_H