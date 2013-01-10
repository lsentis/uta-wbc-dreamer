#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "comm_udp.h"

#define BUFLEN 1024

struct sockaddr_in si_other;
int slen;

message *allocMessage(int count)
{
	message *pRet;

	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(0xaa55);
	si_other.sin_addr.s_addr = inet_addr("127.0.0.1");
	pRet = (message *)malloc( sizeof(message) + sizeof(double)*(count-1) );
		
	return pRet;
}

void *receive_udp(void *arg)
{
	struct sockaddr_in si_me;
	int s;
	command *cmd = (double *)arg;

	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
		return NULL;

	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(CMD_PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	bind(s, &si_me, (socklen_t*)sizeof(si_me));

	double buf[BUFLEN];
	
	fprintf(stderr, "COMM THREAD started\n");
	while (1)
	{
		recvfrom(s, cmd, sizeof(command), 0, &si_other, (socklen_t*)&slen);
		fprintf(stderr, "PACKET RECEIVE... %d(%d),%d,%d,%d,%d,%d %d\n", cmd->command, cmd->data_len, cmd->buf[0], cmd->buf[1], cmd->buf[2], cmd->buf[3], cmd->buf[4], slen);	
		fprintf(stderr, "ADDR: %08x(%d)\n", si_other.sin_addr.s_addr, slen);
		cmd->received_check = 0;
//		head_command[4] = buf[4] * M_PI / 180.;
	}

	return NULL;
}

int s = 0;
int index = 0;
void send_udp(message *pMsg)
{
	if ( s <= 0 )
	{
		fprintf(stderr, "SOCKET CREATED\n");
		if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
		{
			fprintf(stderr, "FAIL TO OPEN PORT\n");
			return;
		}
	}

	int len;
	struct sockaddr_in si_to;
	memset((char *) &si_to, 0, sizeof(si_to));
	si_to.sin_family = AF_INET;
	si_to.sin_port = htons(STT_PORT);
//	si_to.sin_addr.s_addr = si_other.sin_addr.s_addr;
	si_to.sin_addr.s_addr = inet_addr("127.0.0.1");

//	fprintf(stderr, "ADDR: %08x(%d:%d)\n", si_to.sin_addr.s_addr, slen, sizeof(si_to));
	pMsg->index = index;
	len = sendto(s, pMsg, sizeof(message) + sizeof(double)*(pMsg->count-1), 0, &si_to, sizeof(si_to));
//	fprintf(stderr, "PACKET SENT %d:%d (%d)\n", s, len, pMsg->count);
	++index;
}

void *notify_thread(void *arg)
{
	int len;
	int interval = (int)arg;
	struct sockaddr_in si_to;
	memset((char *) &si_to, 0, sizeof(si_to));
	si_to.sin_family = AF_INET;
	si_to.sin_port = htons(NOTIFY_PORT);
	si_to.sin_addr.s_addr = inet_addr(NOTIFY_ADDR);

	while (1)
	{
		char buf[10];
		len = sendto(s, buf, sizeof(buf), 0, &si_to, sizeof(si_to));
		sleep(interval);
	}
}
