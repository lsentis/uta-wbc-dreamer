#ifndef SHARE_DATA_H
#define SHARE_DATA_H

#include <semaphore.h>

#define BUF_LEN	7

namespace wbc_m3_ctrl {

/* share buffer */
struct shareData{
	double 	num[BUF_LEN];
};

struct shmStruct{
	/* mutex for sync */
	sem_t 	mutex_send;
	sem_t 	mutex_recv;
	
	struct shareData sendBuf;
	struct shareData recvBuf;
	
};

}
#endif
