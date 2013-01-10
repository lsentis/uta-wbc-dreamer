#ifdef __cplusplus
extern "C" {
#endif

#define CMD_CALIBRATE				0x01000000
#define CMD_SET_TORQUE				0x02000000
#define CMD_SET_MODE				0x03000000
#define CMD_SET_MODE_RAW_TORQUE		0x03000001
#define CMD_SET_MODE_COMP_TORQUE	0x03000002

#define CMD_DATA_LEN 10

#define NOTIFY_ADDR             "224.0.0.100"
#define NOTIFY_PORT             51123
#define CMD_PORT                51124
#define STT_PORT                51125
typedef struct 
{
	int			index;
	int			count;
	long long	timeStamp;
	double		data[1];
} message;

typedef struct
{
	int			command;
	int			data_len;
	long		buf[CMD_DATA_LEN];
    char        exp[CMD_DATA_LEN];

	// internal use
	char		received_check;
} command;

extern void *receive_udp(void *arg);
extern void *notify_thread(void *arg);
extern void send_udp(message *pBuf);
extern message *allocMessage(int count);

#ifdef __cplusplus
}
#endif
