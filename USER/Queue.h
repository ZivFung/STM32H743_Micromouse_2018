#ifndef QUEUE_H_
#define QUEUE_H_

#include  <includes.h>

class Queue
{
	private:
		volatile uint32_t 	head;
		volatile uint32_t 	tail;
		volatile uint8_t * 	QueueAddr;
		volatile uint32_t 	QueueSize;
		volatile uint8_t 	CellSize;
		volatile uint32_t	QueueLength;
	public:
		Queue(uint8_t *QueueAddr, uint32_t QueueSize, uint8_t CellSize);
		void Config(uint8_t *QueueAddr, uint32_t QueueSize, uint8_t CellSize);
		void Clear(void);
		uint32_t Length(void);
		bool Push(uint8_t *data);
		bool Pop(uint8_t *data);
		bool IsFull(void);
};

//#define QUEUELENGTH		2048
//void QueueInit(void);
//void QueueClear(void);
//CPU_INT16U QueueLength(void);
//bool QueuePush(float vl, float al);
//bool QueuePop(float &vl, float &al);

#endif /* QUEUE_H_ */
