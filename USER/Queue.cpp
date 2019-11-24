#include "Queue.h"

Queue::Queue(uint8_t *Addr, uint32_t QSize, uint8_t CSize)
{
	head = 0;
	tail = 0;	
	QueueAddr = Addr;
	QueueSize = QSize;
	CellSize = CSize;
	QueueLength = 0;
}
void Queue::Config(uint8_t *Addr, uint32_t QSize, uint8_t CSize)
{
	head = 0;
	tail = 0;	
	QueueAddr = Addr;
	QueueSize = QSize;
	CellSize = CSize;
	QueueLength = 0;	
}
void Queue::Clear(void)
{
	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();
	head = 0;
	tail = 0;
	QueueLength = 0;
	OS_CRITICAL_EXIT();	
}
uint32_t Queue::Length(void)
{
	return QueueLength;
}
bool Queue::Push(uint8_t *data)
{
	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();

	if(QueueLength < QueueSize)
	{
		for(int i = 0; i < CellSize; i++)
			*(QueueAddr + tail*CellSize + i) = *(data+i);
		if(tail == QueueSize - 1)
			tail = 0;
		else
			tail ++;
//		if(tail >= head)
//			QueueLength = tail - head;
//		else
//			QueueLength = QueueSize + tail - head;
		QueueLength++;
		OS_CRITICAL_EXIT();
		return true;
	}
	else
	{
		OS_CRITICAL_EXIT();
		return false;
	}	
}
bool Queue::Pop(uint8_t *data)
{
	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();	
	if(QueueLength > 0)
	{
		for(int i = 0; i < CellSize; i++)
			*(data+i) = *(QueueAddr + head*CellSize + i);
		if(head == QueueSize - 1)
		{
			head = 0;
		}
		else
		{
			head++;
		}
//		if(tail >= head)
//		{
//			QueueLength = tail - head;
//		}
//		else
//		{
//			QueueLength = QueueSize + tail - head;
//		}
		QueueLength--;
		OS_CRITICAL_EXIT();
		return true;
	}
	else
	{
		OS_CRITICAL_EXIT();
		return false;
	}	
}

bool Queue::IsFull(void)
{
	if(QueueLength < QueueSize)
		return false;
	else
		return true;
}
