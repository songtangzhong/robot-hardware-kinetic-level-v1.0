#ifndef SEM_COMMON_H_
#define SEM_COMMON_H_

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/sem.h>
#include <iostream>

#define SEM_STATE_OK  1
#define SEM_STATE_NO -1

namespace sem_common
{
typedef union 
{
	int val;
	struct semid_ds *buf;
	unsigned short *arry;
} sem_un;

int create_semaphore(key_t key);

int delete_semaphore(int sem_id);

int semaphore_p(int sem_id);

int semaphore_v(int sem_id);

}

#endif