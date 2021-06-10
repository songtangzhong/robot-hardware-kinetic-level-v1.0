#ifndef SHM_COMMON_H_
#define SHM_COMMON_H_

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/shm.h>

#define SHM_STATE_OK  1
#define SHM_STATE_NO -1

namespace shm_common
{
template <class T>
int create_shm(key_t key, T ** shm_ptr);

template <class T>
int release_shm(int shm_id, T ** shm_ptr);

}

#endif