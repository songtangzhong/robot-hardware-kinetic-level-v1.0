#include <robot_sh_memory/shm_common.h>
#include <robot_sh_memory/arm_shm.h>
#include <ros/ros.h>

namespace shm_common
{
template <class T>
int create_shm(key_t key, T ** shm_ptr)
{
    void *shm_ln_ptr = NULL;

    T *shm_ptr_;

    int shm_id;

    shm_id = shmget(key, sizeof(T), IPC_CREAT | 0666);
    if (shm_id == -1)
    {
        ROS_ERROR("Create shared memory failed.");
		return SHM_STATE_NO;
    }

    shm_ln_ptr = shmat(shm_id, 0, 0);
    if (shm_ln_ptr == (void*)-1)
    {
        ROS_ERROR("Create link address failed.");
		return SHM_STATE_NO;
    }

    *shm_ptr = shm_ptr_ = (T*)shm_ln_ptr;

	return shm_id;
}

template <class T>
int release_shm(int shm_id, T ** shm_ptr)
{
    if (shmdt(*shm_ptr) == -1)
    {
        ROS_ERROR("Seperate shared memory failed.");
		return SHM_STATE_NO;
    }

    if (shmctl(shm_id, IPC_RMID, 0) == -1)
    {
        ROS_ERROR("Release shared memory failed.");
		return SHM_STATE_NO;
    }

	return SHM_STATE_OK;
}

template int create_shm<arm_shm::Arm>(key_t key, arm_shm::Arm ** shm_ptr);
template int release_shm<arm_shm::Arm>(int shm_id, arm_shm::Arm ** shm_ptr);

}
