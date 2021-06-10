#include <robot_sh_memory/sem_common.h>
#include <ros/ros.h>

namespace sem_common
{
int create_semaphore(key_t key)
{
    int sem_id;

    sem_id = semget(key, 1, IPC_CREAT | 0666);
    if (sem_id == -1)
    {
        ROS_ERROR("Create semaphore failed.");
		return SEM_STATE_NO;
    }

	sem_un sem;
    sem.val = 1;
    if  (semctl(sem_id, 0, SETVAL, sem) == -1)
    {
        ROS_ERROR("Init semaphore failed.");
        return SEM_STATE_NO;
    }

    return sem_id;
}

int delete_semaphore(int sem_id)
{
	sem_un sem;

    if (semctl(sem_id, 0, IPC_RMID, sem) == -1)
    {
        ROS_ERROR("Delete semaphore failed.");
        return SEM_STATE_NO;
    }

    return SEM_STATE_OK;
}

int semaphore_p(int sem_id)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = -1;
	sem_b.sem_flg = SEM_UNDO;
	if (semop(sem_id, &sem_b, 1) == -1)
	{
		ROS_ERROR("Get semaphore failed.");
		return SEM_STATE_NO;
	}

    return SEM_STATE_OK;
}

int semaphore_v(int sem_id)
{
	struct sembuf sem_b;
	sem_b.sem_num = 0;
	sem_b.sem_op = 1;
	sem_b.sem_flg = SEM_UNDO;
	if (semop(sem_id, &sem_b, 1) == -1)
	{
		ROS_ERROR("Release semaphore failed.");
		return SEM_STATE_NO;
	}

	return SEM_STATE_OK;
}

}
