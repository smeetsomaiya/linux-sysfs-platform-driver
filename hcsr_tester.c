#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <signal.h>
#include <pthread.h>
#define CONFIG_PINS 11
#define SET_PARAMETERS 12

struct hcsrdata {
	bool is_valid;
	uint64_t data;
	uint64_t timestamp;
};

int MAX_DEVICES;

struct thread_data {
	int fd;
	int trigger_pin;
	int echo_pin;
	int m_samples;
	int delta;
	int dev_id;
	struct hcsrdata hcsr_data;
};

void* thread_function(void* data) {
	int ret, j;
	struct thread_data* data_struct = data;
	char name[15];
	unsigned long pins_data[2];
	memset(name, '\0', sizeof name);
	snprintf(name, 15, "/dev/hcsr_%d",data_struct->dev_id);
	
	data_struct->fd = open(name, O_RDWR);
	if(data_struct->fd < 0) {
		printf("Open failed dev=%s fd=%d errno=%d\n", name, data_struct->fd, errno);
		return NULL;
	}
	memset(name, '\0', sizeof name);
	snprintf(name, 15, "hcsr_%d",data_struct->dev_id);
	
	pins_data[0] = data_struct->trigger_pin;//rand() % 19;
	pins_data[1] = data_struct->echo_pin;//rand() % 19;
	printf("Setting pins dev=%s IO%lu IO%lu\n", name, pins_data[0], pins_data[1]);

	ret = ioctl(data_struct->fd, CONFIG_PINS, pins_data);
	if(ret < 0) {
		printf("Ioctl failed dev=%s ret=%d errno=%d\n", name, ret, errno);
		return NULL;
	}

	pins_data[0] = data_struct->m_samples;//rand() % 19;
	pins_data[1] = data_struct->delta;//rand() % 19;
	printf("Setting parameters dev=%s M %lu Delta %lu\n", name, pins_data[0], pins_data[1]);

	ret = ioctl(data_struct->fd, SET_PARAMETERS, pins_data);
	if(ret < 0) {
		printf("Ioctl failed dev=%s ret=%d errno=%d\n", name, ret, errno);
		return NULL;
	}

	//Try different scenarios like accessing devices concurrently/parallely
	int dataVal = 1; //Clears the fifo
	printf("Write: dev=%s data=%d\n", name, dataVal);
	ret = write(data_struct->fd, &dataVal, 1);
	if(ret < 0) {
		printf("Write failed dev=%s ret=%d errno=%d\n", name, ret, errno);
	}
	sleep(((data_struct->delta) / 1000) + 1);

	printf("Read: dev=%s\n", name);
	for(j = 0; j < 3; j++) {
		dataVal = 0; //rand() % 1;
		ret = write(data_struct->fd, &dataVal, 1); //Trigger yet another measurement
		if(ret < 0) {
			printf("Write failed dev=%s (Measurement may be ongoing) i=%d ret=%d errno=%d\n", name, data_struct->dev_id, ret, errno);
		}
		sleep(((data_struct->delta) / 1000) + 1);
	}

	for(j = 0; j < 4; j++) {
		ret = read(data_struct->fd, &data_struct->hcsr_data, sizeof(struct hcsrdata)); //Read out the data
		if(ret < 0) {
			printf("Read failed dev=%s ret=%d errno=%d\n", name, ret, errno);
		} else {
			printf("Read value distance_cm=%llu tsc=%llu dev=%s\n", data_struct->hcsr_data.data, data_struct->hcsr_data.timestamp, name);
		}
	}

	printf("Close: dev=%s\n", name);
	ret = close(data_struct->fd);
	if(ret < 0) {
		printf("Close failed i=%d ret=%d errno=%d\n", data_struct->dev_id, ret, errno);
	}
	return 0;
}


int main(int argv, char** argc) {
	int i, arg_idx = 2;
	MAX_DEVICES = atoi(argc[1]);
	printf("Number of devices are %d\n", MAX_DEVICES);
	
	struct thread_data* data_struct = (struct thread_data*) malloc(sizeof(struct thread_data) * MAX_DEVICES);
	
	pthread_t* threads = (pthread_t*) malloc(MAX_DEVICES* sizeof(pthread_t));
	
	for(i = 0; i < MAX_DEVICES; i++) {
		data_struct[i].dev_id = i;
		data_struct[i].trigger_pin = atoi(argc[arg_idx++]);
		data_struct[i].echo_pin = atoi(argc[arg_idx++]);
		data_struct[i].m_samples = atoi(argc[arg_idx++]);
		data_struct[i].delta = atoi(argc[arg_idx++]);
				
		pthread_create(&threads[i], NULL, thread_function, &data_struct[i]);
	}
	for(i = 0; i < MAX_DEVICES; i++) {
		pthread_join(threads[i], NULL);
	}
	free(data_struct);
	free(threads);
	return 0;
}