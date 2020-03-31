#include <linux/platform_device.h>

#ifndef __SAMPLE_PLATFORM_H__
#define __SAMPLE_PLATFORM_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include<linux/init.h>
#include<linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>


int MAX_DEVICES = 0;
module_param(MAX_DEVICES, int, 0); //Pass this to specify number of devices

#define DRIVER_NAME		"hcsr"
#define CLASS_NAME "HCSR"

#define CONFIG_PINS 11
#define SET_PARAMETERS 12
#define FIFO_SIZE 5

//int valid_trigger_pins[4] = {0, 1, 10, 12}; //IO pins are restricted to IO8 IO15
int invalid_echo_pins[8] = {7, 8, 14, 15, 16, 17, 18, 19}; //These pins don't have interrupt for "both" edges

int gpio_mapping_array[20][4] = {
			                                {11,32,-1,-1},
							{12,28,45,-1},
							{13,34,77,-1},
 							{14,16,76,64}, //14, 16, 76, 64
							{6,36,-1,-1},
							{0,18,66,-1},
							{1,20,68,-1},
							{38,-1,-1,-1},
							{40,-1,-1,-1},
							{4,22,70,-1},
							{10,26,74,-1},
							{5,24,44,72},
							{15,42,-1,-1},
							{7,30,46,-1},
							{48,-1,-1,-1},
							{50,-1,-1,-1},
							{52,-1,-1,-1},
							{54,-1,-1,-1},
							{56,-1,60,78},
							{58,-1,60,79} //7, 30, 60, 79
};

// Array containing the gpio pin configuration
int gpio_values_array[20][4] = {
			                               {0,0,-1,-1},
							{0,0,0,-1},
							{0,1,0,-1},
 							{0,1,0,0},
							{0,0,-1,-1},
							{0,0,0,-1},
							{0,0,0,-1},
							{0,-1,-1,-1},
							{0,-1,-1,-1},
							{0,0,0,-1},
							{0,0,0,-1},
							{0,0,0,0},
							{0,0,-1,-1},
							{0,0,0,-1},
							{0,-1,-1,-1},
							{0,-1,-1,-1},
							{0,-1,-1,-1},
							{0,-1,-1,-1},
							{0,-1,1,1},
							{0,-1,1,1}
};

#define ROW_SIZE(arr) ((int) (sizeof (arr) / sizeof (arr)[0]))
#define COLUMN_SIZE(arr) ((int) sizeof ((arr)[0]) / sizeof (int))

struct hcsrdata {
	bool is_valid;
	uint64_t data;
	uint64_t timestamp;
};

/* per device structure */
struct kbuf_dev {
	bool is_measuring;
	bool is_gpio_requested;
	bool is_irq_requested;
	bool irq_type_flag;
	unsigned int trigger_pin;
	unsigned int echo_pin;
	unsigned int m_samples;
	unsigned int delta;
	unsigned int trigger_interval;
	unsigned int current_sample_idx;
	unsigned int irq_count;
	unsigned int current_fifo_write_idx;
	unsigned int current_fifo_read_idx;
	unsigned int current_fifo_length;
	unsigned int dev_no;
	unsigned int enable;
	char* name;
	uint64_t time1, time2;
	uint64_t last_distance;
	struct hcsrdata hcsr_data[FIFO_SIZE];
	struct mutex read_lock;
	struct mutex flag_lock;
	struct semaphore signal_lock;
	uint64_t* temp_buf;
	struct task_struct *sleeping_task;
	struct task_struct *kthread_task;
	struct work_struct irq_work;
	struct miscdevice misdev;
	const struct attribute_group *group;
	struct platform_device plf_dev;
	struct device* hcsr_device;
	struct class *kbuf_dev_class;          /* Tie with the device model */
} *kbuf_devp;

//Prototypes
int configure_irq(struct kbuf_dev* kbuf_devp, bool rising);
irqreturn_t irq_handler(int irq, void *dev_id);

#endif /* __SAMPLE_PLATFORM_H__ */