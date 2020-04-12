/*
 * A sample program to show the binding of platform driver and device.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "hcsr_platform_device.h"

static inline uint64_t __attribute__((__always_inline__))
rdtsc(void)
{
    uint32_t a, d;
    __asm __volatile("rdtsc" : "=a" (a), "=d" (d));
    return ((uint64_t) a) | (((uint64_t) d) << 32);
}

int is_gpio_64_to_79(int gpio) { //Check for gpio's which do not have a direction file
	if(gpio >= 64 && gpio <= 79) {
		return 1; //return true
	}
	return 0; //return false
}

int mux_gpio_set(struct kbuf_dev* kbuf_devp, int gpio, int value, bool out_dir, int column)
{
	if(gpio == -1 || value == -1) {
		printk("-1 found in mux_gpio_set\n");
		return -1;
	}
	printk(KERN_ALERT "mux_gpio_set: dev=%s gpio_request gpio%d\n", kbuf_devp->name, gpio);
	gpio_request(gpio, "sysfs");

	if(column == 0) { //Actual pins
		if(out_dir)
			gpio_direction_output(gpio, value);
		else
			gpio_direction_input(gpio); //for input pins
		
		printk(KERN_ALERT "mux_gpio_set: dev=%s gpio_direction_op/ip %d gpio %d value %d\n", kbuf_devp->name, out_dir, gpio, value);
	} else if(column == 1) {
		if(!out_dir) {
			gpio_direction_output(gpio, 1); //Set level shifter High for input pin
		} else {
			gpio_direction_output(gpio, 0); //Set level shifter low for output pinn
		}
	} else if(!is_gpio_64_to_79(gpio)) {
		gpio_direction_output(gpio, value);
		printk(KERN_ALERT "mux_gpio_set: dev=%s gpio_direction_output gpio%d %d\n", kbuf_devp->name, gpio, value);
	} else {
		gpio_set_value_cansleep(gpio, value);
		printk(KERN_ALERT "mux_gpio_set: dev=%s gpio_set_value_cansleep gpio%d %d\n", kbuf_devp->name, gpio, value);
	}
	return 0;
}

void configure_pins(struct kbuf_dev* kbuf_devp, int gpio_array_row, bool out_dir) {
    int column;
    int columns_of_gpio_array = 4;

    for(column = 0; column < columns_of_gpio_array; column++) {
        int read_value = gpio_mapping_array[gpio_array_row][column];
        if(read_value != -1) {
			printk(KERN_ALERT "Configure pins: dev=%s  IO%d - %d\n", kbuf_devp->name, gpio_array_row, gpio_values_array[gpio_array_row][column]);
			mux_gpio_set(kbuf_devp, gpio_mapping_array[gpio_array_row][column], gpio_values_array[gpio_array_row][column], out_dir, column);				
        }
    }
}

bool is_valid_pin(int pin, bool echo_pin) {
	int t;
	if(pin < 0 || pin > 19) { //Works for trigger pins as well
		return false;
	}
	if(echo_pin) {
		for(t = 0; t < ROW_SIZE(invalid_echo_pins); t++) {
			if(pin == invalid_echo_pins[t]) {
				return false;
			}
		}
	}
	return true;
}

void cleanup_gpio(struct kbuf_dev* kbuf_devp) {
	int k, read_value;

	if(kbuf_devp->is_irq_requested) {
		free_irq(gpio_to_irq(gpio_mapping_array[kbuf_devp->echo_pin][0]), kbuf_devp);
		printk(KERN_ALERT "cleanup_gpio: dev=%s free_irq on gpio%d\n", kbuf_devp->name, gpio_mapping_array[kbuf_devp->echo_pin][0]);
		kbuf_devp->is_irq_requested = false;
	}
	
	if(kbuf_devp->is_gpio_requested) {
		for (k = 0; k < COLUMN_SIZE(gpio_mapping_array); k++) {
			read_value = gpio_mapping_array[kbuf_devp->trigger_pin][k];
		    if(read_value != -1) {
				gpio_free(read_value);
				printk(KERN_ALERT "cleanup_gpio: dev=%s gpio_free gpio%d\n", kbuf_devp->name, read_value);
//				printk(KERN_ALERT "Gpio free %d\n", read_value);
			}
			read_value = gpio_mapping_array[kbuf_devp->echo_pin][k];
		    if(read_value != -1) {
				gpio_free(read_value);
				printk(KERN_ALERT "cleanup_gpio: dev=%s gpio_free gpio%d\n", kbuf_devp->name, read_value);
//				printk(KERN_ALERT "Gpio free %d\n", read_value);
			}
		}
		kbuf_devp->is_gpio_requested = false;
	}
}

/*
* Open kbuf driver
*/
int kbuf_driver_open(struct inode *inode, struct file *file)
{
	//If open function is not present, file->private data is null in ioctl, read, write, etc. Hence, this empty function.
	return 0;
}

/*
 * Release kbuf driver
 */
int kbuf_driver_release(struct inode *inode, struct file *file)
{
	struct kbuf_dev* kbuf_devp = container_of(file->private_data, struct kbuf_dev, misdev);
	mutex_lock(&kbuf_devp->flag_lock);
	kbuf_devp->is_measuring = false;
	mutex_unlock(&kbuf_devp->flag_lock);
	
	mutex_destroy(&kbuf_devp->flag_lock);
	flush_scheduled_work();
	printk(KERN_ALERT "close: dev=%s\n", kbuf_devp->name);
	cleanup_gpio(kbuf_devp);
	
	return 0;
}

//Platform driver release
void release_driver(struct device* dev) {
/*	struct platform_device* p = container_of(dev, struct platform_device, dev);
	struct kbuf_dev* kbuf_devp = container_of(p, struct kbuf_dev, plf_dev);
	cleanup_gpio(kbuf_devp);
	printk(KERN_ALERT "Release called %d %p\n", dev->devt, kbuf_devp);
*/
}

inline void send_trigger(struct kbuf_dev* kbuf_devp);

void send_trigger(struct kbuf_dev* kbuf_devp) {
	int i;
	gpio_set_value_cansleep(gpio_mapping_array[kbuf_devp->trigger_pin][0], 0);
	for(i = 0; i < (kbuf_devp->m_samples+2); i++) {
		printk(KERN_ALERT "send_trigger: dev=%s %d trigger(s) on gpio%d\n", kbuf_devp->name, ((kbuf_devp->m_samples)+2), gpio_mapping_array[kbuf_devp->trigger_pin][0]);
		gpio_set_value_cansleep(gpio_mapping_array[kbuf_devp->trigger_pin][0], 1);
		udelay(20);
		gpio_set_value_cansleep(gpio_mapping_array[kbuf_devp->trigger_pin][0], 0);
		msleep(kbuf_devp->trigger_interval); //Trigger seperation
	}
}

int thread_function(void *data) {
	struct kbuf_dev* kbuf_devp = data;
	printk(KERN_ALERT "thread_function: dev=%s %s\n", kbuf_devp->name, kbuf_devp->name);
//	while(!kthread_should_stop()) {
		send_trigger(kbuf_devp);
//	}

	return 0;
}

void start_measurement(struct kbuf_dev* kbuf_devp) {
	//Start the measurement
	mutex_lock(&kbuf_devp->flag_lock);
	kbuf_devp->is_measuring = true;
	mutex_unlock(&kbuf_devp->flag_lock);
	sema_init(&kbuf_devp->signal_lock, 0);
	printk(KERN_ALERT "start_measurement: dev=%s kthread_run\n", kbuf_devp->name);
	kbuf_devp->kthread_task = kthread_run(&thread_function, kbuf_devp, kbuf_devp->name);
}

/*
 * Write to kbuf driver
 */
ssize_t kbuf_driver_write(struct file *file, const char *buf,
           size_t count, loff_t *ppos)
{
	int user_data = 0;
	bool ongoing = false;
	struct kbuf_dev* kbuf_devp = container_of(file->private_data, struct kbuf_dev, misdev);
	printk(KERN_ALERT "write: dev=%s\n", kbuf_devp->name);

	mutex_lock(&kbuf_devp->flag_lock);
	ongoing = kbuf_devp->is_measuring;
	mutex_unlock(&kbuf_devp->flag_lock);

	if(ongoing) {
		printk(KERN_ALERT "write: dev=%s Measurement ongoing\n", kbuf_devp->name);
		return -EINVAL;
	}
	
	if(copy_from_user(&user_data, buf, 1)) {
		printk(KERN_ALERT "write: dev=%s copy_from_user failed\n", kbuf_devp->name);
		return -EFAULT;
	}
	if(user_data) {
		//Stop ongoing measurements
//		kthread_stop(kbuf_devp->kthread_task);
		printk(KERN_ALERT "dev=%s kthread_stop\n", kbuf_devp->name);
		
		//Clear the per-device buffer
		// We don't need sync here, since no measurement is running
		printk(KERN_ALERT "write: dev=%s Clear fifo buffer\n", kbuf_devp->name);
		memset(kbuf_devp->hcsr_data, 0, sizeof(kbuf_devp->hcsr_data));
		kbuf_devp->current_fifo_write_idx = 0;
		kbuf_devp->current_fifo_read_idx = 0;
		kbuf_devp->current_fifo_length = 0;
	}
	start_measurement(kbuf_devp);
	return 0;
}
/*
 * Read to kbuf driver
 */
ssize_t kbuf_driver_read(struct file *file, char *buf,
           size_t count, loff_t *ppos)
{
	int ret = 0, fifo_length;
	bool ongoing = false;
	struct hcsrdata* temp;
	struct kbuf_dev* kbuf_devp = container_of(file->private_data, struct kbuf_dev, misdev);
	printk(KERN_ALERT "read: dev=%s\n", kbuf_devp->name);
	
	mutex_lock(&kbuf_devp->flag_lock);
	ongoing = kbuf_devp->is_measuring;
	mutex_unlock(&kbuf_devp->flag_lock);
	
	mutex_lock(&kbuf_devp->read_lock);
	fifo_length = kbuf_devp->current_fifo_length;
	mutex_unlock(&kbuf_devp->read_lock);
	
	if(fifo_length == 0) { //Empty buffer
		printk(KERN_ALERT "read: dev=%s Buffer is empty\n", kbuf_devp->name);
		if(!ongoing) { //Measurement not ongoing, start new
			printk(KERN_ALERT "read: dev=%s Measurement is not ongoing\n", kbuf_devp->name);
			start_measurement(kbuf_devp); //Assuming ioctl has already configured everything
		}
		printk(KERN_ALERT "read: dev=%s blocking until measurement completes\n", kbuf_devp->name);
		ret = down_interruptible(&kbuf_devp->signal_lock); //Block until ongoing measurement completes
//		down_timeout(&kbuf_devp->signal_lock, msecs_to_jiffies(kbuf_devp->trigger_interval));

		//Yeild the processor, wait for measurement to complete
/*		kbuf_devp->sleeping_task = current;
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
*/
		printk(KERN_ALERT "read: dev=%s returned from block\n", kbuf_devp->name);
		if(ret < 0) {
			return ret;
		}
	}

	temp = (struct hcsrdata*) kzalloc(sizeof(struct hcsrdata), GFP_KERNEL);

	printk(KERN_ALERT "read: dev=%s FIFO state read_idx=%d len=%d", kbuf_devp->name, kbuf_devp->current_fifo_read_idx, kbuf_devp->current_fifo_length);

	memcpy(temp, &kbuf_devp->hcsr_data[kbuf_devp->current_fifo_read_idx++], sizeof(struct hcsrdata));

	mutex_lock(&kbuf_devp->read_lock);
	if(kbuf_devp->current_fifo_length > 0) {
		kbuf_devp->current_fifo_length--;
	}
	mutex_unlock(&kbuf_devp->read_lock);
	
	if(kbuf_devp->current_fifo_read_idx == FIFO_SIZE) {
		kbuf_devp->current_fifo_read_idx = 0;
	}

	ret = copy_to_user(buf, temp, sizeof(struct hcsrdata));
	printk(KERN_ALERT "read: dev=%s copy_to_user %d bytes\n", kbuf_devp->name, count);
	kfree(temp);
	return ret;
}

int configure_irq(struct kbuf_dev* kbuf_devp, bool rising) {
	int ret = gpio_to_irq(gpio_mapping_array[kbuf_devp->echo_pin][0]);
	printk(KERN_ALERT "configure_irq: dev=%s gpio_to_irq %d gpio%d\n", kbuf_devp->name, ret, gpio_mapping_array[kbuf_devp->echo_pin][0]);
	if(ret < 0) {
		return ret;
	}
	if(kbuf_devp->is_irq_requested) {
		free_irq(ret, kbuf_devp);
		kbuf_devp->is_irq_requested = false;
	}
//	printk(KERN_ALERT "Calling request_irq w/ %d %d %s %p", ret,  IRQ_TYPE_EDGE_RISING, kbuf_devp->name, (void*) kbuf_devp);

	if(rising) {
		ret = request_irq(ret, irq_handler,  /*IRQF_TRIGGER_RISING*/ IRQ_TYPE_EDGE_RISING, kbuf_devp->name, (void*) kbuf_devp);
		if(ret >= 0)
			kbuf_devp->is_irq_requested = true;
		
		printk(KERN_ALERT "configure_irq: dev=%s request_irq (rising) %d\n", kbuf_devp->name, ret);
	}
	else {
		ret = request_irq(ret, irq_handler,  /*IRQF_TRIGGER_FALLING*/ IRQ_TYPE_EDGE_FALLING, kbuf_devp->name, (void*) kbuf_devp);
		printk(KERN_ALERT "configure_irq: dev=%s request_irq (falling) %d\n", kbuf_devp->name, ret);
	}
	return ret;
}

void work_handler(struct work_struct *data) {
	struct kbuf_dev* kbuf_devp = container_of(data, struct kbuf_dev, irq_work);
	uint64_t distance_cm = div_u64((kbuf_devp->time2 - kbuf_devp->time1) * 170, 4000000);
	printk(KERN_ALERT "work_handler: dev=%s distance (cm) %llu\n", kbuf_devp->name, distance_cm);
	
	//Populate the temp buffer
	if(kbuf_devp->current_sample_idx < (kbuf_devp->m_samples+2)) {
		kbuf_devp->temp_buf[kbuf_devp->current_sample_idx++] = distance_cm;
	}
	
	if(kbuf_devp->current_sample_idx == (kbuf_devp->m_samples+2)) {
		//remove outliers, take avg, store in fifo buffer
		int i;
		uint64_t sum = 0, average;
		int min = kbuf_devp->temp_buf[0];
		int max = kbuf_devp->temp_buf[0];
		int min_idx = 0;
		int max_idx = 0;
		
		kbuf_devp->current_sample_idx = 0; //Reset the temp buffer for next samples
		
		for(i = 0; i < kbuf_devp->m_samples+2; i++) {
			if(kbuf_devp->temp_buf[i] < min) {
				min = kbuf_devp->temp_buf[i];
				min_idx = i;
			}
			if(kbuf_devp->temp_buf[i] > max) {
				max = kbuf_devp->temp_buf[i];
				max_idx = i;
			}
		}
		printk(KERN_ALERT "work_handler: dev=%s outliers %llu %llu\n", kbuf_devp->name, kbuf_devp->temp_buf[min_idx], kbuf_devp->temp_buf[max_idx]);
		kbuf_devp->temp_buf[min_idx] = 0;
		kbuf_devp->temp_buf[max_idx] = 0;
		
		for(i = 0; i < kbuf_devp->m_samples+2; i++) {
			sum += kbuf_devp->temp_buf[i];
		}
		average = div_u64(sum, kbuf_devp->m_samples);
		kbuf_devp->last_distance = average;
		printk(KERN_ALERT "work_handler: dev=%s average (cm) %llu\n", kbuf_devp->name, average);
		
		
		if(kbuf_devp->current_fifo_write_idx == FIFO_SIZE) {
			printk(KERN_ALERT "dev=%s FIFO wrap around\n", kbuf_devp->name);
			kbuf_devp->current_fifo_write_idx = 0; //Wrap around
		}
	
		//Write to FIFO
/*		if(kbuf_devp->hcsr_data[kbuf_devp->current_fifo_write_idx].is_valid) {
			printk(KERN_ALERT "dev=%s FIFO overwrite at %d. New read_idx=%d\n", kbuf_devp->name, kbuf_devp->current_fifo_write_idx, ++kbuf_devp->current_fifo_read_idx); //Adjust the read index to point to the oldest data in the FIFO
		}*/

		printk(KERN_ALERT "work_handler: dev=%s FIFO entry write_idx=%d value=%llu", kbuf_devp->name, kbuf_devp->current_fifo_write_idx, average);
		kbuf_devp->hcsr_data[kbuf_devp->current_fifo_write_idx].data = average;
		kbuf_devp->hcsr_data[kbuf_devp->current_fifo_write_idx].timestamp = rdtsc();
		kbuf_devp->hcsr_data[kbuf_devp->current_fifo_write_idx++].is_valid = true;

		mutex_lock(&kbuf_devp->read_lock);
		if(kbuf_devp->current_fifo_length < FIFO_SIZE) {
			kbuf_devp->current_fifo_length++; //This param indicates if buffer is full
			mutex_unlock(&kbuf_devp->read_lock);
		} else {
			mutex_unlock(&kbuf_devp->read_lock);
			printk(KERN_ALERT "work_handler: dev=%s FIFO full, overwritten. oldest_data_idx=%d", kbuf_devp->name, kbuf_devp->current_fifo_read_idx);
		}

		mutex_lock(&kbuf_devp->flag_lock);
		kbuf_devp->is_measuring = false;
		mutex_unlock(&kbuf_devp->flag_lock);

		up(&kbuf_devp->signal_lock); //Signal to the blocking read call
/*		if(kbuf_devp->sleeping_task) {
			wake_up_process(kbuf_devp->sleeping_task);
			kbuf_devp->sleeping_task = NULL;
		}*/
		printk(KERN_ALERT "work_handler: dev=%s Measurement complete sem=%d\n", kbuf_devp->name, kbuf_devp->signal_lock.count);		
		return;
	}
}

irqreturn_t irq_handler(int irq, void *dev_id) {
	struct kbuf_dev* kbuf_devp = dev_id; //Retrieve our per device struct
	if(!kbuf_devp->irq_type_flag) {
		kbuf_devp->time1 = rdtsc();
		irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);
		kbuf_devp->irq_type_flag = true;
//		printk(KERN_ALERT "rising\n");
	} else {
		kbuf_devp->time2 = rdtsc();
		schedule_work(&kbuf_devp->irq_work);

		irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
		kbuf_devp->irq_type_flag = false;
		printk(KERN_ALERT "irq_handler: (falling) dev=%s count %d\n", kbuf_devp->name, ++kbuf_devp->irq_count);
	}
	return IRQ_HANDLED;
}

long kbuf_ioctl(struct file *file, unsigned int cmd, unsigned long datap) {
	int ret;
	unsigned long user_data[2];
	struct kbuf_dev *kbuf_devp = container_of(file->private_data, struct kbuf_dev, misdev);
	printk(KERN_ALERT "ioctl: kbuf_devp %p\n", kbuf_devp->name);

	if(kbuf_devp->is_measuring) {
		return -EINVAL; //Reject params since measurement is ongoing
	}
	
	//copy parameters from user space
	ret = copy_from_user(user_data, (unsigned long*) datap, sizeof(user_data));
	if(ret) {
		printk(KERN_ALERT "ioctl: dev=%s copy_from_user %d\n", kbuf_devp->name, ret);
		return -EFAULT;
	}

	if(cmd == CONFIG_PINS) {
		printk(KERN_ALERT "ioctl: dev=%s Pins before %d %d\n", kbuf_devp->name, kbuf_devp->trigger_pin, kbuf_devp->echo_pin);
		kbuf_devp->trigger_pin = user_data[0];
		kbuf_devp->echo_pin = user_data[1];
		printk(KERN_ALERT "ioctl: dev=%s Pins After %d %d\n", kbuf_devp->name, kbuf_devp->trigger_pin, kbuf_devp->echo_pin);

		if((kbuf_devp->trigger_pin == kbuf_devp->echo_pin) || !is_valid_pin(kbuf_devp->trigger_pin, false)
			|| !is_valid_pin(kbuf_devp->echo_pin, true)) {
			printk(KERN_ALERT "ioctl: dev=%s Invalid pins in the input\n", kbuf_devp->name);
			return -EINVAL;
		}

		configure_pins(kbuf_devp, kbuf_devp->trigger_pin, true);
		configure_pins(kbuf_devp, kbuf_devp->echo_pin, false);
		kbuf_devp->is_gpio_requested = true; 

		INIT_WORK(&kbuf_devp->irq_work, work_handler); //Initialize an interrupt bottom half for calculating distance

		return configure_irq(kbuf_devp, true); //Setup irq

	} else if (cmd == SET_PARAMETERS) {
		kbuf_devp->m_samples = user_data[0];
		kbuf_devp->delta = user_data[1];
		
		if(kbuf_devp->m_samples <= 0 || kbuf_devp->delta <= 0) {
			printk(KERN_ALERT "ioctl: dev=%s m and/or delta value(s) cannot be <= 0\n", kbuf_devp->name);
			return -EINVAL;
		}
		kbuf_devp->trigger_interval = div_u64(kbuf_devp->delta, (kbuf_devp->m_samples+2));
		if(kbuf_devp->trigger_interval < 60) {
			kbuf_devp->trigger_interval = 0;
			printk(KERN_ALERT "ioctl: dev=%s delta/(m+2) < 60 is not allowed\n", kbuf_devp->name);
			return -EINVAL;
		}
		kbuf_devp->temp_buf = (uint64_t*) kzalloc((kbuf_devp->m_samples+2) * sizeof(uint64_t), GFP_KERNEL);
		mutex_init(&kbuf_devp->read_lock); //For atomic access to shared fifo
		mutex_init(&kbuf_devp->flag_lock); //For atomic access to measurement ongoing flag
		sema_init(&kbuf_devp->signal_lock, 0); //For blocking read operation
	}
	return 0;
}


/* File operations structure. Defined in linux/fs.h */
static struct file_operations kbuf_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= kbuf_driver_open,        /* Open method */
    .release	= kbuf_driver_release,     /* Release method */
    .write		= kbuf_driver_write,       /* Write method */
    .read		= kbuf_driver_read,        /* Read method */
	.unlocked_ioctl = kbuf_ioctl
};


ssize_t show_trigger_pin(struct device *dev, struct device_attribute *attr,char *buf) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	int len = snprintf(buf, PAGE_SIZE, "%d\n", kbuf_devp->trigger_pin);
	printk(KERN_ALERT "show: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->trigger_pin);
	return len;
}

ssize_t show_echo_pin(struct device *dev, struct device_attribute *attr,char *buf) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	int len = snprintf(buf, PAGE_SIZE, "%d\n", kbuf_devp->echo_pin);
	printk(KERN_ALERT "show: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->echo_pin);
	return len;
}

ssize_t show_number_samples(struct device *dev, struct device_attribute *attr,char *buf) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	int len = snprintf(buf, PAGE_SIZE, "%d\n", kbuf_devp->m_samples);
	printk(KERN_ALERT "show: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->m_samples);
	return len;
}

ssize_t show_sampling_period(struct device *dev, struct device_attribute *attr,char *buf) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	int len = snprintf(buf, PAGE_SIZE, "%d\n", kbuf_devp->delta);
	printk(KERN_ALERT "show: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->delta);
	return len;
}

ssize_t show_enable(struct device *dev, struct device_attribute *attr,char *buf) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	int len = snprintf(buf, PAGE_SIZE, "%d\n", kbuf_devp->enable);
	printk(KERN_ALERT "show: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->enable);
	return len;
}

ssize_t show_distance(struct device *dev, struct device_attribute *attr,char *buf) {
	int len;
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	flush_scheduled_work();
	len = snprintf(buf, PAGE_SIZE, "%llu\n", kbuf_devp->last_distance);
	printk(KERN_ALERT "show: dev=%s attr=%s value=%llu\n", kbuf_devp->name, attr->attr.name, kbuf_devp->last_distance);
	return len;
}

ssize_t store_trigger_pin(struct device *dev, struct device_attribute *attr,const char *buf, size_t count) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	sscanf(buf, "%d", &kbuf_devp->trigger_pin);
	printk(KERN_ALERT "store_trigger_pin: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->trigger_pin);
	if(!is_valid_pin(kbuf_devp->echo_pin, false)) {
		printk(KERN_ALERT "store_trigger_pin: failed dev=%s trigger_pin is not valid\n", kbuf_devp->name);
		return count;
	}
	configure_pins(kbuf_devp, kbuf_devp->trigger_pin, true);
	kbuf_devp->is_gpio_requested = true;
	return count;
}

ssize_t store_echo_pin(struct device *dev, struct device_attribute *attr,const char *buf, size_t count) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	sscanf(buf, "%d", &kbuf_devp->echo_pin);
	printk(KERN_ALERT "store_echo_pin: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->echo_pin);
	if(!is_valid_pin(kbuf_devp->echo_pin, true)) {
		printk(KERN_ALERT "store_echo_pin: failed dev=%s echo_pin is not valid\n", kbuf_devp->name);
		return count;
	}
	configure_pins(kbuf_devp, kbuf_devp->echo_pin, false);
	kbuf_devp->is_gpio_requested = true;
	INIT_WORK(&kbuf_devp->irq_work, work_handler); //Initialize an interrupt bottom half for calculating distance
	configure_irq(kbuf_devp, true);
	return count;
}

ssize_t store_number_samples(struct device *dev, struct device_attribute *attr,const char *buf, size_t count) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	sscanf(buf, "%d", &kbuf_devp->m_samples);
	printk(KERN_ALERT "store_number_samples: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->m_samples);
	if(kbuf_devp->m_samples <= 0) {
		kbuf_devp->m_samples = 0;
		printk(KERN_ALERT "store_number_samples: failed dev=%s number_samples can't be <= 0\n", kbuf_devp->name);
		return count;
	}
	kbuf_devp->temp_buf = (uint64_t*) kzalloc((kbuf_devp->m_samples+2) * sizeof(uint64_t), GFP_KERNEL); //Allocate a FIFO
	mutex_init(&kbuf_devp->read_lock); //For atomic access to shared fifo
	mutex_init(&kbuf_devp->flag_lock); //For atomic access to measurement ongoing flag
	sema_init(&kbuf_devp->signal_lock, 0); //For blocking read operation
	return count;
}

ssize_t store_sampling_period(struct device *dev, struct device_attribute *attr,const char *buf, size_t count) {
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	sscanf(buf, "%d", &kbuf_devp->delta);
	printk(KERN_ALERT "store_sampling_period: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->delta);
	if(kbuf_devp->delta <= 0) {
		kbuf_devp->delta = 0;
		printk(KERN_ALERT "store_sampling_period: failed dev=%s sampling_period can't be <= 0\n", kbuf_devp->name);
		return count;
	}
	return count;
}

bool check_all_params(struct kbuf_dev* kbuf_devp) {
	if((kbuf_devp->trigger_pin == kbuf_devp->echo_pin) || !is_valid_pin(kbuf_devp->trigger_pin, false)
		|| !is_valid_pin(kbuf_devp->echo_pin, true)) {
		printk(KERN_ALERT "check_all_params: failed dev=%s Invalid pins in the input\n", kbuf_devp->name);
		return false;
	}
	if(kbuf_devp->m_samples <= 0 || kbuf_devp->delta <= 0) {
		printk(KERN_ALERT "check_all_params: failed dev=%s m and/or delta value(s) cannot be <= 0\n", kbuf_devp->name);
		return false;
	}
	kbuf_devp->trigger_interval = div_u64(kbuf_devp->delta, (kbuf_devp->m_samples+2));
	if(kbuf_devp->trigger_interval < 60) {
		kbuf_devp->trigger_interval = 0;
		printk(KERN_ALERT "check_all_params: failed dev=%s delta/(m+2) < 60 is not allowed\n", kbuf_devp->name);
		return false;
	}
	return true;
}

ssize_t store_enable(struct device *dev, struct device_attribute *attr,const char *buf, size_t count) {
	bool ongoing = false;
	struct kbuf_dev* kbuf_devp = dev_get_drvdata(dev);
	sscanf(buf, "%d", &kbuf_devp->enable);
	printk(KERN_ALERT "store: dev=%s attr=%s value=%d\n", kbuf_devp->name, attr->attr.name, kbuf_devp->enable);

	if(!check_all_params(kbuf_devp)) {
		printk(KERN_ALERT "store_enable: failed dev=%s Set all params before enabling measurement\n", kbuf_devp->name);
		return count;
	}
	
	mutex_lock(&kbuf_devp->flag_lock);
	ongoing = kbuf_devp->is_measuring;
	mutex_unlock(&kbuf_devp->flag_lock);

	//Measurement stops automatically after collecting m+2 samples
	if(kbuf_devp->enable && !ongoing) {
		start_measurement(kbuf_devp);
	}
	return count;
}

static DEVICE_ATTR(trigger_pin, S_IRUGO | S_IWUSR, show_trigger_pin,
                   store_trigger_pin);
static DEVICE_ATTR(echo_pin, S_IRUGO | S_IWUSR, show_echo_pin,
                   store_echo_pin);
static DEVICE_ATTR(number_samples, S_IRUGO | S_IWUSR, show_number_samples,
                   store_number_samples);
static DEVICE_ATTR(sampling_period, S_IRUGO | S_IWUSR, show_sampling_period,
                   store_sampling_period);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, show_enable,
                   store_enable);
static DEVICE_ATTR(distance, S_IRUGO | S_IWUSR, show_distance,
                   NULL);

static struct attribute *hcsr_attrs[] = {
	&dev_attr_trigger_pin.attr,
	&dev_attr_echo_pin.attr,
	&dev_attr_number_samples.attr,
	&dev_attr_sampling_period.attr,
	&dev_attr_enable.attr,
	&dev_attr_distance.attr,
	NULL,
};

static struct attribute_group hcsr_attr_group = {
	.attrs = hcsr_attrs,
};

static int P_driver_probe(struct platform_device *dev_found)
{
	int ret = 0;
	struct kbuf_dev* kbuf_devp = container_of(dev_found, struct kbuf_dev, plf_dev);

	kbuf_devp->plf_dev.dev.release = release_driver; //Release function to avoid dmesg warning
	kbuf_devp->group = &hcsr_attr_group; //Attach attribute_group

	//Create a misc device
	kbuf_devp->misdev.minor = MISC_DYNAMIC_MINOR;
	kbuf_devp->misdev.fops = &kbuf_fops;

	ret = misc_register(&(kbuf_devp->misdev));
	printk("probe: misc_register dev=%s status=%d ptr=%p minor=%d\n", kbuf_devp->misdev.name, ret, kbuf_devp, kbuf_devp->misdev.minor);
	
	kbuf_devp->hcsr_device = device_create(kbuf_devp->kbuf_dev_class, kbuf_devp->misdev.this_device, kbuf_devp->misdev.minor, kbuf_devp, kbuf_devp->name);
	if(IS_ERR(kbuf_devp->hcsr_device)) {
		printk(KERN_ALERT "probe: dev=%s device_create error\n", kbuf_devp->name);
		return PTR_ERR(kbuf_devp->hcsr_device);
	}
	ret = sysfs_create_group(&kbuf_devp->hcsr_device->kobj, kbuf_devp->group);
	printk(KERN_ALERT "probe: dev=%s sysfs_create_group %d\n", kbuf_devp->name, ret);
	return 0;
};

static int P_driver_remove(struct platform_device *pdev)
{
	struct kbuf_dev* kbuf_devp = container_of(pdev, struct kbuf_dev, plf_dev);
	cleanup_gpio(kbuf_devp);
	kfree(kbuf_devp->temp_buf);

	sysfs_remove_group(&kbuf_devp->hcsr_device->kobj, kbuf_devp->group);
	device_destroy(kbuf_devp->kbuf_dev_class, kbuf_devp->misdev.minor);
	misc_deregister(&(kbuf_devp->misdev));
	
	printk(KERN_ALERT "remove: dev=%p -- %s  %d \n", kbuf_devp, kbuf_devp->name, kbuf_devp->dev_no);
	return 0;
};

static struct platform_driver P_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= P_driver_probe,
	.remove		= P_driver_remove,
};

module_platform_driver(P_driver);
MODULE_LICENSE("GPL");