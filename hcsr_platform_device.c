/*
 * A sample program to show the binding of platform driver and device.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "hcsr_platform_device.h"


static int p_device_init(void)
{
	int ret = 0, i;
	struct class* temp_class_ptr;
	char* name;

	/* Allocate memory for the per-device structure */
	kbuf_devp = kzalloc(sizeof(struct kbuf_dev) * MAX_DEVICES, GFP_KERNEL);
	
	if (!kbuf_devp) {
		printk("Bad Kmalloc\n"); return -ENOMEM;
	}

	temp_class_ptr = class_create(THIS_MODULE, CLASS_NAME); //create /sys/class/CLASS_NAME

	for(i=0; i < MAX_DEVICES; i++) {
		kbuf_devp[i].kbuf_dev_class = temp_class_ptr;
		name = kzalloc(10 * sizeof(char), GFP_KERNEL);
		memset(name, 0, 10);
		sprintf(name,"hcsr_%d",i);
		kbuf_devp[i].plf_dev.name = name; //Name for bus matching
		kbuf_devp[i].misdev.name = name; //Name of miscdevice
 		kbuf_devp[i].name = name; //Name for our reference
		kbuf_devp[i].dev_no = i;

//		kbuf_devp[i].plf_dev.dev.release = release_driver; //Release function to avoid dmesg warning

		/* Register the device */
		ret = platform_device_register(&(kbuf_devp[i].plf_dev));
		printk(KERN_ALERT "init: platform_device_register %s status %d ptr %p\n", kbuf_devp[i].name, ret, &kbuf_devp[i]);
	}
	
	return ret;
}

static void p_device_exit(void)
{
	int i;
	struct class* temp_class_ptr;

	temp_class_ptr = kbuf_devp[0].kbuf_dev_class;
	for(i=0; i<MAX_DEVICES; i++) {
		platform_device_unregister(&(kbuf_devp[i].plf_dev));
	}
	kfree(kbuf_devp->name);	
	kfree(kbuf_devp);

	/* Destroy driver_class */
	class_destroy(temp_class_ptr);

	printk(KERN_ALERT "Goodbye, unregistered the platform device\n");
}

module_init(p_device_init);
module_exit(p_device_exit);
MODULE_LICENSE("GPL");