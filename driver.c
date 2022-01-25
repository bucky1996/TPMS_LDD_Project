#include <linux/kernel.h> /* Needed by all modules */
#include <linux/init.h>  /* Needed for the macros */
#include <linux/module.h> /* Needed for KERN_INFO */
#include <linux/kdev_t.h>
#include <linux/fs.h>  /*For file operations fops struct */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h> /*kmalloc*/
#include <linux/uaccess.h>/* copy to and from user */
#include <linux/ioctl.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>/* Interrupt related functions */
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/signal.h>
//#include <linux/signal_types.h>

#define SET_CALIBRATION _IOW(31,31,int32_t)
#define GET_CALIBRATION _IOR(31,32,int32_t)
#define REG_CURRENT_TASK _IOW(32,31,int32_t)
#define THRESHOLD_VALUE 23
MODULE_LICENSE("GPL");
#define SIGETX 44
#define IRQ_NO 11

static int tpms_open(struct inode *inode, struct file  *file);
static int tpms_release(struct inode* inode,struct file *file);
static ssize_t tpms_read(struct file*file,char __user*buf,size_t len,loff_t *off);
static ssize_t tpms_write(struct file*file, const char __user*buf,size_t len,loff_t *off);
static long tpms_ioctl(struct file*file, unsigned int cmd, unsigned long arg);
static int tpms_map_mmap(struct file*file,struct vm_area_struct* vma);
unsigned int poll(struct file *filp, struct poll_table_struct *wait);

static struct cdev etx_cdev;
static char *model = "Ford GT";
static int u8_fl = 27;
static int u8_fr = 27;
static int u8_rl = 27;
static int u8_rr = 27;
static int MAX_SIZE =512;
volatile int is_open;
static dev_t dev;
static int32_t calibration_value = 0;
struct mutex dev_mutex;
static struct task_struct *etx_thread;
int thread_function(void *pv);
//static struct task_struct *kthread;
static wait_queue_head_t waitqueue; /* defined in wait.h  It contains only a lock variable and a linked list of sleeping processes.
				       The individual data items in the list are of type wait_queue_t, 
				       and the list is the generic list defined in  <linux/list.h> */
static char temp_buf[1024];
static void *msg;
static struct task_struct *task = NULL;
static int signum = 0;
int32_t value = 0;

/* interrupt handler function */
static irqreturn_t irq_handler(int irq,void *dev_id) {
	struct siginfo info;//defined in signal.h
	printk(KERN_INFO "Shared IRQ: Interrupt Occurred");
	//Sending signal to app
	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = SIGETX;
	info.si_code = SI_QUEUE;
	info.si_int = 1;
	if (task != NULL) {
		printk(KERN_INFO "Sending signal to app\n");
		if(send_sig_info(SIGETX, &info, task) < 0) {
			printk(KERN_INFO "Unable to send signal\n");
		}
	}

	return IRQ_HANDLED;
}

/*provide interface for driver file operationsa*/
static struct file_operations fops=
{
	.owner          = THIS_MODULE,
	.read           = tpms_read,
	.write          = tpms_write,
	.open           = tpms_open,
	.release        = tpms_release,
	.unlocked_ioctl = tpms_ioctl,
	.mmap           = tpms_map_mmap,
	.poll           = poll  //asynchorous event
};

/* thread function to reduce the rr value for every 5 seconds*/
int thread_function(void *pv)
{

	int i=0;
	while(!kthread_should_stop()) {
		printk(KERN_INFO " Thread Function called : %d\n", i++);
		msleep(5000);
		wake_up(&waitqueue);
		u8_rr = u8_rr-calibration_value;
		printk(KERN_ALERT "Value of rr is %d\n\n\n",u8_rr);
		if((u8_fl < THRESHOLD_VALUE) || (u8_fr < THRESHOLD_VALUE) || (u8_rl < THRESHOLD_VALUE) || (u8_rr < THRESHOLD_VALUE))
		{
			printk(KERN_ALERT "Value is not in control\n");
			asm("int $0x3B");
		}
		else
		{
			printk(KERN_ALERT "value in  control\n");
		}
	}
	return 0;
}

static int tpms_open(struct inode *inode, struct file  *file)
{
        if (is_open == 1) {
        printk(KERN_INFO "Error - tpms device is already open");
        return -EBUSY;
        }
        is_open = 1;
	printk(KERN_ALERT "Driver open function called\n");
	return 0;
}

static int tpms_release(struct inode* inode,struct file *file)
{
        if (is_open == 0) {
        printk(KERN_INFO "Error - tpms device is already open");
        return -EBUSY;
        }
        is_open = 0;
	printk(KERN_ALERT "Driver release function is called\n");
	return 0;
}
/* Driver read function update the buffer value and send to application program to display tyre pressure values*/
static ssize_t tpms_read(struct file *file, char __user *obuf, size_t len, loff_t *off)
{
	printk(KERN_ALERT " DRiver read function called\n");
	sprintf(temp_buf, "Pressure values for the tyres are - FL: %d ,FR: %d ,RL: %d ,RR: %d", u8_fl,u8_fr,u8_rl,u8_rr-calibration_value);
	wake_up_process(etx_thread);
	copy_to_user(obuf,temp_buf,sizeof(temp_buf));
	wake_up_process(etx_thread);
	return len;
}
/* write function is protected from concurrent access using mutex lock*/
static ssize_t tpms_write(struct file *file, const char __user *ibuf,size_t len,loff_t *off)
{
	printk(KERN_ALERT "Driver write function is called\n");
	mutex_lock(&dev_mutex);
	msleep(10000);
	printk(KERN_ALERT "mutex lock calledoff  for 10 seconds\n");
	mutex_unlock(&dev_mutex);
	printk(KERN_ALERT "userbuf %s\n\n\n",ibuf);
	return len;
}
/*Provide the driver with appropriate action based on command received*/ 
static long tpms_ioctl(struct file*file, unsigned int cmd, unsigned long arg)
{
	switch(cmd)
	{
		case SET_CALIBRATION:
			copy_from_user(&calibration_value,(int32_t*)arg,sizeof(calibration_value));
			printk(KERN_INFO "Value of calib is = %d\n",calibration_value);
			break;
		case GET_CALIBRATION:
			copy_to_user((int32_t*)arg,&calibration_value,sizeof(calibration_value));
			break;
		case REG_CURRENT_TASK: 
			printk(KERN_INFO "REG_CURRENT_TASK\n");
			task = get_current();
			signum = SIGETX;
			break;

	}
	return 0;
}
/*Send the asynchoronous event when rear right tyre pressure value is below threshold level*/
/*Note: For simulation purpose rr value is reduced and check*/
unsigned int poll(struct file *filp, struct poll_table_struct *wait)
{
	poll_wait(filp, &waitqueue, wait);
	if (u8_rr < THRESHOLD_VALUE)
		return POLLIN;
	else
		return 0;
}
/* Driver firmware handler function */
static int tpms_map_mmap(struct file*file,struct vm_area_struct* vma)
{
	int ret=0;
	void* msg;
	struct page*page = NULL;
	unsigned long size = (unsigned long)(vma->vm_end - vma->vm_start);
	if(size > MAX_SIZE){
		ret = -EINVAL;
		goto out;
	}
	page = virt_to_page((unsigned long)msg +(vma->vm_pgoff << PAGE_SHIFT));
	ret = remap_pfn_range(vma,vma->vm_start,page_to_pfn(page),size,vma->vm_page_prot);
	if(ret!=0){
		goto out;
	}
out:
	return ret;
}
/*Driver initialization module*/
static int tpms_init(void)
{
	int count = 1;
	printk(KERN_ALERT "TPMS initiated\n");
	printk(KERN_ALERT "TPMS for model %s",model);
	dev = MKDEV(123,0);
	register_chrdev_region(dev,count,"tpms_dev");
	cdev_init(&etx_cdev,&fops);
	etx_cdev.owner = THIS_MODULE;
	etx_cdev.ops = &fops;

	if((cdev_add(&etx_cdev,dev,1))< 0) // registers char device in kernel
	{
		printk(KERN_ALERT " Cannot add device\n");
		return -1;
	}
        else
        {
                printk(KERN_INFO "char_dev registered with maj_num 123");
        } 
	mutex_init(&dev_mutex);
	msg = kmalloc(MAX_SIZE,GFP_KERNEL);
	if(msg == NULL)
	{
		return(-ENOMEM);
	}
	init_waitqueue_head(&waitqueue);
	etx_thread = kthread_create(thread_function,NULL,"eTx Thread");
	if(etx_thread) {
		wake_up_process(etx_thread);
	} else {
		printk(KERN_ERR "Cannot create kthread\n");
	}
	if (request_irq(IRQ_NO, irq_handler, IRQF_SHARED, "tpms_dev", (void *)(irq_handler))) {
		printk(KERN_INFO "my_device: cannot register IRQ ");
		goto irq;
irq:
		free_irq(IRQ_NO,(void *)(irq_handler));
	}
	return 0;

}

static void tpms_exit(void)
{
	printk(KERN_ALERT "TPMS removed\n");
	free_irq(IRQ_NO,(void *)(irq_handler));
	kthread_stop(etx_thread);
	kfree(msg);
	cdev_del(&etx_cdev);
	unregister_chrdev_region(dev,1);
}
module_param(model,charp ,S_IRUGO); 
module_init(tpms_init);
module_exit(tpms_exit);
