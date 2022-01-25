#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <poll.h>
#include <ctype.h>

#define SET_CALIBRATION _IOW(31,31,int32_t)
#define GET_CALIBRATION _IOR(31,32,int32_t)
#define THRESHOLD_VALUE 23
#define REG_CURRENT_TASK _IOW(32,32,int32_t)

#define SIGETX 44
#define IRQ_NO 11

int print_flag = 0;
static int done = 0;
int check = 0;
struct sigaction act;

void handle_alarm( int signo ) {
	print_flag = 1;
}

/* Interrupt handler function for ctrl + c */
void ctrl_c_handler(int n, siginfo_t *info, void *unused)
{
	//printf("\t %s \t\t %d\n", __func__, __LINE__);
	if (n == SIGINT) {
		printf("Recieved ctrl-c\n");
		done = 1;
	}
}

/*customized interrupt handler function*/
/*As testing is based on driver simulation it will display no irq vector available*/
void sig_event_handler(int n, siginfo_t *info, void *unused)
{
	if (n == SIGETX) {
		check = info->si_int;
		printf ("Received signal from kernel : Value =  %u\n", check);
	}
}

int main()
{
	int fd,number;
	int FL,FR,RL,RR;
	FL=FR=RL=RR=0;
	char delimt_buf[100];
	int32_t calib_value = 0;
	int8_t write_buf[1024];
	char read_buf[1024];
	char* pointer;
	struct pollfd pfd; /* To handle asynchronous event received from driver */
	int  i, n;
	short revents;

	pointer = (char*)mmap(0,4096,PROT_READ | PROT_WRITE,MAP_SHARED,fd,0);/*to simulate firmware upgrade of driver*/
	fd = open("./tpms_dev",O_RDWR); 
	if(fd < 0)
	{
		printf("cannot open the device file\n");
		return 0;
	}
	printf("done!\n\n");
	printf("Enter calib value \n");
	scanf("%d",&calib_value);
	if(!(isdigit(calib_value))){
		printf("Invalid input,Please enter numbers\n");
		return 0;
	}
	printf("Writing calibration value to Driver\n");
	ioctl(fd,SET_CALIBRATION,(int32_t*)&calib_value);
	printf("Reading calibration value from Driver\n");
	read(fd,read_buf,1024);
	printf("%s\n\n",read_buf);
	printf("Firmware memory modified buffer value %s \n\n\n", pointer);
	pfd.fd = fd;  
	pfd.events = POLLIN;
	sigemptyset (&act.sa_mask);
	act.sa_flags = (SA_SIGINFO | SA_RESETHAND);
	act.sa_sigaction = ctrl_c_handler;
	sigaction (SIGINT, &act, NULL);
	//signal( SIGINT, ctrl_c_handler );

	/* install custom signal handler */
	sigemptyset(&act.sa_mask);
	act.sa_flags = (SA_SIGINFO | SA_RESTART);
	act.sa_sigaction = sig_event_handler;
	sigaction (SIGETX, &act, NULL);

	if (ioctl(fd, REG_CURRENT_TASK,(int32_t*) &number)) {  /*To handle customized interrupt */
		printf("Failed\n");
		close(fd);
		exit(1);
	}
	signal( SIGALRM, handle_alarm );
	alarm( 5 ); /* read the tyre pressure values for every 5 seconds */

	for (;;) { 
		if ( print_flag ) {
			print_flag = false;
			alarm( 5 );
			printf("reading calibration value from Driver\n");
			read(fd,read_buf,1024);
			printf("%s\n\n",read_buf);
			ioctl(fd,SET_CALIBRATION,(int32_t*)&calib_value);
			i = poll(&pfd, 1, -1);
			if (i == -1) {
				printf("Tyre pressure is under control\n\n\n");
			}
			revents = pfd.revents;
			if (revents & POLLIN) {
				n = read(pfd.fd, read_buf, sizeof(read_buf));
				printf(" Value %s\n", read_buf);
				printf("Tyre pressure is below threshold level, press ctrl+c to stop the process\n\n");
			}

			if (ioctl(fd, REG_CURRENT_TASK,(int32_t*) &number)) {
				printf("Failed\n");
				close(fd);
				exit(1);
			}
		}
	}
	printf("buffer value %s\n", pointer);
	munmap(pointer,4096);
	return 0;
}
