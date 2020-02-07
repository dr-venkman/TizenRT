/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License\n");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <tinyara/gpio.h>
#include <iotbus/iotbus_gpio.h>
#include <iotbus/iotbus_error.h>
#include <tinyara/irq.h>
#include <tinyara/clock.h>

#define SS_SEG_A 5
#define SS_SEG_B 12
#define SS_SEG_C 13
#define SS_SEG_D 14
#define SS_SEG_E 22
#define SS_SEG_F 23
#define SS_SEG_G 25 
#define SS_SEG_H 26

#define SS_COM1 15
#define SS_COM2 18
#define SS_COM3 19
#define SS_COM4 21

static iotbus_gpio_context_h io_pin_com1;
static iotbus_gpio_context_h io_pin_com2;
static iotbus_gpio_context_h io_pin_com3;
static iotbus_gpio_context_h io_pin_com4;

static iotbus_gpio_context_h io_pinA;
static iotbus_gpio_context_h io_pinB;
static iotbus_gpio_context_h io_pinC;
static iotbus_gpio_context_h io_pinD;
static iotbus_gpio_context_h io_pinE;
static iotbus_gpio_context_h io_pinF;
static iotbus_gpio_context_h io_pinG;
static iotbus_gpio_context_h io_pinH;

unsigned char g_seven_seg_table[10][8] =                
{
  {0, 0,  1,  1,  1,  1,  1,  1},     //0
  {0, 0,  0,  0,  0,  1,  1,  0},     //1
  {0, 1,  0,  1,  1,  0,  1,  1},     //2
  {0, 1,  0,  0,  1,  1,  1,  1},     //3
  {0, 1,  1,  0,  0,  1,  1,  0},     //4
  {0, 1,  1,  0,  1,  1,  0,  1},     //5
  {0, 1,  1,  1,  1,  1,  0,  1},     //6
  {0, 0,  0,  0,  0,  1,  1,  1},     //7
  {0, 1,  1,  1,  1,  1,  1,  1},     //8
  {0, 1,  1,  0,  1,  1,  1,  1}      //9
};


static sem_t g_stdk_demo_sem = SEM_INITIALIZER(0);
static int g_sd_mode = 0; // check program is running


#define SSEG_OPEN(pin_var, pin_no)                                     \
	do {                                                               \
		pin_var = iotbus_gpio_open(pin_no);                            \
		if (!pin_var) {                                                \
			printf("[%s]: Checkpoint line %d\n", __FUNCTION__, __LINE__);\
			return -1;                                                 \
		}                                                              \
		iotbus_gpio_set_direction(pin_var, GPIO_DIRECTION_OUT);        \
	} while (0)

#define SSEG_CLOSE(pin_var)                                            \
	do {                                                               \
		if (pin_var) {                                                 \
			iotbus_gpio_close(pin_var);                                \
		}                                                              \
	} while (0)
	
#define SSEG_WRITE(pin_var, value)                                     \
	do {                                                               \
		iotbus_gpio_write(pin_var, 1 - value);                         \
	} while (0)

#define SSEG_DISPLAY(timer_val)														                          \
	do {                                                               \
		seven_seg_setval(4, timer_val % 10);                           \
		seven_seg_setval(3, (timer_val / 10) % 10);                    \
		seven_seg_setval(2, (timer_val / 100) % 10);                   \
		seven_seg_setval(1, (timer_val / 1000));                       \
	} while (0)


#define STDK_DEMO_SIGNAL                                               \
	do {                                                               \
		sem_post(&g_stdk_demo_sem);            	                       \
		printf(" T%d send signal\n", getpid());	                       \
	} while (0)

#define STDK_DEMO_WAIT                                                 \
	do {                                                               \
		printf(" T%d wait signal\n", getpid());	                       \
		sem_wait(&g_stdk_demo_sem);                                    \
	} while (0)

/*
 * Signal
 */
int stdk_demo_signal_init(void)
{
	if (g_sd_mode != 0) {
		printf("Program is already running\n");
		return -1;
	}
	g_sd_mode = 1;
	return 0;
}

void stdk_demo_signal_deinit(void)
{
	g_sd_mode = 0;
}

static int seven_seg_init(void)
{
	SSEG_OPEN(io_pin_com1, SS_COM1);
	SSEG_OPEN(io_pin_com2, SS_COM2);
	SSEG_OPEN(io_pin_com3, SS_COM3);
	SSEG_OPEN(io_pin_com4, SS_COM4);
	
	SSEG_OPEN(io_pinA, SS_SEG_A);
	SSEG_OPEN(io_pinB, SS_SEG_B);
	SSEG_OPEN(io_pinC, SS_SEG_C);
	SSEG_OPEN(io_pinD, SS_SEG_D);
	SSEG_OPEN(io_pinE, SS_SEG_E);
	SSEG_OPEN(io_pinF, SS_SEG_F);
	SSEG_OPEN(io_pinG, SS_SEG_G);
	SSEG_OPEN(io_pinH, SS_SEG_H);
	return 0;
}

static void seven_seg_deinit(void)
{
	SSEG_CLOSE(io_pin_com1);
	SSEG_CLOSE(io_pin_com2);
	SSEG_CLOSE(io_pin_com3);
	SSEG_CLOSE(io_pin_com4);
	
	SSEG_CLOSE(io_pinA);
	SSEG_CLOSE(io_pinB);
	SSEG_CLOSE(io_pinC);
	SSEG_CLOSE(io_pinD);
	SSEG_CLOSE(io_pinE);
	SSEG_CLOSE(io_pinF);
	SSEG_CLOSE(io_pinG);
	SSEG_CLOSE(io_pinH);
}

static void seven_seg_display(int id, int num) {
	SSEG_WRITE(io_pinA, 0);
	SSEG_WRITE(io_pinB, 0);
	SSEG_WRITE(io_pinC, 0);
	SSEG_WRITE(io_pinD, 0);
	SSEG_WRITE(io_pinE, 0);
	SSEG_WRITE(io_pinF, 0);
	SSEG_WRITE(io_pinG, 0);
	SSEG_WRITE(io_pinH, 0);

	switch (id) {
		case 1:
			SSEG_WRITE(io_pin_com1, 0);
			SSEG_WRITE(io_pin_com2, 1);
			SSEG_WRITE(io_pin_com3, 1);
			SSEG_WRITE(io_pin_com4, 1);
		break;
		
		case 2:
			SSEG_WRITE(io_pin_com1, 1);
			SSEG_WRITE(io_pin_com2, 0);
			SSEG_WRITE(io_pin_com3, 1);
			SSEG_WRITE(io_pin_com4, 1);
		break;
		
		case 3:
			SSEG_WRITE(io_pin_com1, 1);
			SSEG_WRITE(io_pin_com2, 1);
			SSEG_WRITE(io_pin_com3, 0);
			SSEG_WRITE(io_pin_com4, 1);
		break;
		
		case 4:
			SSEG_WRITE(io_pin_com1, 1);
			SSEG_WRITE(io_pin_com2, 1);
			SSEG_WRITE(io_pin_com3, 1);
			SSEG_WRITE(io_pin_com4, 0);
		break;
		
		default:
			SSEG_WRITE(io_pin_com1, 0);
			SSEG_WRITE(io_pin_com2, 0);
			SSEG_WRITE(io_pin_com3, 0);
			SSEG_WRITE(io_pin_com4, 0);
		break;
	}

	SSEG_WRITE(io_pinA, g_seven_seg_table[num][7]);
	SSEG_WRITE(io_pinB, g_seven_seg_table[num][6]);
	SSEG_WRITE(io_pinC, g_seven_seg_table[num][5]);
	SSEG_WRITE(io_pinD, g_seven_seg_table[num][4]);
	SSEG_WRITE(io_pinE, g_seven_seg_table[num][3]);
	SSEG_WRITE(io_pinF, g_seven_seg_table[num][2]);
	SSEG_WRITE(io_pinG, g_seven_seg_table[num][1]);
	SSEG_WRITE(io_pinH, g_seven_seg_table[num][0]);	
}

static void seven_seg_reset_display(void) {
	seven_seg_display(5, 0);
	return;
}

static void seven_seg_setval(int id, int val) {
	seven_seg_display(id, val);
	usleep(10);
}


static void seven_seg_countdown(int val)
{
	volatile int timeout_cnt = 0;	
	int timer_val = val;
	struct timeval ts_start;
	struct timeval ts_end;
	uint32_t tdiff;
	irqstate_t flags = irqsave();
	gettimeofday(&ts_start, NULL); 
	SSEG_DISPLAY(timer_val);
	while (timer_val) {
		gettimeofday(&ts_end, NULL); 
		tdiff = (ts_end.tv_sec - ts_start.tv_sec)*1000000ul + (ts_end.tv_usec - ts_start.tv_usec);		
		SSEG_DISPLAY(timer_val);
		timeout_cnt++;
		if (tdiff >= 1000000ul) {
			timer_val--;
			timeout_cnt = 0;
			gettimeofday(&ts_start, NULL); 
		}
	}
	seven_seg_reset_display();
	irqrestore(flags);
}

static void stdk_demo_process(int argc, char *argv[])
{
	int countdown_val;
	if (seven_seg_init() != 0) {
		goto exit;
	}
	if (argc == 2) {
		countdown_val = CONFIG_STDK_DEMO_COUNTDOWNVAL;
	} else if (argc == 3) {
		countdown_val = atoi(argv[2]);
		if (countdown_val > 9999 || countdown_val < 0) {
			goto exit;
		}
	}
	seven_seg_reset_display();
	sleep(2);
	seven_seg_countdown(countdown_val);
	printf("[%s]: Success..\n", __FUNCTION__);
exit:
	printf("[%s]: exiting ..\n", __FUNCTION__);
	seven_seg_deinit();
	STDK_DEMO_SIGNAL;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int stdk_demo_main(int argc, char *argv[])
#endif
{
	printf("STDK Demo test!!\n");
	int res = stdk_demo_signal_init();
	if (res < 0) {
		return -1;
	}
	task_create("STDK Demo sample", 100, 1024 * 16, (main_t)stdk_demo_process, argv);

	STDK_DEMO_WAIT;

	stdk_demo_signal_deinit();
	printf("Exiting %s\n", __FUNCTION__);

	return 0;
}
