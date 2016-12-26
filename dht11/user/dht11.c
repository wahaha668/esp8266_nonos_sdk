#include "ets_sys.h"
#include "driver/uart.h"
#include "uart.h"
#include "uart_register.h"
#include "osapi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "stdlib.h"
#include "stdio.h"

void dht_init();
void dht_getdat();
uint8_t dht_readat();
void st(char *s, int n);

os_timer_t dht_timer;

uint8_t data[5];

void st(char *s, int n)
{
    char *num;
    int i=0, j=0, m=0;

    num = (char *)os_malloc(sizeof(char));
    while(n>0)
    {
        *(num+i) = n%10;
        n = n/10;
        i++;
    }
    for(j=(i-1), m=0;j>=0;j--, m++)
        *(s+m) = num[j] + '0';
}

static uint8_t dht_readbit(void)
{
	while(GPIO_INPUT_GET(12) == 0);

	os_delay_us(40);

	if(GPIO_INPUT_GET(12) == 1)
	{
		while(GPIO_INPUT_GET(12) == 1);
		return 1;
	}
	else
		return 0;
}

uint8_t dht_readat()
{
	uint8_t i, dat=0;

	for(i=0; i<8; i++)
	{
		dat <<= 1;
		dat |= dht_readbit();
	}
	return dat;
}

void dht_getdat()
{
	uint8_t i=0;

	gpio_output_set(0, BIT12, BIT12, 0);
	os_delay_us(18000);//low level at least 18ms
	gpio_output_set(BIT12, 0, BIT12, 0);
	os_delay_us(50);

	gpio_output_set(0, 0, 0, BIT12);
	if(GPIO_INPUT_GET(12) == 0)
	{
		os_delay_us(160);

		for(i=0; i<5; i++)
		{
			data[i] = dht_readat();
		}
		os_delay_us(50);
		gpio_output_set(BIT12, 0, BIT12, 0);
	}
	else
	{
		os_printf("dht connect error!\n");
	}

//	os_printf("dht_t1 : %d!\n", data[0]);
//	os_printf("dht_t2 : %d!\n", data[1]);
//	os_printf("dht_d1 : %d!\n", data[2]);
//	os_printf("dht_d2 : %d!\n", data[3]);
}

void dht_init()
{
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
	gpio_output_set(BIT12, 0, BIT12, 0);
	os_delay_us(1000);
}
