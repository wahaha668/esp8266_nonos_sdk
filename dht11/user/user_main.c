/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "uart.h"
#include "uart_register.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "user_light.h"
#include "stdlib.h"
#include "stdio.h"

MQTT_Client mqttClient;
//LOCAL os_timer_t sntp_timer;

os_timer_t rtc_test_t;
#define RTC_MAGIC 0X56774100

typedef struct{
	uint64 time_acc;
	uint32 magic;
	uint32 time_base;
}RTC_TIMER_DEMO;

int mqtt_send = 0;

struct msg{
	char ssid[10];
	char pwd[10];
	int num;
};

const char led_on[] = {"breaker:open"};
const char led_off[] = {"breaker:close"};

extern uint8_t data[5];

ETSTimer send_timer1;

void send_data(char *data){
	int i=0;
	os_printf("the uart data is :%s", data);
}

//void ICACHE_FLASH_ATTR user_check_sntp_stamp(void *arg){
//	uint32 current_stamp;
//	current_stamp = sntp_get_current_timestamp();
//	if(current_stamp == 0){
//		os_timer_arm(&sntp_timer, 100, 0);
//	}
//	else{
//		os_timer_disarm(&sntp_timer);
//		os_printf("sntp: %d, %s \r\n",current_stamp,sntp_get_real_time(current_stamp));
//		if(time_sntp == 1){
//			espconn_secure_ca_enable(0x01,0x3B);
//			espconn_secure_cert_req_enable(0x01,0x3A);
//			MQTT_Connect(&mqttClient);
//		}
//	}
//}

void ICACHE_FLASH_ATTR send_tmp(void *arg)
{
	uint8_t tmp;
	char str_tmp[8] = {0};
	MQTT_Client* client = (MQTT_Client*)arg;

	dht_getdat();
	tmp = data[0] + data[1] + data[2] + data[3];
	if(data[4] == tmp)
	{
		os_sprintf(str_tmp, "tmp:%d,hum:%d", data[2], data[0]);
		os_printf("pub msg : %s\n", str_tmp);
		MQTT_Publish(client, "OID/th", str_tmp, os_strlen(str_tmp), 0, 1);
	}
}

void wifiConnectCb(uint8_t status)
{
	if((status == STATION_GOT_IP)){
		espconn_secure_ca_enable(0x01,0x3B);
		espconn_secure_cert_req_enable(0x01,0x3A);
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
//	MQTT_Subscribe(client, "meijie/b", 0);
//	MQTT_Subscribe(client, "/mqtt/topic/1", 1);
//	MQTT_Subscribe(client, "/mqtt/topic/2", 2);
//
//	MQTT_Publish(client, "/mqtt/topic/0", "hello0", 6, 0, 0);
//	MQTT_Publish(client, "/mqtt/topic/1", "hello1", 6, 1, 0);
//	MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);
	MQTT_Publish(client, "OID/th", "hello0", 6, 0, 0);
	MQTT_Subscribe(client, "OID/hdw", 0);

//	os_timer_disarm(&send_timer1);
//	os_timer_setfn(&send_timer1, (os_timer_func_t *)send_tmp, client);
//	os_timer_arm(&send_timer1, 3000, 1);

	INFO("OK\r\n");

}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
//	int n;

//	char *topicBuf = (char*)os_zalloc(topic_len+1),
//			*dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

//	os_memcpy(topicBuf, topic, topic_len);
//	topicBuf[topic_len] = 0;
//
//	os_memcpy(dataBuf, data, data_len);
//	dataBuf[data_len] = 0;

//	n = atoi(dataBuf);
//	pwm_set_duty(n,0);
//	pwm_start();

	if(os_strstr(data, led_on))
	{
		gpio_output_set(BIT4, 0, BIT4, 0);
		gpio_output_set(BIT5, 0, BIT5, 0);
		MQTT_Publish(client, "OID/th", "open", 4, 0, 0);
	}
	else if(os_strstr(data, led_off))
	{
		gpio_output_set(0, BIT4, 0, BIT4);
		gpio_output_set(0, BIT5, 0, BIT5);
		MQTT_Publish(client, "OID/th", "close", 5, 0, 0);
	}


//	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
//	INFO("data len is : %d , data is :%d, n is :%d\r\n",data_len, (int)*data,n);
//
//	os_free(topicBuf);
//	os_free(dataBuf);
}

void rtc_count(){
	RTC_TIMER_DEMO rtc_time;
	static uint8 cnt = 0;
	system_rtc_mem_read(64, &rtc_time, sizeof(rtc_time));

	if(rtc_time.magic != RTC_MAGIC){
		os_printf("rtc time init ... \r\n");
		rtc_time.magic = RTC_MAGIC;
		rtc_time.time_acc = 0;
		rtc_time.time_base = system_get_rtc_time();
		os_printf("time base : %d\r\n",rtc_time.time_base);
	}

	os_printf("rtc time test:\r\n");

	uint32 rtc_t1, rtc_t2;
	uint32 st1, st2;
	uint32 cal1, cal2;

	rtc_t1 = system_get_rtc_time();
	st1 = system_get_time();
	cal1 = system_rtc_clock_cali_proc();
	os_delay_us(300);

	st2 = system_get_time();
	rtc_t2 = system_get_rtc_time();
	cal2 = system_rtc_clock_cali_proc();

	os_printf("=================================\r\n");

	rtc_time.time_acc += ( ((uint64)(rtc_t2 - rtc_time.time_base)) * ((uint64)((cal2 * 1000) >> 12)) );
	os_printf("rtc time acc :%lld \r\n", rtc_time.time_acc);

	rtc_time.time_base = rtc_t2;
	system_rtc_mem_write(64, &rtc_time, sizeof(rtc_time));
	os_printf("----------------\r\n");

	WIFI_Connect("xtyk", "xtyk88888", wifiConnectCb);
}

void flash_w_r()
{
	uint32 src_addr = 0x100*SPI_FLASH_SEC_SIZE;
	SpiFlashOpResult ret=0;
	struct msg info_m, info_m2;

	info_m.num = 5;
    sprintf(info_m.ssid, "abc");
    sprintf(info_m.pwd, "mei");
	os_printf("the size is %d\n", sizeof(info_m));

	ret = spi_flash_erase_sector(0x100);
	if(ret == SPI_FLASH_RESULT_OK)
		os_printf("erase ok: %d\n", ret);

	ret = spi_flash_write(src_addr, (uint32 *)&info_m, sizeof(info_m));
	if(ret != SPI_FLASH_RESULT_OK)
		os_printf("write error: %d\n", ret);
	else
		os_printf("write data is ok\n");

	ret = spi_flash_read(src_addr, (uint32 *)&info_m2, sizeof(info_m2));
	if(ret == SPI_FLASH_RESULT_OK)
	{
		os_printf("read success!\n");
		os_printf("the num is : %d\n", info_m2.num);
		os_printf("the ssid is : %s\n", info_m2.ssid);
		os_printf("the pwd is : %s\n", info_m2.pwd);
	}
	else
	{
		os_printf("read fail :%d\n", ret);
	}
}

void user_rf_pre_init(void)
{
}

struct esp_platform_saved_param {
    uint8 devkey[40];
    uint8 token[40];
    uint8 activeflag;
    uint8 pad[3];
};
struct esp_platform_saved_param esp_param;

void user_init(void)
{
/**************************************************************************************************************************/
//	uint64 ret=50;

	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000);

//pwm set,when esp8266 received the number , like 1000, the gpio12 will be set 1000 as pwm_output , the message only need the number.
//	uint32 pwm_duty_init[PWM_CHANNEL] = {0};
//	uint32 io_info[][3] = {   {PWM_0_OUT_IO_MUX,PWM_0_OUT_IO_FUNC,PWM_0_OUT_IO_NUM},
//		                      {PWM_1_OUT_IO_MUX,PWM_1_OUT_IO_FUNC,PWM_1_OUT_IO_NUM},
//		                      {PWM_2_OUT_IO_MUX,PWM_2_OUT_IO_FUNC,PWM_2_OUT_IO_NUM}
//		                      };
//	pwm_init(1000,pwm_duty_init,3,io_info);
//	pwm_set_duty(10000,0);
//    pwm_start();

//	CFG_Load();
//	ret = system_mktime(2016,7,12,10,10,10);
//	INFO("\r\nret is :%lld\r\n",ret);
	//MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	//MQTT_InitConnection(&mqttClient, "192.168.11.122", 1880, 0);

	// no secure
	MQTT_InitConnection(&mqttClient, "101.200.207.137", 1883, 0);
	// ssl secure
	//MQTT_InitConnection(&mqttClient, "101.200.207.137", 8883, 1);

	MQTT_InitClient(&mqttClient, "10095", "10066", "123456", 30, 1);
	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 2, 1);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect("xtyk", "xtyk88888", wifiConnectCb);

	dht_init();

//	ip_addr_t *addr = (ip_addr_t *)os_zalloc(sizeof(ip_addr_t));
//	sntp_setservername(0, "us.pool.ntp.org"); // set server 0 by domain name
//	sntp_setservername(1, "ntp.sjtu.edu.cn"); // set server 1 by domain name
//	ipaddr_aton("210.72.145.44", addr);
//	sntp_setserver(2, addr); // set server 2 by IP address
//	sntp_init();
//	os_free(addr);
//
//	os_timer_disarm(&sntp_timer);
//	os_timer_setfn(&sntp_timer, (os_timer_func_t *)user_check_sntp_stamp, NULL);
//	os_timer_arm(&sntp_timer, 100, 0);

//	rtc_count();
//	os_timer_disarm(&rtc_test_t);
//	os_timer_setfn(&rtc_test_t, rtc_count, NULL);
//	os_timer_arm(&rtc_test_t, 10000, 1);

	INFO("\r\nSystem started ...\r\n");
	INFO("SDK version:%s\n", system_get_sdk_version());

	//flash_w_r();
//	gpio16_output_conf();
//	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
//	gpio_output_set(BIT12,0,BIT12,0);
//	ret = GPIO_INPUT_GET(12);
//	os_printf("gpio get is %d\n", ret);
/***************************************************************************************************************************/



	//system_param_load(0x7D, 0, &esp_param, sizeof(esp_param));
//	struct upgrade_server_info *server;
//	system_upgrade_start(server);

}
