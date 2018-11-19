//Copyright 2015, 2018 <>< Charles Lohr, Adam Feinstein see LICENSE file.

/*==============================================================================
 * Includes
 *============================================================================*/

#include "mem.h"
#include "c_types.h"
#include "user_interface.h"
#include "ets_sys.h"
#include "uart.h"
#include "osapi.h"
#include "espconn.h"
#include "esp82xxutil.h"
#include "commonservices.h"
#include "vars.h"
#include <espnow.h>
#include "ws2812_i2s.h"
#include <mdns.h>
#include <string.h>

/*==============================================================================
 * Process Defines
 *============================================================================*/

#define procTaskPrio        0
#define procTaskQueueLen    1
os_event_t    procTaskQueue[procTaskQueueLen];

const int devid = 1;

/*==============================================================================
 * Variables
 *============================================================================*/

static os_timer_t some_timer;
static struct espconn *pUdpServer;
usr_conf_t * UsrCfg = (usr_conf_t*)(SETTINGS.UserData);

/*==============================================================================
 * Functions
 *============================================================================*/

uint32_t EHSVtoHEX( uint8_t hue, uint8_t sat, uint8_t val )
{
	#define SIXTH1 43
	#define SIXTH2 85
	#define SIXTH3 128
	#define SIXTH4 171
	#define SIXTH5 213

	uint16_t or = 0, og = 0, ob = 0;

	hue -= SIXTH1; //Off by 60 degrees.

	//TODO: There are colors that overlap here, consider 
	//tweaking this to make the best use of the colorspace.

	if( hue < SIXTH1 ) //Ok: Yellow->Red.
	{
		or = 255;
		og = 255 - ((uint16_t)hue * 255) / (SIXTH1);
	}
	else if( hue < SIXTH2 ) //Ok: Red->Purple
	{
		or = 255;
		ob = (uint16_t)hue*255 / SIXTH1 - 255;
	}
	else if( hue < SIXTH3 )  //Ok: Purple->Blue
	{
		ob = 255;
		or = ((SIXTH3-hue) * 255) / (SIXTH1);
	}
	else if( hue < SIXTH4 ) //Ok: Blue->Cyan
	{
		ob = 255;
		og = (hue - SIXTH3)*255 / SIXTH1;
	}
	else if( hue < SIXTH5 ) //Ok: Cyan->Green.
	{
		og = 255;
		ob = ((SIXTH5-hue)*255) / SIXTH1;
	}
	else //Green->Yellow
	{
		og = 255;
		or = (hue - SIXTH5) * 255 / SIXTH1;
	}

	uint16_t rv = val;
	if( rv > 128 ) rv++;
	uint16_t rs = sat;
	if( rs > 128 ) rs++;

	//or, og, ob range from 0...255 now.
	//Need to apply saturation and value.

	or = (or * val)>>8;
	og = (og * val)>>8;
	ob = (ob * val)>>8;

	//OR..OB == 0..65025
	or = or * rs + 255 * (256-rs);
	og = og * rs + 255 * (256-rs);
	ob = ob * rs + 255 * (256-rs);
//printf( "__%d %d %d =-> %d\n", or, og, ob, rs );

	or >>= 8;
	og >>= 8;
	ob >>= 8;

	return or | (og<<8) | ((uint32_t)ob<<16);
}


/**
 * This task is called constantly. The ESP can't handle infinite loops in tasks,
 * so this task will post to itself when finished, in essence looping forever
 *
 * @param events unused
 */
static void ICACHE_FLASH_ATTR procTask(os_event_t *events)
{
	CSTick( 0 );

	// Post the task in order to have it called again
	system_os_post(procTaskPrio, 0, 0 );
}

struct cnespsend
{
	uint32_t code;
	uint32_t op;
	uint32_t param1;
	uint32_t param2;
	uint32_t param3;
	uint8_t payload[12*3];
}  __attribute__ ((aligned (1))) __attribute__((packed));

/**
 * This is a timer set up in user_main() which is called every 100ms, forever
 * @param arg unused
 */
static void ICACHE_FLASH_ATTR timer100ms(void *arg)
{
	struct cnespsend thisesp;

	memset( &thisesp, 0, sizeof(thisesp) );
	thisesp.code = 0xbeefbeef;
	thisesp.op = 3;
	thisesp.param1 = 20;
	thisesp.param2 = 2000;
	thisesp.param3 = 50;
	int i;
	for( i = 0; i < 12; i++ )
	{
		uint32_t color = EHSVtoHEX( i*20, 255, 255 );
		thisesp.payload[i*3+0] = color>>8;
		thisesp.payload[i*3+1] = color;
		thisesp.payload[i*3+2] = color>>16;
	}

	espNowSend( &thisesp, sizeof(thisesp) );
	CSTick( 1 ); // Send a one to uart
}

/**
 * UART RX handler, called by the uart task. Currently does nothing
 *
 * @param c The char received on the UART
 */
void ICACHE_FLASH_ATTR charrx( uint8_t c )
{
	//Called from UART.
}

/**
 * This is called on boot for versions ESP8266_NONOS_SDK_v1.5.2 to
 * ESP8266_NONOS_SDK_v2.2.1. system_phy_set_rfoption() may be called here
 */
void user_rf_pre_init(void)
{
	; // nothing
}

/**
 * Required function as of ESP8266_NONOS_SDK_v3.0.0. Must call
 * system_partition_table_regist(). This tries to register a few different
 * partition maps. The ESP should be happy with one of them.
 */
void ICACHE_FLASH_ATTR user_pre_init(void)
{
	LoadDefaultPartitionMap();
}




uint8_t lbuf[12*3];

/**
 * This callback function is called whenever an ESP-NOW packet is received
 *
 * @param mac_addr The MAC address of the sender
 * @param data     The data which was received
 * @param len      The length of the data which was received
 */
void ICACHE_FLASH_ATTR espNowRecvCb(uint8_t* mac_addr, uint8_t* data, uint8_t len)
{
    // Buried in a header, goes from 1 (far away) to 91 (practically touching)
    uint8_t rssi = data[-51];

	struct cnespsend * d = (struct cnespsend*)data;


	if( d->code != 0xbeefbeef ) return;
	switch( d->op )
	{
	case 0:
		//printf( "Pushing %p %p\n", d->payload, data );
		ws2812_push( d->payload, 12*3 );
		break;
	case 1:
		ws2812_push( d->payload + 12*3*devid, 12*3 );
		break;
	case 2:
		if( d->param1 == devid )
			ws2812_push( d->payload, 12*3 );
		break;
	case 3:
	case 4:
		{
			int r = (d->op==3)?rssi:d->param3;
			int p1 = d->param1;
			int p2 = d->param2;

			if( p2 == 0 ) { printf( "NO P2\n" ); return; }

			int m = ((r - p1) * p2)>>8;

			int hue = m;
			if( hue < 0 ) hue = 0;
			if( hue > 140 ) hue = 140;

			int sat = ( 140+128 - m ) * 2;
			if( sat < 0 ) sat = 0;
			if( sat > 255 ) sat = 255;
			int val = (m + 128) * 2;
			if( val < 0 ) val = 0;
			if( val > 255 ) val = 255;
			uint32_t color = EHSVtoHEX( hue, sat, val );
			int i;
			for( i = 0; i < 12; i++ )
			{
				lbuf[i*3+0] = color>>8;
				lbuf[i*3+1] = color;
				lbuf[i*3+2] = color>>16;
			}
			ws2812_push( lbuf, 12*3 );
		}
		break;
	}

#if 0
    // Debug print the received payload
    char dbg[256] = {0};
    char tmp[8] = {0};
    int i;
    for (i = 0; i < len; i++)
    {
        ets_sprintf(tmp, "%02X ", data[i]);
        strcat(dbg, tmp);
    }
    os_printf("%s, MAC [%02X:%02X:%02X:%02X:%02X:%02X], RSSI [%d], Bytes []\r\n",
              __func__,
              mac_addr[0],
              mac_addr[1],
              mac_addr[2],
              mac_addr[3],
              mac_addr[4],
              mac_addr[5],
              rssi );
#endif


}

/**
 * This is a wrapper for esp_now_send. It also sets the wifi power with
 * wifi_set_user_fixed_rate()
 *
 * @param data The data to broadcast using ESP NOW
 * @param len  The length of the data to broadcast
 */
void ICACHE_FLASH_ATTR espNowSend(const uint8_t* data, uint8_t len)
{
	/// This is the MAC address to transmit to for broadcasting
	static const uint8_t espNowBroadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // Call this before each transmission to set the wifi speed
    wifi_set_user_fixed_rate(FIXED_RATE_MASK_ALL, 3); //3 = 11mbit/s B

    // Send a packet
   	esp_now_send((uint8_t*)espNowBroadcastMac, (uint8_t*)data, len);
}

/**
 * This callback function is registered to be called after an ESP NOW
 * transmission occurs. It notifies the program if the transmission
 * was successful or not. It gives no information about if the transmission
 * was received
 *
 * @param mac_addr The MAC address which was transmitted to
 * @param status   MT_TX_STATUS_OK or MT_TX_STATUS_FAILED
 */
void ICACHE_FLASH_ATTR espNowSendCb(uint8_t* mac_addr, uint8_t status)
{
    // os_printf("SEND MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
    //           mac_addr[0],
    //           mac_addr[1],
    //           mac_addr[2],
    //           mac_addr[3],
    //           mac_addr[4],
    //           mac_addr[5]);

    switch(status)
    {
        case 0:
        {
            // os_printf("ESP NOW MT_TX_STATUS_OK\r\n");
            break;
        }
        default:
        {
            os_printf("ESP NOW MT_TX_STATUS_FAILED\r\n");
            break;
        }
    }
}

/**
 * Initialize ESP-NOW and attach callback functions
 */
void ICACHE_FLASH_ATTR espNowInit(void)
{
    if(0 == esp_now_init())
    {
        os_printf("ESP NOW init!\r\n");
        if(0 == esp_now_set_self_role(ESP_NOW_ROLE_COMBO))
        {
            os_printf("set as combo\r\n");
        }
        else
        {
            os_printf("esp now mode set fail\r\n");
        }

        if(0 == esp_now_register_recv_cb(espNowRecvCb))
        {
            os_printf("recvCb registered\r\n");
        }
        else
        {
            os_printf("recvCb NOT registered\r\n");
        }

        if(0 == esp_now_register_send_cb(espNowSendCb))
        {
            os_printf("sendCb registered\r\n");
        }
        else
        {
            os_printf("sendCb NOT registered\r\n");
        }
    }
    else
    {
        os_printf("esp now fail\r\n");
    }
}

/**
 * The default method, equivalent to main() in other environments. Handles all
 * initialization
 */
void ICACHE_FLASH_ATTR user_init(void)
{
	// Initialize the UART
	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	os_printf("\r\nesp82XX Web-GUI\r\n%s\b", VERSSTR);

	//Uncomment this to force a system restore.
	//	system_restore();

	// Load settings and pre-initialize common services
	CSSettingsLoad( 0 );
	CSPreInit();

	ws2812_init();

	// Initialize common settings
	CSInit( 0 );

	// Start MDNS services
	SetServiceName( "espcom" );
	AddMDNSName(    "esp82xx" );
	AddMDNSName(    "espcom" );
	AddMDNSService( "_http._tcp",    "An ESP82XX Webserver", WEB_PORT );
	AddMDNSService( "_espcom._udp",  "ESP82XX Comunication", COM_PORT );
	AddMDNSService( "_esp82xx._udp", "ESP82XX Backend",      BACKEND_PORT );

/*
	struct softap_config {
		uint8 ssid[32];
		uint8 password[64];
		uint8 ssid_len;	// Note: Recommend to set it according to your ssid
		uint8 channel;	// Note: support 1 ~ 13
		AUTH_MODE authmode;	// Note: Don't support AUTH_WEP in softAP mode.
		uint8 ssid_hidden;	// Note: default 0
		uint8 max_connection;	// Note: default 4, max 4
		uint16 beacon_interval;	// Note: support 100 ~ 60000 ms, default 100
	} sap;*/
	struct softap_config sap;
	memset( &sap, 0, sizeof( sap ) );
	sap.ssid_len = 0;
	sap.channel = 1;
	sap.authmode = 0;
	sap.ssid_hidden = 1;
	sap.beacon_interval = 60000;

	wifi_softap_set_config(&sap);

	wifi_set_opmode( 2 );
	wifi_set_channel( 1 );

	// Set timer100ms to be called every 100ms
	os_timer_disarm(&some_timer);
	os_timer_setfn(&some_timer, (os_timer_func_t *)timer100ms, NULL);
	os_timer_arm(&some_timer, 20, 1);

	os_printf( "Boot Ok.\n" );

	espNowInit();

	// Add a process and start it
	system_os_task(procTask, procTaskPrio, procTaskQueue, procTaskQueueLen);
	system_os_post(procTaskPrio, 0, 0 );
}

/**
 * This will be called to disable any interrupts should the firmware enter a
 * section with critical timing. There is no code in this project that will
 * cause reboots if interrupts are disabled.
 */
void ICACHE_FLASH_ATTR EnterCritical(void)
{
	;
}

/**
 * This will be called to enable any interrupts after the firmware exits a
 * section with critical timing.
 */
void ICACHE_FLASH_ATTR ExitCritical(void)
{
	;
}
