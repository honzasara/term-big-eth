#include <ModbusMaster.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <UIPEthernet.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <RTClib.h>
#include <saric_ds2482.h>
#include <ow.h>
#include <EEPROM.h>
#include <printf.h>

#include "saric_utils.h"
#include "saric_thermostat.h"
#include "saric_tds_function.h"
#include "saric_mqtt_network.h"

#define PERIOD_50_MS 50

#define OUTPUT_REAL_MODE_NONE 0
#define OUTPUT_REAL_MODE_STATE 1
#define OUTPUT_REAL_MODE_PWM 2
#define OUTPUT_REAL_MODE_DELAY 3

#define OUTPUT_TYPE_HW 2
#define OUTPUT_TYPE_RS485 1

#define TYPE_FREE 0
#define TYPE_THERMCTL 1
#define TYPE_TERMBIG 2

#define HW_ONEWIRE_MAXROMS 32
#define HW_ONEWIRE_MAXDEVICES 32
#define MAX_OUTPUT 32

#define MAX_AVG_TEMP 10



#define POWER_OUTPUT_ERR 255
#define POWER_OUTPUT_OFF 254
#define POWER_OUTPUT_NEGATE 253
#define POWER_OUTPUT_ON 252

#define POWER_OUTPUT_HEAT_MAX 10
#define POWER_OUTPUT_COOL_MAX 11
#define POWER_OUTPUT_FAN_MAX 12





#define bootloader_tag 0
#define time_offset 1
#define set_default_values 90


#define my_serial_1_mode 92
#define my_serial_1_speed_low 93
#define my_serial_1_speed_high 94

#define my_serial_2_mode 95
#define my_serial_2_speed_low 96
#define my_serial_2_speed_high 97

#define my_rs_1_id 98
#define my_rs_2_id 99






#define output0 2200




#define pin_rs485_1 12

#define LED 0

#define ETH_RST 18
#define ETH_INT 23

#define SER1  24
#define SCLK  26
#define PCLK  27
#define OE 25

#define MAX_TEMP_BUFFER 32
#define UART_RECV_MAX_SIZE 64

StaticJsonDocument<256> doc;


#define SELFTEST_ERR_RTC 0
#define SELFTEST_ERR_NTP 1
#define SELFTEST_MQTT_LINK 6



typedef struct struct_output
{
  uint8_t used;
  char name[8];
  uint8_t outputs;
  uint8_t mode_enable;
  uint8_t mode_now;
  uint16_t period;
  uint16_t period_timer;
  uint8_t state;
  uint8_t state_timer;
  uint8_t id;
  uint8_t type;
};

uint8_t output_state[MAX_OUTPUT];
uint8_t output_state_timer[MAX_OUTPUT];
uint8_t output_mode_now[MAX_OUTPUT];
uint16_t output_period_timer[MAX_OUTPUT];


/*
   usporadani pameti output
   0..9 -- nazev
   10 - used - vystup je aktivni/neni aktivni
   11 - outputs - 0..7 vystupu, bitove pole
   12 - mode_enable - jake jsou poveleny mody pro dany vystup
      - mode_now - aktualni nastaveny mod
      - period_timer - zakladni casovy interval pro pwm
   14 - low period - pro pwm
   15 - high period
      - state - v jakem nastavenem stavu vystup je. 0 / 1, pwm cislo
   16 - type - HW, VIRTUAL, RS485....
   17 - virtualni id
*/



ModbusMaster node;




RTC_DS1307 rtc;
DateTime now;
EthernetClient ethClient;
EthernetUDP udpClient;
PubSubClient mqtt_client(ethClient);
NTPClient timeClient(udpClient);

long load = 0;
long load_max = 0;
long load_min = 0xffffffff;

volatile uint8_t rsid_1;
volatile uint8_t rsid_2;
long uptime = 0;
long  milis = 0;
long  milis_005s = 0;
long  milis_1s = 0;
long  milis_10s = 0;

uint8_t start_2_at = 0;
uint8_t re_2_at = 0;
uint8_t r_2_id = 0;
char uart_2_recv[UART_RECV_MAX_SIZE];

uint8_t serial_1_mode = 0;
uint8_t serial_2_mode = 0;

//uint8_t timer2_counter;
//uint16_t timer3_counter;




uint8_t selftest_data = 0;

uint8_t output_last = 255;



const char thermctl_header_in[] PROGMEM  = "/thermctl-in/";
const char thermctl_header_out[] PROGMEM  = "/thermctl-out/";
const char termbig_header_in[] PROGMEM  = "/termbig-in/";
const char termbig_header_out[] PROGMEM  = "/termbig-out/";
const char global_time_set[] PROGMEM = "global/time/set";
const char global_time_ntp[] PROGMEM = "global/time/ntp";
const char global_time_offset[] PROGMEM = "global/time/offset";
const char thermctl_subscribe[] PROGMEM = "/ctl/thermctl/subscribe";
const char termbig_subscribe[] PROGMEM = "/ctl/termbig/subscribe";
const char termbig_virtual_output[] PROGMEM = "/termbig-out/virtual-output";

void(* resetFunc) (void) = 0; //declare reset function @ address 0


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************************************************************************************************************************************//
/// funkce pro obsluhu seriove rs485 linky
void new_parse_at(char *input, char *out1, char *out2)
{
  uint8_t count = 0;
  uint8_t q = 0;
  out1[0] = 0;
  out2[0] = 0;

  while ( (count < strlen(input)) && (input[count] != ',') && (q < MAX_TEMP_BUFFER - 1))
  {
    out1[q] = input[count];
    out1[q + 1] = 0;
    q++;
    count++;
  }

  count++;
  q = 0;
  while ((count < strlen(input)) && (q < MAX_TEMP_BUFFER - 1) )
  {
    out2[q] = input[count];
    out2[q + 1] = 0;
    q++;
    count++;
  }
}

//////////////////////////////////////////////////////////////////////////
void send_at_2(uint8_t id, char *cmd, char *args)
{
  char tmp1[MAX_TEMP_BUFFER];
  char tmp2[8];
  tmp1[0] = 0;
  tmp2[0] = 0;
  strcpy(tmp1, "at+");
  itoa(id, tmp2, 10);
  strcat(tmp1, tmp2);
  strcat(tmp1, ",");
  strcat(tmp1, cmd);
  if (strlen(args) > 0)
  {
    strcat(tmp1, ",");
    strcat(tmp1, args);
  }
  strcat(tmp1, ";");
  Serial1.println(tmp1);
  Serial1.flush();
}
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void read_at_2(char *input)
{
  uint8_t c;
  while (Serial1.available() > 0)
  {
    c = Serial1.read();
    //////////
    if ( re_2_at > 0 && re_2_at < (UART_RECV_MAX_SIZE - 1) )
    {
      if (c == ';')
      {
        start_2_at = 255;
        re_2_at = 0;
        break;
        goto endloop_2;
      }
      input[re_2_at - 1] = c;
      input[re_2_at] = 0;
      re_2_at++;
    };
    //////////
    if (start_2_at == 2)
      if (c == '+')
        start_2_at = 3;
      else
        start_2_at = 0;
    //////////
    if (start_2_at == 1)
      if ( c == 't')
        start_2_at = 2;
      else
        start_2_at = 0;
    //////////
    if (start_2_at == 0)
      if (c == 'a')
        start_2_at = 1;
    //////////
    if (start_2_at == 3)
    {
      re_2_at = 1;
      start_2_at = 4;
    }
    //////////
endloop_2:;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// obsluha modbus sbernice ///
void ModBus_preTransmission()
{
  digitalWrite(pin_rs485_1, 1);
  digitalWrite(pin_rs485_1, 1);
}

void ModBus_postTransmission()
{
  digitalWrite(pin_rs485_1, 0);
  digitalWrite(pin_rs485_1, 0);
}

/*
 * ********** Ziskani/Nastaveni rychlosti seriove linky *************
*/
uint16_t serial1_get_speed(void)
{
  uint16_t speed = 9600;
  speed = EEPROM.read(my_serial_1_speed_high) << 8 + EEPROM.read(my_serial_1_speed_low);
  return speed;
}
void serial1_set_speed(uint16_t speed)
{
  EEPROM.write(my_serial_1_speed_high, speed >> 8);
  EEPROM.write(my_serial_1_speed_low, (speed & 0xff));
}

uint16_t serial2_get_speed(void)
{
  uint16_t speed = 9600;
  speed = EEPROM.read(my_serial_2_speed_high) << 8 + EEPROM.read(my_serial_2_speed_low);
  return speed;
}

void serial2_set_speed(uint16_t speed)
{
  EEPROM.write(my_serial_2_speed_high, speed >> 8);
  EEPROM.write(my_serial_2_speed_low, (speed & 0xff));
}




///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
uint8_t time_get_offset(void)
{
  return EEPROM.read(time_offset);
}

void time_set_offset(uint8_t offset)
{
  EEPROM.write(time_offset, offset);
}

///////////////////////////////////////////////////////////////////////////////////////////
void output_get_name(uint8_t idx, char *name)
{
  for (uint8_t i = 0; i < 9; i++)
  {
    name[i] = EEPROM.read(output0 + (idx * 20) + i);
    name[i+1] = 0;
  }
}

void output_set_name(uint8_t idx, char *name)
{
  for (uint8_t i = 0; i < 10; i++)
  {
    EEPROM.write((output0 + (idx * 20) + i), name[i]);
  }
}
///////////////////
void output_get_variables(uint8_t idx, uint8_t *used, uint8_t *mode_enable, uint8_t *type, uint8_t *outputs, uint8_t *id, uint16_t *period)
{
  *used = EEPROM.read((uint16_t) output0 + (idx * 20) + 10);
  *type = EEPROM.read((uint16_t) output0 + (idx * 20) + 16);
  *mode_enable = EEPROM.read((uint16_t) output0 + (idx * 20) + 12);
  *outputs = EEPROM.read((uint16_t) output0 + (idx * 20) + 11);
  *id = EEPROM.read((uint16_t) output0 + (idx * 20) + 17);
  *period = (EEPROM.read((uint16_t) output0 + (idx * 20) + 15) << 8) + EEPROM.read((uint16_t) output0 + (idx * 20) + 14);

}

void output_set_variables(uint8_t idx, uint8_t used, uint8_t mode_enable, uint8_t type, uint8_t outputs, uint8_t id, uint16_t period)
{
  EEPROM.write((uint16_t)(output0 + (idx * 20) + 10), used);
  EEPROM.write((uint16_t)(output0 + (idx * 20) + 16), type);
  EEPROM.write((uint16_t)(output0 + (idx * 20) + 12), mode_enable);
  EEPROM.write((uint16_t)(output0 + (idx * 20) + 11), outputs);
  EEPROM.write((uint16_t)(output0 + (idx * 20) + 17), id);
  EEPROM.write((uint16_t)(output0 + (idx * 20) + 14), period);
  EEPROM.write((uint16_t)(output0 + (idx * 20) + 15), (period >> 8) & 0xff);
}
////////////////
void output_get_all(uint8_t idx, struct_output *output)
{
  output_get_name(idx, output->name);
  output_get_variables(idx, &output->used, &output->mode_enable, &output->type, &output->outputs, &output->id, &output->period);
  output->state = output_state[idx];
  output->mode_now = output_mode_now[idx];
  output->period_timer = output_period_timer[idx];
}

void output_get_period_for_pwm(uint8_t idx, uint16_t *period, uint16_t *period_now)
{
  *period = (EEPROM.read((uint16_t) output0 + (idx * 20) + 15) << 8) + EEPROM.read((uint16_t) output0 + (idx * 20) + 14);
  *period_now = output_mode_now[idx];
}

void output_set_state(uint8_t idx, uint8_t state)
{
  output_state[idx] = state;
}

void output_inc_state_timer(uint8_t idx)
{
  output_state_timer[idx]++;
}


void output_set_mode_now(uint8_t idx, uint8_t mode)
{
  output_mode_now[idx] = mode;
}

void output_inc_period_timer(uint8_t idx)
{
  output_period_timer[idx]++;
}
void output_reset_period_timer(uint8_t idx)
{
  output_period_timer[idx] = 0;
}

void output_set_all(uint8_t idx, struct_output output)
{
  output_set_name(idx, output.name);
  output_set_variables(idx, output.used, output.mode_enable, output.type, output.outputs, output.id, output.period);
  output_set_state(idx, output.state);
}

//////////
void output_update_used(uint8_t idx, uint8_t used)
{
  struct_output output;
  output_get_all(idx, &output);
  output.used = used;
  output_set_all(idx, output);
}
void output_update_mode_enable(uint8_t idx, uint8_t mode)
{
  struct_output output;
  output_get_all(idx, &output);
  output.mode_enable = mode;
  output_set_all(idx, output);
}
void output_update_type(uint8_t idx, uint8_t type)
{
  struct_output output;
  output_get_all(idx, &output);
  output.type = type;
  output_set_all(idx, output);
}
void output_update_outputs(uint8_t idx, uint8_t outputs)
{
  struct_output output;
  output_get_all(idx, &output);
  output.outputs = outputs;
  output_set_all(idx, output);
}
void output_update_id(uint8_t idx, uint8_t id)
{
  struct_output output;
  output_get_all(idx, &output);
  output.id = id;
  output_set_all(idx, output);
}
void output_update_period(uint8_t idx, uint16_t period)
{
  struct_output output;
  output_get_all(idx, &output);
  output.period = period;
  output_set_all(idx, output);
}
//////////////////////////////////////////////////////////////////////////////////////////////







/////////////////////////////////////////////////
//// /termbig-out/XXXXX/1wire/count
//// /termbig-out/XXXXX/1wire/IDcko/rom
void send_mqtt_onewire(void)
{
  char str_topic[64];
  char hostname[10];
  char payload[64];
  char tmp1[4];
  if (mqtt_client.connected())
  {
    itoa(Global_HWwirenum, payload, 10);
    send_mqtt_general_payload(&mqtt_client, "1wire/count", payload);
    ///
    for (uint8_t i = 0; i < Global_HWwirenum; i++)
    {
      createString(payload, ':', w_rom[i].rom, 8, 16);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "1wire", i, "rom", payload);
      ///
      itoa(w_rom[i].assigned_ds2482, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "1wire", i, "assigned", payload);
    }
  }
}
//// /termbig-out/XXXXX/tds/ID/temp
//// /termbig-out/XXXXX/tds/ID/name
//// /termbig-out/XXXXX/tds/ID/offset
//// /termbig-out/XXXXX/tds/ID/online
//// /termbig-out/XXXXX/tds/ID/rom
//// /termbig-out/XXXXX/tds/ID/period
void send_mqtt_tds(void)
{
  struct_DDS18s20 tds;
  char payload[64];
  char tmp1[4];
  int tt;
  long avg = 0;
  for (uint8_t id = 0; id < HW_ONEWIRE_MAXROMS; id++)
    if (get_tds18s20(id, &tds) == 1)
      if (tds.used == 1)
      {
        tt = status_tds18s20[id].temp / 10;
        itoa(tt, payload, 10);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "temp", payload);
        ///
        avg = 0;
        for (uint8_t c = 0; c < MAX_AVG_TEMP; c++) avg = avg + status_tds18s20[id].average_temp[c];
        avg = avg / MAX_AVG_TEMP;
        avg = avg / 10;
        itoa(avg, payload, 10);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "temp_avg", payload);
        ///
        strcpy(payload, tds.name);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "name", payload);
        tt = tds.offset;
        itoa(tt, payload, 10);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "offset", payload);
        payload[0] = 0;
        createString(payload, ':', tds.rom, 8, 16);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "rom", payload);
        tt = tds.period;
        itoa(tt, payload, 10);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "period", payload);
        tt = (uptime & 0xff) - status_tds18s20[id].period_now;
        itoa(tt, payload, 10);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "start_at", payload);
        itoa(tds.assigned_ds2482, payload, 10);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "assigned", payload);
        if (status_tds18s20[id].online == True)
        {
          strcpy(payload, "true");
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "online", payload);
        }
        else
        {
          strcpy(payload, "false");
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "online", payload);
        }
      }
}

void send_mqtt_outputs(void)
{
  struct_output output;
  char payload[64];
  char tmp1[4];
  int tt;
  for (uint8_t id = 0; id < MAX_OUTPUT; id++)
  {
    output_get_all(id, &output);
    if (output.used != 0)
    {
      strcpy(payload, output.name);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "name", payload);

      tt = output.outputs;
      itoa(tt, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "outputs", payload);

      tt = output.mode_enable;
      itoa(tt, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "mode_enable", payload);

      tt = output.mode_now;
      itoa(tt, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "mode_now", payload);

      tt = output.type;
      itoa(tt, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "type", payload);

      tt = output.id;
      itoa(tt, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "virtual-id", payload);

      tt = output.state;
      itoa(tt, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "state", payload);

      tt = output.used;
      itoa(tt, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "used", payload);

      tt = output.period;
      itoa(tt, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "output", id, "period", payload);
    }
  }
}

void send_mqtt_output_virtual(PubSubClient *mqtt_client_v, const char *topic, const char *payload, uint8_t virtual_id)
{
  char t_topic[64];
  char str1[12];
  strcpy_P(t_topic, termbig_virtual_output);
  itoa(virtual_id, str1, 10);
  strcat(t_topic, "/");
  strcat(t_topic, str1);
  strcat(t_topic, "/");
  strcat(t_topic, topic);
  send_mqtt_payload(mqtt_client_v, t_topic, payload);
}

void send_mqtt_outputs_virtual_id(PubSubClient *mqtt_client_v)
{
  char payload[64];
  char topic[64];
  char name[12];
  char str1[12];
  struct_output output;
  uint8_t tt = 0;
  for (uint8_t id = 0; id < MAX_OUTPUT; id++)
  {
    output_get_all(id, &output);
    if (output.used != 0)
    {
      tt = output.id;
      send_mqtt_output_virtual(mqtt_client_v, "name", output.name, tt);
      /*
      itoa(output.type, str1, 10);
      send_mqtt_output_virtual(mqtt_client_v, "type", str1, tt);
      itoa(output.mode-enable, str1, 10);
      send_mqtt_output_virtual(mqtt_client_v, "mode_enable", str1, tt);
      */
      itoa(output.state, str1, 10);
      send_mqtt_output_virtual(mqtt_client_v, "state", str1, tt);
    }
  }
}



////
//// termbig-out/XXXXX/network/mac
//// termbig-out/XXXXX/network/ip
//// termbig-out/XXXXX/network/netmask
//// termbig-out/XXXXX/network/gw
//// termbig-out/XXXXX/network/dns
//// termbig-out/XXXXX/network/ntp
//// termbig-out/XXXXX/network/mqtt_host
//// termbig-out/XXXXX/network/mqtt_port
//// termbig-out/XXXXX/network/mqtt_user
//// termbig-out/XXXXX/network/mqtt_key
//// termbig-out/XXXXX/network/name
void send_network_config(void)
{
  char payload[20];
  createString(payload, ':', device.mac, 6, 16);
  send_mqtt_general_payload(&mqtt_client, "network/mac", payload);
  ///
  createString(payload, '.', device.myIP, 4, 10);
  send_mqtt_general_payload(&mqtt_client, "network/ip", payload);
  ///
  createString(payload, '.', device.myMASK, 4, 10);
  send_mqtt_general_payload(&mqtt_client, "network/netmask", payload);
  ///
  createString(payload, '.', device.myGW, 4, 10);
  send_mqtt_general_payload(&mqtt_client, "network/gw", payload);
  ///
  createString(payload, '.', device.myDNS, 4, 10);
  send_mqtt_general_payload(&mqtt_client, "network/dns", payload);
  ///
  createString(payload, '.', device.ntp_server, 4, 10);
  send_mqtt_general_payload(&mqtt_client, "network/ntp", payload);
  ///
  createString(payload, '.', device.mqtt_server, 4, 10);
  send_mqtt_general_payload(&mqtt_client, "network/mqtt_host", payload);
  ///
  itoa(device.mqtt_port, payload, 10);
  send_mqtt_general_payload(&mqtt_client, "network/mqtt_port", payload);
  send_mqtt_general_payload(&mqtt_client, "network/mqtt_user", device.mqtt_user);
  send_mqtt_general_payload(&mqtt_client, "network/mqtt_key", device.mqtt_key);
  send_mqtt_general_payload(&mqtt_client, "network/name", device.nazev);
  itoa(time_get_offset(), payload, 10);
  send_mqtt_general_payload(&mqtt_client, "time/ntp_offset", payload);
}
////
////////////////////////////////////////////////////////////////////////////////////////////
void send_mqtt_device_status(void)
{
  char str_topic[64];
  char hostname[10];
  char payload[64];
  if (mqtt_client.connected())
  {
    ///
    strcpy(str_topic, "status/uptime");
    //sprintf(payload, "%d", uptime);
    itoa(uptime, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    strcpy(str_topic, "status/load_min");
    itoa(load_min, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    strcpy(str_topic, "status/load_max");
    itoa(load_max, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    load_max = 0;
    load_min = 0xffffffff;
    ///
    strcpy(str_topic, "status/selftest");
    itoa(selftest_data, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  char str1[64];
  char str2[128];
  char my_payload[128];
  char tmp1[16];
  boolean ret = 0;
  uint8_t cnt = 0;
  uint8_t id = 0;
  uint8_t id_interval = 0;
  struct_DDS18s20 tds;
  char *pch;
  uint8_t active;
  ///for (uint8_t j = 0; j < 128; j++) str2[j] = 0;
  for (uint8_t j = 0; j < 128; j++) my_payload[j] = 0;
  strncpy(my_payload, (char*) payload, length);
  ////
  mqtt_receive_message++;
  active = 0;
  strcpy_P(str1, thermctl_subscribe);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    active = 0;
    for (uint8_t idx = 0; idx < count_know_mqtt; idx++)
      if (strcmp(know_mqtt[idx].device, my_payload) == 0)
      {
        id = idx;
        active = 1;
        break;
      }
    if (active == 1)
    {
      know_mqtt[id].last_update = 0;
    }
    if (active == 0)
    {
      know_mqtt[count_know_mqtt].type = TYPE_THERMCTL;
      know_mqtt[count_know_mqtt].last_update = 0;
      strcpy(know_mqtt[count_know_mqtt].device, my_payload);
      count_know_mqtt++;
    }
  }
  ////
  //// /termbig-in/global/time/set - nastaveni casu. payload json
  strcpy_P(str1, termbig_header_in);
  strcat_P(str1, global_time_set);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    deserializeJson(doc, my_payload);
    JsonObject root = doc.as<JsonObject>();
    if (root.containsKey("year") && root.containsKey("month") && root.containsKey("month") && root.containsKey("hour") && root.containsKey("minute") && root.containsKey("second"))
      rtc.adjust(DateTime(root["year"], root["month"], root["day"], root["hour"], root["minute"], root["second"]));
  }
  //// /thermctl-in/global/time/ntp - jednorazova aktualizace casu z ntp serveru
  strcpy_P(str1, termbig_header_in);
  strcat_P(str1, global_time_ntp);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    createString(tmp1, '.', device.ntp_server, 4, 10);
    timeClient.begin();
    timeClient.setTimeOffset(3600 * time_get_offset());
    timeClient.setPoolServerName(tmp1);
    if (timeClient.update() == false)
    {
      sbi(selftest_data, SELFTEST_ERR_NTP);
    }
    else
    {
      cbi(selftest_data, SELFTEST_ERR_NTP);
      rtc.adjust(DateTime(timeClient.getYear(), timeClient.getMonth() , timeClient.getDate(), timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds()));
    }
    timeClient.end();
  }
  //// /termbig-in/global/time/offset - nastaveni offsetu casu
  strcpy_P(str1, termbig_header_in);
  strcat_P(str1, global_time_offset);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    time_set_offset(atoi(my_payload));
  }


  //// /thermctl-in/XXXX/tds/associate - asociace do tds si pridam mac 1wire - odpoved je pod jakem ID to mam ulozeno
  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/tds/associate");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    id = atoi(my_payload);
    if ( id < Global_HWwirenum)
    {
      for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXDEVICES; idx++)
      {
        get_tds18s20(idx, &tds);
        if (tds.used == 0 && w_rom[id].used == 1)
        {
          tds.used = 1;
          for (uint8_t i = 0; i < 8; i++)
            tds.rom[i] = w_rom[id].rom[i];
          tds.assigned_ds2482 = w_rom[id].assigned_ds2482;
          set_tds18s20(idx, &tds);
          for (uint8_t cnt = 0; cnt < MAX_AVG_TEMP; cnt++)
            status_tds18s20[idx].average_temp[cnt] = 20000;
          break;
        }
      }
    }
    else
    {
      log_error(&mqtt_client, "tds/associate bad id");
    }
  }
  ///
  ///
  //// /termbig-in/XXXX/tds/set/IDcko/name - nastavi cidlu nazev
  //// /termbig-in/XXXX/tds/set/IDcko/offset
  //// /termbig-in/XXXX/tds/set/IDcko/period
  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/tds/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (cnt == 0) id = atoi(pch);
      if (id < HW_ONEWIRE_MAXROMS)
      {
        if ((cnt == 1) && (strcmp(pch, "name") == 0)) tds_set_name(id, my_payload);
        if ((cnt == 1) && (strcmp(pch, "offset") == 0)) tds_set_offset(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "period") == 0)) tds_set_period(id, atoi(my_payload));
      }
      else
      {
        log_error(&mqtt_client, "tds/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  ////////



  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/output/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (cnt == 0) id = atoi(pch);
      if (id < MAX_OUTPUT)
      {
        if ((cnt == 1) && (strcmp(pch, "used") == 0)) output_update_used(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "enable-mode") == 0)) output_update_mode_enable(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "type") == 0)) output_update_type(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "outputs") == 0)) output_update_outputs(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "virtual-id") == 0)) output_update_id(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "name") == 0)) output_set_name(id, my_payload);
        if ((cnt == 1) && (strcmp(pch, "period") == 0)) output_update_period(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "reset_period") == 0))output_reset_period_timer(id);
        if ((cnt == 1) && (strcmp(pch, "direct-output") == 0))shiftout(atoi(my_payload));
      }
      else
      {
        log_error(&mqtt_client, "output/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }

  ////
  //// /termbig-in/XXXX/tds/clear
  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/tds/clear");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    id = atoi(my_payload);
    if (id < HW_ONEWIRE_MAXROMS)
      tds_set_clear(id);
    else
      log_error(&mqtt_client, "tds/clear bad id");
  }
  ////
  //// ziskani nastaveni site
  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/network/get/config");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    send_network_config();
  }
  ////
  /// nastaveni site
  //// thermctl-in/XXXXX/network/set/mac
  //// thermctl-in/XXXXX/network/set/ip
  //// thermctl-in/XXXXX/network/set/netmask
  //// thermctl-in/XXXXX/network/set/gw
  //// thermctl-in/XXXXX/network/set/dns
  //// thermctl-in/XXXXX/network/set/ntp
  //// thermctl-in/XXXXX/network/set/mqtt_host
  //// thermctl-in/XXXXX/network/set/mqtt_port
  //// thermctl-in/XXXXX/network/set/mqtt_user
  //// thermctl-in/XXXXX/network/set/mqtt_key
  //// thermctl-in/XXXXX/network/set/name
  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/network/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
  }
  /////
  //////// ovladani vystupu
  strcpy_P(str1, termbig_header_in);
  strcat(str1, "virtual-output/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      //digitalWrite(LED, 0);
      if (cnt == 0) id = atoi(pch);
      if (cnt == 1) if (strcmp(pch, "state") == 0) output_set_function(id, OUTPUT_REAL_MODE_STATE, atoi(my_payload));
      if (cnt == 1) if (strcmp(pch, "heat/pwm") == 0) output_set_function(id, OUTPUT_REAL_MODE_PWM, atoi(my_payload));
      if (cnt == 1) if (strcmp(pch, "cool/pwm") == 0) output_set_function(id, OUTPUT_REAL_MODE_PWM, atoi(my_payload));
      if (cnt == 1) if (strcmp(pch, "fan/pwm") == 0) output_set_function(id, OUTPUT_REAL_MODE_PWM, atoi(my_payload));
      if (cnt == 1) if (strcmp(pch, "err/pwm") == 0) output_set_function(id, OUTPUT_REAL_MODE_NONE, atoi(my_payload));
      //if (cnt == 1) if (strcmp(pch, "delay") == 0) output_set_function(id, OUTPUT_REAL_MODE_DELAY, atoi(my_payload));
      pch = strtok (NULL, "/");
      cnt++;
    }

  }


  //// thermctl-in/XXXXX/reload
  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/reload");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    log_error(&mqtt_client, "reload ..... ");
    resetFunc();
  }

  //// thermctl-in/XXXXX/bootloader
  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/bootloader");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    log_error(&mqtt_client, "bootloader ..... ");
    EEPROM.write(bootloader_tag, 255);
    wdt_enable(WDTO_1S);
    while (1);
  }

  //// /thermctl-in/XXXXX/reset_default
  strcpy_P(str1, termbig_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/default");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    EEPROM.write(set_default_values, atoi(my_payload));
  }
}

///////////////////
void output_set_function(uint8_t id, uint8_t mode, uint8_t args)
{
  struct_output output;
  char str1[16];
  for (uint8_t idx = 0; idx < MAX_OUTPUT; idx++)
  {
    output_get_all(idx, &output);
    if (output.used != 0 && output.id == id && mode_in_mode_enable(output.mode_enable, mode) == 1)
    {
      if (mode == OUTPUT_REAL_MODE_STATE)
      {
        output_set_state(idx, args);
        output_set_mode_now(idx, mode);
        strcpy(str1, "set-state");
        send_mqtt_output_quick_state(output.id, str1, args);
      }

      if (mode == OUTPUT_REAL_MODE_PWM)
      {
        output_set_state(idx, args);
        output_set_mode_now(idx, mode);
        strcpy(str1, "set-pwm");
        send_mqtt_output_quick_state(output.id, str1, args);
      }
      /*
        if (mode == OUTPUT_REAL_MODE_DELAY)
        {
        output_set_state(idx, args);
        output_set_mode_now(idx, mode);
        strcpy(str1, "delay");
        send_mqtt_output_quick_state(output.id, str1, args);
        }
      */
    }
  }
}


void send_mqtt_output_quick_state(uint8_t idx, char *mode, uint8_t args)
{
  char topic[64];
  char payload[16];
  char str1[8];

  strcpy_P(topic, termbig_virtual_output);
  itoa(idx, str1, 10);
  strcat(topic, str1);
  strcat(topic, "/");
  strcat(topic, mode);
  itoa(args, payload, 10);
  send_mqtt_payload(&mqtt_client, topic, payload);
}

// saric /termbig-out/output/IDX/{pwm|state}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t mode_in_mode_enable(uint8_t mode_enable, uint8_t mode)
{
  uint8_t ret = 0;
  for (uint8_t idx = 0; idx < 8; idx++)
    if  ((mode_enable & (1 << idx)) != 0  && (mode & (1 << idx)) != 0)
    {
      ret = 1;
      break;
    }
  return ret;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/// nastavi registr vystupniho stavu
void phy_set_output(uint8_t bite, uint8_t state)
{
  //printf("bite: %d, state %d\n", bite, state);
  if (state == 0)
    cbi(output_last, bite);
  else
    sbi(output_last, bite);
    
  shiftout(output_last);
}
///// zpetne ziska registr vystupniho stavu
uint8_t phy_get_output(uint8_t bite)
{
  return (output_last & (1 << bite));
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void update_phy_output(void)
{
  struct_output output;
  //printf("here\n");
  
  for (uint8_t idx = 0; idx < MAX_OUTPUT; idx++)
  {
    output_get_all(idx, &output);
    //if (idx == 0)
    //printf("idx %d ; mode-now %d ; type %d\n", idx, output.mode_now, output.type);
    
    if (output.mode_now == OUTPUT_REAL_MODE_STATE)
    {
      if (output.type == OUTPUT_TYPE_HW)
      {
        for (uint8_t oid = 0; oid < 8; oid++)
          if ((output.outputs & (1 << oid)) != 0)
          {
            //printf("output.state %d oid %d\n", output.state, oid);
            if (output.state == POWER_OUTPUT_OFF) phy_set_output(oid, 1);
            if (output.state == POWER_OUTPUT_ERR) phy_set_output(oid, 1);
            if (output.state == POWER_OUTPUT_HEAT_MAX) phy_set_output(oid, 0);
            if (output.state == POWER_OUTPUT_COOL_MAX) phy_set_output(oid, 0);
            if (output.state == POWER_OUTPUT_FAN_MAX) phy_set_output(oid, 0);
            if (output.state == POWER_OUTPUT_NEGATE)
            {
              if (phy_get_output(oid) == 0) 
                {
                  phy_set_output(oid, 1);
                  output.state = POWER_OUTPUT_OFF;
                }
              else 
              {
                phy_set_output(oid, 0);
                output.state = POWER_OUTPUT_ON;
              }
             output_set_all(idx, output); 
            }
          }
      }
    }
    //////
    if (output.mode_now == OUTPUT_REAL_MODE_PWM)
    {
      if (output.type == OUTPUT_TYPE_HW)
      {
        for (uint8_t oid = 0; oid < 8; oid++)
          if ((output.outputs & (1 << oid)) != 0)
          {
            if (output.state > output.state_timer)
              phy_set_output(oid, 0); /// vystupy do zapnuto
            else
              phy_set_output(oid, 1); /// vystupy do vypnuto
          }
      }
    }
    /////
    //// zpozdovac
    /*
      if (output.type == OUTPUT_REAL_MODE_DELAY)
      {
      if (output.type == OUTPUT_TYPE_HW)
      {
      for (uint8_t oid = 0; oid < 8; oid++)
          if (output.outputs & (1 << idx) != 0)
          {
          /// ted akce
          /// zastavit citac
          }
      }
      }
    */
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
boolean mqtt_reconnect(void)
{
  char nazev[10];
  char topic[26];
  boolean ret = true;
  ///  /termbig-in/xxxxxxxx/#
  ///  /termbig-in/global/#
  ///  /termbig-in/power-output/#
  ///  /termbig-in/power-output/#
  device_get_name(nazev);
  mqtt_client.disconnect();
  ret = mqtt_client.connect(nazev);
  if (ret == true)
  {
    strcpy_P(topic, termbig_header_in);
    strcat(topic, nazev);
    strcat(topic, "/#");
    mqtt_client.subscribe(topic);

    strcpy_P(topic, termbig_header_in);
    strcat(topic, "global/#");
    mqtt_client.subscribe(topic);

    strcpy_P(topic, thermctl_subscribe);
    mqtt_client.subscribe(topic);

    strcpy_P(topic, termbig_header_in);
    strcat(topic, "virtual-output/#");
    mqtt_client.subscribe(topic);
  }
  return ret;
}

///////////// kontrolni funkce ///////////////////////////
/// selftest_set_0 - nastaveni chyboveho flagu
/*
   what - jaky chybovy flag
*/
void selftest_set_0(uint8_t what)
{
  sbi(selftest_data, what) ;
}
///
/// selftest_clear_0 - vymazani chyboveho flagu, problem jiz neni
/*
   what - jaky chybovy flag
*/
void selftest_clear_0(uint8_t what)
{
  cbi(selftest_data, what) ;
}
///
/// selftest_get_0 - flag je v jakem stavu
/*
   what - jaky chybovy flag

   return
     ... 0 - neni oznacen jako chybny
     ... neni 0 - chybny flag
*/
uint8_t selftest_get_0(uint8_t what)
{
  return selftest_data & (1 << what);
}


void selftest(void)
{
  if (!rtc.isrunning())
    selftest_set_0(SELFTEST_ERR_RTC);
  else
    selftest_clear_0(SELFTEST_ERR_RTC);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  char tmp1[20];
  char tmp2[20];
  uint8_t itmp;
  boolean bol;

  long milis;

  wdt_enable(WDTO_8S);
  wdt_reset();

  mqtt_set_public_mqtt_client(&mqtt_client);
  fdevopen( &printf_via_mqtt, 0);
  printf_begin();

  struct_DDS18s20 tds;

  pinMode(ETH_RST, OUTPUT);
  digitalWrite(ETH_RST, LOW);
  for (uint16_t i = 0; i < 1024; i++);
  digitalWrite(ETH_RST, HIGH);







  noInterrupts();           // disable all interrupts

  pinMode(SER1, OUTPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(PCLK, OUTPUT);
  pinMode(OE, OUTPUT);
  pinMode(LED, OUTPUT);

  pinMode(pin_rs485_1, OUTPUT);
  digitalWrite(pin_rs485_1, 0 );
  digitalWrite(LED, 1);
  digitalWrite(OE, 0);


  interrupts();
  SPI.begin();
  rtc.begin();





  for (uint8_t inic = 0; inic < 15; inic++)
  {
    wdt_reset();
    if (inic == 0)
    {
      if (EEPROM.read(set_default_values) == 255)
      {
        EEPROM.write(set_default_values, 0);
        for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXDEVICES; idx++)
        {
          get_tds18s20(idx, &tds);
          strcpy(tds.name, "FREE");
          tds.used = 0;
          tds.offset = 0;
          tds.assigned_ds2482 = 255;
          tds.period = 10;
          set_tds18s20(idx, &tds);
        }
        for (uint8_t idx = 0; idx < MAX_OUTPUT; idx++)
        {
          strcpy(tmp1, "OUT");
          itoa(idx, tmp2, 10);
          strcat(tmp1, tmp2);
          output_set_name(idx, tmp1);
          output_set_variables(idx, 0, OUTPUT_REAL_MODE_NONE, 0, 0, 0, PERIOD_50_MS);
        }
        rtc.adjust(DateTime(2020, 3, 4, 17, 14, 0));
        device.mac[0] = 2; device.mac[1] = 44; device.mac[2] = 112; device.mac[3] = analogRead(A4) * analogRead(A7); device.mac[4] = analogRead(A4); device.mac[5] = analogRead(A7);
        uint8_t ip4 = 0;
        uint8_t f = 0;
        while (f == 0)
        {
          ip4 = analogRead(A7);
          if (ip4 > 210 && ip4 < 250) f = 1;
        }
        device.myIP[0] = 192; device.myIP[1] = 168; device.myIP[2] = 2; device.myIP[3] = ip4;
        device.myMASK[0] = 255; device.myMASK[1] = 255; device.myMASK[2] = 255; device.myMASK[3] = 0;
        device.myGW[0] = 192; device.myGW[1] = 168; device.myGW[2] = 2; device.myGW[3] = 1;
        device.myDNS[0] = 192; device.myDNS[1] = 168; device.myDNS[2] = 2; device.myDNS[3] = 1;
        device.mqtt_server[0] = 192; device.mqtt_server[1] = 168; device.mqtt_server[2] = 2; device.mqtt_server[3] = 1;
        device.ntp_server[0] = 192; device.ntp_server[1] = 168; device.ntp_server[2] = 2; device.ntp_server[3] = 1;
        device.mqtt_port = 1883;
        strcpy(device.mqtt_user, "saric");
        strcpy(device.mqtt_key, "no");
        save_setup_network();
        device_set_name("TERMBIG2");
        char hostname[10];
        device_get_name(hostname);
        time_set_offset(2);

        serial1_set_speed(9600);
        serial2_set_speed(9600);
      }
    }
    ///
    if (inic == 1)
    {
      load_setup_network();
    }
    ///
    if (inic == 2)
    {

      for (uint8_t idx = 0; idx < 32; idx++)
      {
        strcpy(know_mqtt[idx].device, "");
        know_mqtt[idx].type = 0;
        know_mqtt[idx].last_update = 0;
      }
      count_know_mqtt = 0;
      //// kvuli lepsimu nabehu pocitani nastavim vychozi hodnotu na 2000 = 20 stupnu
      for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXROMS; idx++)
        for (uint8_t cnt = 0; cnt < MAX_AVG_TEMP; cnt++)
          status_tds18s20[idx].average_temp[cnt] = 20000;

      for (uint8_t idx = 0; idx < MAX_OUTPUT; idx++)
      {
        output_state[idx] = 0;
        output_mode_now[idx] = 0;
        output_state_timer[idx] = 0;
        output_period_timer[idx] = 0;
      }
    }
    ///
    if (inic == 3)
    {
      ds2482_address[0].i2c_addr = 0b0011000;
      ds2482_address[0].HWwirenum = 0;
      ds2482_address[1].i2c_addr = 0b0011011;
      ds2482_address[1].HWwirenum = 0;
      ///
      for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXROMS; idx++ )
      {
        status_tds18s20[idx].wait = false;
        status_tds18s20[idx].period_now = 0;
      }
      ///
      Global_HWwirenum = 0;
      for (uint8_t idx = 0; idx < 2; idx++)
      {
        if (ds2482reset(ds2482_address[idx].i2c_addr) == DS2482_ERR_OK)
        {
        }
        else
        {
        }
        one_hw_search_device(idx);
      }
    }
    ///
    if (inic == 6)
    {
      if (!rtc.isrunning())
      {
        selftest_set_0(SELFTEST_ERR_RTC);
      }
      else
      {
        selftest_clear_0(SELFTEST_ERR_RTC);
      }
    }
    ///
    if (inic == 9)
    {
      if ( Enc28J60Network::linkStatus() == 1)
      {
        //link_status = 1;
      }
      else
      {
        //link_status = 0;
      }
    }
    ///
    if (inic == 8)
    {
      Ethernet.begin(device.mac, device.myIP, device.myDNS, device.myGW, device.myMASK);
    }
    ///
    if (inic == 10)
    {
      mqtt_client.setServer(device.mqtt_server, device.mqtt_port);
      mqtt_client.setCallback(mqtt_callback);
      send_mqtt_set_header(termbig_header_out);
      milis = millis();
      while ((millis() - milis) < 3000 )
      {
        mqtt_client.loop();
        if (mqtt_reconnect() == 0)
        {
          selftest_clear_0(SELFTEST_MQTT_LINK);
          break;
        }
        else
          selftest_set_0(SELFTEST_MQTT_LINK);
      }
    }
    ///
    if (inic == 11)
    {
      rsid_1 = EEPROM.read(my_rs_1_id);
      rsid_2 = EEPROM.read(my_rs_2_id);

      serial_1_mode = EEPROM.read(my_serial_1_mode);
      serial_2_mode = EEPROM.read(my_serial_2_mode);

      Serial.begin(serial1_get_speed());
      Serial1.begin(serial2_get_speed());

      // Modbus slave ID 1
      node.begin(1, Serial);
      // Callbacks allow us to configure the RS485 transceiver correctly
      node.preTransmission(ModBus_preTransmission);
      node.postTransmission(ModBus_postTransmission);
    }

    if (inic == 14)
    {
    }
  }

  milis_10s = millis();
  milis_1s = millis();
  milis_005s = millis();
}

/////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  /* TODO
    1. RS485 full-duplex mod

    2. RS485 falf-duplex mode
     - MODBUS protokol?

    3. vyzkouset cidla ds18s20

    4. umet nastavit sit pres RS485

    ///// myslenky
    - definice output 0..MAX_OUTPUT
     - k vystupu priradim skutecny vystup, mnozinu vystupu
     - mody:
       -- zapnuto/vypnuto - hotovo
       -- pwm - nastavitelna perioda - hotovo
       -- zpozdovaci impuls
       -- casovac
     - nazev
     - pres mqtt reportovat aktualni stav
     - vystup pro topeni/chlazeni/ventilator/neco jineho

    udelat subscribe /thermctl-out/+/power-output/+/"mode"
        - state
        - pwm
        - servo

    zpetna vazba pro termostat

    /termbig/XXXXX/output/set/IDcko/id - nastaveni virtualniho kanalu
  */

  unsigned long load_now, load;


  char str1[MAX_TEMP_BUFFER];
  char str2[MAX_TEMP_BUFFER];
  uint16_t period_now, period;

  wdt_reset();

  load_now = millis();



  //// obsluha prijatych prikazu pro full rs485
  read_at_2(uart_2_recv);
  if (start_2_at == 255)
  {
    start_2_at = 0;
    new_parse_at(uart_2_recv, str1, str2);
  }


  /*
    //// zjisteni stavu sitove linky
    if ( Enc28J60Network::linkStatus() == 1)
      link_status = 1;
    else
      link_status = 0;

    //// zjisteni stavu mqtt klienta
    if (link_status == 1)
    {
  */
  if (!mqtt_client.connected())
    mqtt_reconnect();
  /*
    else
      mqtt_status = 1;
    }
    else
    {
    mqtt_client.disconnect();
    mqtt_status = 0;
    }
  */


  /// zjistim jestli nejsou nove data
  mqtt_client.loop();

  /// pro rizeni PWM vystupu
  for (uint8_t idx = 0; idx < MAX_OUTPUT; idx++)
  {
    output_get_period_for_pwm(idx, &period_now, &period);
    if (period_now > period)
    {
      output_reset_period_timer(idx);
      output_inc_state_timer(idx);
    }
  }



  ////////////////////
  /// kazdych 10sec

  if  ((millis() - milis_10s) >= 10000)
  {
    milis_10s += 10000;
    selftest();
    send_mqtt_onewire();
    send_mqtt_status(&mqtt_client);
    send_mqtt_device_status();
    send_mqtt_tds();
    send_mqtt_outputs();
    ///
  }
  ////////////////////
  /// kazdych 10ms
  if ((millis() - milis_005s) >= 100 )
  {
    milis_005s += 100;
    update_phy_output();
  }
  ////////////////////
  /// kazdou 1sec
  if ((millis() - milis_1s) >= 1000)
  {
    milis_1s += 1000;
    now = rtc.now();
    uptime++;
    if (uptime % 2)
      digitalWrite(LED, 1);
    else
      digitalWrite(LED, LOW);

    mereni_hwwire(uptime);

    strcpy_P(str2, termbig_subscribe);
    device_get_name(str1);
    send_mqtt_payload(&mqtt_client, str2, str1);

    update_know_mqtt_device();
    send_mqtt_outputs_virtual_id(&mqtt_client);
  }


  load = millis() - load_now;
  if (load < load_min) load_min = load;
  if (load > load_max) load_max = load;

}








///////////////////////////////////////////////////////////////////
void shiftout(uint8_t data)
{
  digitalWrite(PCLK, 0);
  uint8_t i;
  for (i = 0; i < 8; i++)  {
    digitalWrite(SCLK, LOW);
    digitalWrite(SER1, !!((data) & (1 << i)) );
    digitalWrite(SCLK, HIGH);
  }
  digitalWrite(PCLK, 1);
}
//////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/// obsuha preruseni

/*
  ISR(TIMER2_OVF_vect)
  {
  uint8_t backup = SREG;
  timer2_counter++;
  if (timer2_counter < 30)
    timer2_counter = 0;
  load2 = load_2;
  load_2 = 0;
  SREG = backup;
  }

  ISR(TIMER3_OVF_vect)
  {
  uint8_t backup = SREG;
  TCNT3 = timer3_counter;   // preload timer
  milis++;

  for (uint8_t idx = 0; idx < MAX_OUTPUT; idx++)
    output_inc_period_timer(idx);

  SREG = backup;
  }

*/
/////////////////////////////////////////////////////////
