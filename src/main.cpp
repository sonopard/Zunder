/* 
 * This file is part of the Zunder project (https://github.com/sonopard/Zunder).
 * Copyright (c) 2021 Benjamin Kunz.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>
#include "errno.h"

#include "Logger.h"

#include "NeoPixelBus.h"


#include "BluetoothSerial.h"

#include "SerialCommands.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Board support package does not have bluetooth built in
#endif

#define LEDS_PIN 3
#define NUM_LEDS 120

Logger log;
char log_str[256];

BluetoothSerial bt_serial;
boolean bt_pairing_request_pending = true;

char sc_buffer[4096];
SerialCommands sc(&bt_serial, sc_buffer, sizeof(sc_buffer), "\n", " ");

NeoBuffer<NeoBufferMethod<NeoGrbFeature>> image(120,1,NULL);
NeoPixelBus<NeoGrbFeature, NeoEsp32I2sSpeedWs2812x> strip(120, LEDS_PIN);


template<typename T_COLOR_FEATURE> class BrightnessShader : public NeoShaderBase
{
public:
  BrightnessShader():
    NeoShaderBase(),
    _brightness(255) // default to full bright
  {}

  // required for a shader object, it will be called for
  // every pixel
  void Apply(uint16_t index, uint8_t* pDest, uint8_t* pSrc)
  {
    // we don't care what the index is so we ignore it
    //
    // to apply our brightness shader, 
    // use the source color, modify, and apply to the destination 
    
    // for every byte in the pixel,
    // scale the source value by the brightness and 
    // store it in the destination byte
    const uint8_t* pSrcEnd = pSrc + T_COLOR_FEATURE::PixelSize;
    while (pSrc != pSrcEnd)
    {
        *pDest++ = (*pSrc++ * (uint16_t(_brightness) + 1)) >> 8;
    }
  }

  // provide an accessor to set brightness
  void setBrightness(uint8_t brightness)
  {
    _brightness = brightness;
    Dirty(); // must call dirty when a property changes
  }

  // provide an accessor to get brightness
  uint8_t getBrightness()
  {
    return _brightness;
  }
  
private:
  uint8_t _brightness;    
};

// create an instance of our shader object with the same feature as our buffer
BrightnessShader<NeoGrbFeature> shader;


enum anim_mode_enum
{ 
    ANIM_OFF, 
    ANIM_GREEN
};
typedef enum anim_mode_enum anim_mode_t;
anim_mode_t anim_mode = ANIM_OFF;

void BTConfirmRequestCallback(uint32_t numVal)
{
  bt_pairing_request_pending = true;
  snprintf(log_str, sizeof(log_str), "BT pairing request %d", numVal); log.log(log.NOTICE, __FILE__, log_str);
}

void BTAuthCompleteCallback(boolean success)
{
  bt_pairing_request_pending = false;
  if (success)
  {
    snprintf(log_str, sizeof(log_str), "BT paired"); log.log(log.NOTICE, __FILE__, log_str);
  }
  else
  {
    snprintf(log_str, sizeof(log_str), "BT pairing rejected"); log.log(log.WARNING, __FILE__, log_str);
  }
}

void sc_cmd_unknown(SerialCommands* sender, const char* cmd)
{
  snprintf(log_str, sizeof(log_str), "sc_cmd_unknown %s", cmd); log.log(log.WARNING, __FILE__, log_str);
	sender->GetSerial()->print("NACK unrecognized command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}

void sc_cmd_anim(SerialCommands* sender)
{
	char* anim_type = sender->Next();
	if (anim_type == NULL)
	{
    snprintf(log_str, sizeof(log_str), "sc_cmd_anim: parameter required"); log.log(log.WARNING, __FILE__, log_str);
		sender->GetSerial()->println("NACK parameter required");
		return;
	}

  if (!strncmp("OFF",anim_type, 3)) {
    anim_mode = ANIM_OFF;
	  sender->GetSerial()->println("ACK");
    return;
	}
  if (!strncmp("GREEN",anim_type, 3)) {
    anim_mode = ANIM_GREEN;
    image.ClearTo(RgbColor(0, 255, 0)); 
    
    image.Render<BrightnessShader<NeoGrbFeature>>(strip, shader);
    strip.Show();
	  sender->GetSerial()->println("ACK");
    return;
	}
  snprintf(log_str, sizeof(log_str), "sc_cmd_anim: unrecognized anim mode"); log.log(log.WARNING, __FILE__, log_str);
  sender->GetSerial()->println("NACK unrecognized anim mode");
}

bool parse_num(const char *str, intmax_t *val) {
    char *temp;
    bool retval = true;
    errno = 0;
    *val = strtoimax(str, &temp, 0);

    if (temp == str || *temp != '\0' ||
        ((*val == INTMAX_MIN || *val == INTMAX_MIN) && errno == ERANGE))
        retval = false;

    return retval;
}

bool parse_uint16(const char *str, uint16_t *val) {
  intmax_t im_val;
  bool retval = true;

  if(!parse_num(str, &im_val))
    retval = false;
  else if (im_val > UINT16_MAX)
    retval = false;
  else if (im_val < 0)
    retval = false;
  else 
    *val = im_val;
  return retval;
}

bool parse_uint8(const char *str, uint8_t *val) {
  intmax_t im_val;
  bool retval = true;

  if(!parse_num(str, &im_val))
    retval = false;
  else if (im_val > UINT8_MAX)
    retval = false;
  else if (im_val < 0)
    retval = false;
  else 
    *val = im_val;
  return retval;
}

void sc_cmd_blit(SerialCommands* sender) {
	char* sc_blit_len = sender->Next();
  uint16_t blit_len = 0;
  char blit_px_str[3] = {0,0,0};

  if (!sc_blit_len) {
    snprintf(log_str, sizeof(log_str), "sc_cmd_anim: blit length required"); log.log(log.WARNING, __FILE__, log_str);
    sender->GetSerial()->println("NACK blit length required");
    return;
  }
  if (!parse_uint16(sc_blit_len, &blit_len)) {
    snprintf(log_str, sizeof(log_str), "sc_cmd_anim: blit length invalid"); log.log(log.WARNING, __FILE__, log_str);
    sender->GetSerial()->println("NACK blit length invalid");
    return;
  }

  char* blit_data = sender->Next();
  if(!blit_data) {
    snprintf(log_str, sizeof(log_str), "sc_cmd_anim: blit data required"); log.log(log.WARNING, __FILE__, log_str);
    sender->GetSerial()->println("NACK blit data required");
    return;
  }
  if((strlen(blit_data) != blit_len) || !(blit_len % 6)) {
    snprintf(log_str, sizeof(log_str), "sc_cmd_anim: blit data length invalid"); log.log(log.WARNING, __FILE__, log_str);
    sender->GetSerial()->println("NACK blit data length invalid");
    return;
  }
  for (int i = 0; i < blit_len/2; i+=2) {
    blit_px_str[0] = blit_data[i];
    blit_px_str[1] = blit_data[i+1];
    RgbColor c = RgbColor();
    uint8_t rbgval;
    if (!parse_uint8(blit_px_str, &rbgval)){
      switch(i%3) {
        case 0:
          c.R = rbgval;
          break;
        case 1:
          c.G = rbgval;
          break;
        case 2:
          c.B = rbgval;
          image.SetPixelColor(i/3, 1, c);
      }
    }
    else
    {
      snprintf(log_str, sizeof(log_str), "sc_cmd_anim: blit data invalid"); log.log(log.WARNING, __FILE__, log_str);
      sender->GetSerial()->println("NACK blit data invalid");
      return;
    }
  }
  snprintf(log_str, sizeof(log_str), "sc_cmd_anim: unrecognized anim mode"); log.log(log.WARNING, __FILE__, log_str);
  sender->GetSerial()->println("NACK unrecognized anim mode");
}

void setup()
{
  Serial.begin(115200);
  snprintf(log_str, sizeof(log_str), "=> SETUP"); log.log(log.WARNING, __FILE__, log_str);
  bt_serial.enableSSP();
  bt_serial.onConfirmRequest(BTConfirmRequestCallback);
  bt_serial.onAuthComplete(BTAuthCompleteCallback);
  bt_serial.begin("Zunder");
  snprintf(log_str, sizeof(log_str), "<= SETUP"); log.log(log.WARNING, __FILE__, log_str);
}

void loop()
{
  if (bt_pairing_request_pending)
  {
        bt_serial.confirmReply(true);
  }
  else
  {
    
  }
}

