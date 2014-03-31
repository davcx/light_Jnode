// Control some LED strips, using settings received by wireless
// or local button
// Evoluzione di luceChecco e LuceMarta
// $Id: light_Jnode.ino 1 2014-3-26 21:21:22Z davide $
// V2.0

// Source originario per pulsanti: arblink.pde
// Source originario per gestione RGB ( gestione PWM software):
// see http://jeelabs.org/2010/06/15/remote-rgb-strip-control/
// and http://jeelabs.org/2010/10/03/software-pwm-at-1-khz/

/* ===========================
(A0); // , uses AIO port 1 PC0
(A1); // , uses AIO port 2 PC1
(A2); // , uses AIO port 3 PC2
(A3); // , uses AIO port 4 PC3

(3);  // , uses IRQ port [1-4]
(4);  // , uses DIO port 1 PD4
(5);  // , uses DIO port 2 PD5
(6);  // , uses DIO port 3 PD6
(7);  // , uses DIO port 4 PD7
In aggiunta:
PB0 = 8
PB1 = 9
I rimanenti di PORTB Atmel sono utilizzati da RF12
============================*/


/* ==================================
	I pin PWM utilizzabili sono:
numportaATM		PD3		PD5		PD6		PB1		PB2		PB3
pin Arduino		p03		p05		p06		p09		p10		p11
pin JeeNode		IRQ		Port2D		Port3D		ISP8	_NA		_NA
=====================================
funzione		__R		__G		__B		__W
*/
// assegnati

#include <JeeLib.h>
#include <avr/eeprom.h>
#include <OneWire.h>

#define DEBUG 1

#define VERSION "\n[light_Jnode.V3.1]"           // keep in sync with the above


// Parametri radio RF12
// NODEID 1 Altana
// NODEID 2 Marta
// NODEID 3 Checco
// NODEID 4 Tester
// NODEID 15 Ethernode
// frequenza group e nodeID da caricare via RF12Demo in EEPROM

#define OUTDest 15      // nodo del bridge rf12<->MQTT

#define LED_PIN     8    // activity LED(B0), comment out to disable

// NVALUES definisce il numero di valori scambiati su rlink
#define NVALUES  5        // num campi (to be adjusted from 0..8) (must also adjust "masks")
#define S_DATA   0        // indirizzo del nodo mittente (Sender)
#define L_DATA   1        // offset in Settings level light
#define C_DATA   2        // offset command
#define TMDATA   3        // MSB temperatura trasmessa
#define TLDATA   4        // LSB temperatura trasmessa

#define LOUTR 3		// PD3 IRQ
#define LOUTG 5		// PD5 P2D
#define LOUTB 6		// PD6 P3D
#define LOUTW 9		// PB1 PB1

// assegnazione pin per pulsanti
// jeenode: usare port 1 (4-A0) (PD4 / PC0) perche non usa PWM
#define loButton A0 //lowerbutton
#define upButton 4  //upperbutton
#define STEP 1
#define MAXSTEPS 6

#define THERMOM A2 
//init the one wire interface on AIO Port4
OneWire  ow(THERMOM);

// RF12 configuration area
typedef struct {
    byte nodeId;            // used by rf12_config, offset 0
    byte band;              // estrae band   dal campo RF12Config.nodeId
    byte group;             // used by rf12_config, offset 1
    byte lnodeId;           // estrae nodeID dal campo RF12Config.nodeId
    byte dnodeId;           // estrae nodeID dal campo RF12Config.nodeId
} RF12Config;

static RF12Config config;

static word ledState;

static byte settings[NVALUES];	// 0=nIDSender, 1=vIntensita luce, 2=comando ,3=MSB temp, 4=LSB temp
static byte intensita[NVALUES];	// 0 = red, 1 = green, 2 = blue, 3 = white
static byte lookUpTbl[4][7] = {
                      {255,255,250,240,200,100,0},
                      {255,255,250,240,200,100,0},
                      {255,255,250,240,200,100,0},
                      {255,254,250,240,200,100,0}
};
MilliTimer timer,timerLhouse;

static const byte masks[] = { LOUTR, LOUTG, LOUTB, LOUTW };
int pending = false;


// led di controllo rf activity
static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
    delay(50);
#endif
}

// configura GPIO
static void setupIO() {
    int pin=0;
    
    //upperButton input mode, with pull-up resistor
    pin=upButton;
    pinMode(pin, INPUT); digitalWrite(pin, 1);
    //lowerButton input mode, with pull-up resistor
    pin=loButton;
    pinMode(pin, INPUT); digitalWrite(pin, 1);
    

//set pin for output
    for (byte i = 0; i < NVALUES; ++i) {
        pinMode(masks[i], OUTPUT);
        digitalWrite(masks[i], 0);
    }
}

// lettura stato pulsante superiore !!!!!attenzione alla logica negata
static byte upperButton () {
	return !digitalRead(upButton);
}

// lettura stato pulsante inferiore !!!!!attenzione alla logica negata
static byte lowerButton () {
	return !digitalRead(loButton);
}



// load da EEPROM configurazione RF12
static void loadConfig() {
    
        //((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + 0);
        config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR + 0);
        //((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
        config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
        config.band = config.nodeId & 0xE0;
        config.lnodeId = config.nodeId & 0x1F;
        config.dnodeId = OUTDest;
}

static void showConfig() {
    Serial.println(VERSION);
    Serial.print("Radio Init->");Serial.print(" ");Serial.print( config.nodeId,HEX);
    Serial.print("NODEID ");Serial.print( config.nodeId,HEX);Serial.print(" | ");
    Serial.print("BAND ");Serial.print( config.nodeId & 0xE0,HEX);Serial.print(" | ");
    Serial.print("GROUP ");Serial.print( config.group);Serial.print(" | ");
    Serial.println();
    Serial.println();
}

// lettura da EEPROM a vettore settings[]
static void loadSettings() {
    for (byte i = 0; i < NVALUES; ++i){
        settings[i] = eeprom_read_byte((byte*) i);
    }
    prepareSlots();
}

// calcolo vettore stato PWM
static void prepareSlots() {    
    for (byte i=0; i < NVALUES; i++){
        intensita[i] = (lookUpTbl[i][settings[L_DATA]]  ) ;
        analogWrite(masks[i],intensita[i]);
    }

#if DEBUG
    Serial.print("Settings[]  values:");
        for (byte i = 0; i < NVALUES; ++i) {
	    Serial.print(" | ");
	    Serial.print(i);
	    Serial.print(" - ");
	    Serial.print(settings[i], DEC);
	}
	Serial.println();

	Serial.print("Intensita[] values:");
	for (byte i = 0; i < NVALUES; ++i) {
	    Serial.print(" | ");
	    Serial.print(i);
	    Serial.print(" - ");
	    Serial.print(intensita[i], DEC);
	}
	Serial.println();
	Serial.println();
#endif
}

// scrittura da vettore a EEPROM
static void saveSettings() {
    for (byte i = 0; i < NVALUES; ++i){
	eeprom_write_byte((byte*) i, settings[i]);
    }
    prepareSlots();
}



float getTemp12(OneWire  ds,byte datatemp[] ) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return 0;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return 0;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return 0 ;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(500);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  float TemperatureSum = (float)raw / 16.0;
  Serial.println(TemperatureSum);
  datatemp[TMDATA] = (int)TemperatureSum;
  datatemp[TLDATA] = (TemperatureSum - datatemp[TMDATA])*100;

  return TemperatureSum;
}


//********** Setup ***********************
void setup () {
    Serial.begin(57600);
    if (rf12_configSilent()) {
        loadConfig();            // load da EEPROM configurazione RF12
        showConfig();
        settings[S_DATA]=config.lnodeId;
    } else {
        Serial.println("EEPROM RF12 data errors");
    }
    setupIO();      // configurazione GPIO
    loadSettings(); // load da EEPROM parametri runtime
    
    //rf12_initialize(config.nodeId & 0x1F, config.nodeId >> 6, config.group);
}


//********** Loop ************************
void loop () {
    delay(200);
    
    //trasmissione faro ogni 10 sec
    if (timerLhouse.poll(10000)){
        settings[C_DATA] = 'T';
        float temper = getTemp12(ow,settings );
        Serial.print ("temperatura letta -- ");
        Serial.println (temper);
        pending = true;
    }
            
    // Se push upperbutton
    if (upperButton() & !pending){
        if (settings[L_DATA] >= MAXSTEPS){
            settings[L_DATA]= MAXSTEPS;
        }else{
            settings[L_DATA] += STEP;
        }
        Serial.println("UP ->");
        settings[C_DATA] = 'L';
        saveSettings();
        pending = true;
    }
    

    // Se push lowerbutton
    if (lowerButton() & !pending){ 
        if (settings[L_DATA] >=STEP){
            settings[L_DATA] -= STEP;
        }else{
            settings[L_DATA] = 0;
        }
        Serial.println("DWN ->");
        settings[C_DATA] = 'L';
        saveSettings();	
    	pending = true;
    }


// se ricevuto un frame remoto aggiorna il vettore 
// eventualmente invia ack
    if ( rf12_recvDone()){
        if( rf12_hdr == (RF12_HDR_DST | config.nodeId)
		&& rf12_crc == 0
		&& rf12_len == (NVALUES)
		&& rf12_data[C_DATA] == 'L') {
//	    if (RF12_WANTS_ACK && rf12_canSend() ) {
//	        Serial.println(" -> ack");
//	        rf12_sendStart(RF12_ACK_REPLY, 0, 0);
//	    }
            activityLed (1);
            Serial.print(" RX frame:     ");
            for (byte i=0; i < NVALUES; i++){
                Serial.print(" | ");
                Serial.print(i);
                Serial.print(" - ");
                Serial.println(intensita[i], DEC);
            }
            //pending = true;
            memcpy(settings, (void*) rf12_data, rf12_len);
            saveSettings();
            delay(100);
            activityLed (0);
        }
    }
    
    // Se aggiornato il vettore PWM fai writeupdate    
    if (pending){
        for (byte i=0; i < NVALUES; i++){
            Serial.print(" - ");
            Serial.print(settings[i], DEC);
        }
        Serial.print("   ....tx. , ");
        Serial.println(OUTDest);
        
        Serial.println();
        //invia il frame radio per update
        if ( rf12_canSend()){
            Serial.println("   TX >>>>");
            activityLed (1);
            rf12_sendStart(RF12_HDR_CTL | RF12_HDR_DST | OUTDest, settings, sizeof(settings));
            delay(100);
            activityLed (0);
            pending = false;
        }
    }
}

