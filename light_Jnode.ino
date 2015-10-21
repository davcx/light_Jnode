// Control some LED strips, using settings received by wireless
// or local button
// Evoluzione di luceChecco e LuceMarta
// $Id$
// V3.3

// Source originario per pulsanti: arblink.pde
// Source originario per gestione RGB ( gestione PWM software):
// see http://jeelabs.org/2010/06/15/remote-rgb-strip-control/
// and http://jeelabs.org/2010/10/03/software-pwm-at-1-khz/
// RF12_HDR_CTL | RF12_HDR_DST 
// Link protocollo:
// http://jeelabs.net/projects/jeelib/wiki/RF12demo


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

#define VERSION "\n[light_Jnode.V3.3]"           // keep in sync with the above


// Parametri radio RF12
// NODEID 1 Altana
// NODEID 2 Marta
// NODEID 3 Checco
// NODEID 4 Tester
// NODEID 31 GW
// frequenza group e nodeID da caricare via RF12Demo in EEPROM

#define OUTDest 31      // nodo del bridge rf12<->MQTT

#define LED_PIN     8    // activity LED(B0), comment out to disable

// NVALUES definisce il numero di valori scambiati su rlink
#define NVALUES  5        // num campi (to be adjusted from 0..8) (must also adjust "masks")

#define S_DATA   0        // indirizzo del nodo mittente (Sender)
#define L_DATA   1        // offset in Settings level light
#define C_DATA   2        // offset command
#define TMDATA   3        // MSB temperatura trasmessa
#define TLDATA   4        // LSB temperatura trasmessa

#define NLEDS 4         // numero totale di pwm da pilotare
#define LOUTR 3		// PD3 IRQ
#define LOUTG 5		// PD5 P2D
#define LOUTB 6		// PD6 P3D
#define LOUTW 9		// PB1 PB1

// assegnazione pin per pulsanti
// jeenode: usare port 1 (4-A0) (PD4 / PC0) perche non usa PWM
#define loButton A0 //lowerbutton
#define upButton 4  //upperbutton
#define STEP 1
#define MAXSTEPS 7

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
static byte lookUpTbl[NLEDS][MAXSTEPS] = {
                      {255,255,250,240,190,100,0},
                      {255,255,250,230,200,100,0},
                      {255,255,240,240,200,100,0},
                      {255,254,250,240,200,100,0}
};
MilliTimer timer,timerLhouse;

static const byte masks[] = { LOUTR, LOUTG, LOUTB, LOUTW };
int pending = false;



// configura GPIO
 void setupIO() {
    int pin= NULL;
    
    //upperButton input mode, with pull-up resistor
    pin=upButton;
    pinMode(pin, INPUT); digitalWrite(pin, HIGH);
    //lowerButton input mode, with pull-up resistor
    pin=loButton;
    pinMode(pin, INPUT); digitalWrite(pin, HIGH);
    

//set pin for output
    for (byte i = 0; i < NVALUES; ++i) {
        pinMode(masks[i], OUTPUT);
        digitalWrite(masks[i], 0);
    }
}

// lettura stato pulsante superiore !!!!!attenzione alla logica negata
 byte upperButton () {
	return !digitalRead(upButton);
}

// lettura stato pulsante inferiore !!!!!attenzione alla logica negata
 byte lowerButton () {
	return !digitalRead(loButton);
}




// lettura da EEPROM a vettore settings[]
void loadSettings() {
//    for (byte i = 0; i < NVALUES; ++i){
//        settings[i] = eeprom_read_byte((byte*) i);
//    }
//load del solo valore utile (Level_Data)
    settings[L_DATA] = eeprom_read_byte((byte*) L_DATA);
    prepareSlots();
}

// calcolo vettore stato PWM
 void prepareSlots() {    
    for (byte i=0; i < NLEDS; i++){
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
	Serial.println();

	Serial.print("Intensita[] values:");
        Serial.println();
	for (byte i = 0; i < NLEDS; ++i) {
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
 void saveSettings() {
//    for (byte i = 0; i < NVALUES; ++i){
//	eeprom_write_byte((byte*) i, settings[i]);
//    }
	eeprom_write_byte((byte*) L_DATA, settings[L_DATA]);
    prepareSlots();
}





//********** Setup ***********************
void setup () {
    Serial.begin(57600);
    if (rf12_configSilent()) {   // Se RF12Config restituisce il nodeId
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
    
    //trasmissione faro ogni 10 sec
    if (timerLhouse.poll(30000)){
        settings[C_DATA] = 'T';
        float temper = getTemp12(ow,settings );
        Serial.print ("temperatura letta -- ");
        Serial.println (temper);
        pending = true;
    }
            
    // Se push upperbutton
    if (upperButton() & !pending){
        if (settings[L_DATA] < MAXSTEPS-1){
            settings[L_DATA] += STEP;
        }else{
            settings[L_DATA] = (MAXSTEPS - 1);
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
        Serial.print(" RX frame:<<<<  ");
                    Serial.print(" NVALUES:");
                    Serial.println(rf12_len);
                for (byte i=0; i < rf12_len; i++){
                            Serial.print(" | ");
                            Serial.print(i);
                            Serial.print(" - ");
                            Serial.println(rf12_data[i], DEC);
                 }
        if( rf12_hdr == (RF12_HDR_DST | config.nodeId & 0x1F) && rf12_crc == 0){
                if (rf12_len == NVALUES){

                if (   rf12_data[S_DATA] == config.lnodeId
                    && rf12_data[C_DATA] == '1'){
	                    if (RF12_WANTS_ACK && rf12_canSend() ) {
	                         Serial.println(" -> ack");
	                         rf12_sendStart(RF12_ACK_REPLY, 0, 0);
	                    }
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
        }
        }
    
    // Se aggiornato il vettore PWM fai writeupdate    
    if (pending){
        for (byte i=0; i < NVALUES; i++){
            Serial.print(" - ");
            Serial.print(settings[i], DEC);
        }
        Serial.print("   ....tx. , ");
        Serial.println(config.lnodeId);
        
        Serial.println();
        //invia il frame radio per update
        if ( rf12_canSend()){
            Serial.println("   TX >>>>");
            activityLed (1);
            rf12_sendStart( config.lnodeId, settings, sizeof(settings));
            delay(100);
            activityLed (0);
            pending = false;
        }
    }
}

