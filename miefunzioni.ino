///////////////////////////////
//$Id$
///////////////////////////////

///////////////////
//Activity led
//
// led di controllo rf activity
 void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
    delay(50);
#endif
}

//////////////////////
// 
//
// load da EEPROM configurazione RF12
 void loadConfig() {
    
        //((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + 0);
        config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR + 0);
        //((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
        config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
        config.band = (config.nodeId & 0xE0)>>6;
        config.lnodeId = config.nodeId & 0x1F;
        config.dnodeId = OUTDest;
}

//////////////////////
// 
//
// Show configurazione RF12 da EEPROM 
 void showConfig() {
    Serial.println(VERSION);
    Serial.print("Radio Init->");Serial.print(" ");Serial.print( config.nodeId,HEX);Serial.print(" ");
    Serial.print("NODEID ");Serial.print( config.lnodeId,HEX);Serial.print(" | ");
    Serial.print("BAND ");Serial.print( (config.nodeId & 0xE0)>>6,HEX);Serial.print(" | ");
    Serial.print("GROUP ");Serial.print( config.group);Serial.print(" | ");
    Serial.println();
    Serial.println();
}






//////////////////////
// chiamata a lettura temperatura di DS generico
//

float getTemp12 ( OneWire  ds,uint8_t datatemp[] ) {
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

