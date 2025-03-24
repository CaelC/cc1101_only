#include "cc1101.h"
#include "spi_communication.h"

#include <string.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"


#define     WRITE_BURST         0x40                        //write burst
#define     READ_SINGLE         0x80                        //read single
#define     READ_BURST          0xC0                        //read burst
#define     BYTES_IN_RXFIFO     0x7F                        //byte number in RXfifo

#define FREQUENCY_433           433.2e6

#define HIGH    1
#define LOW     0

#define CC1101_DEBUG


static const char *TAG = "cc1101";

static uint8_t status[2];

// This is the buffer size of the CC1101 fifo. This library limits the payload to 61 bytes,
// the other 3 bytes are for CRC-OK and LQI-RSSI reports
const uint8_t BUFFER_SIZE = 64;


bool CC1101_begin(CC1101_t* self, const uint32_t freq, bool only_pins) {
    gpio_set_direction(self->CSNpin, GPIO_MODE_OUTPUT);
    gpio_set_direction(self->MISOpin, GPIO_MODE_INPUT);

    hV_HAL_SPI_begin(SPI_CLOCK_1M);

    if(only_pins) 
        return true;

    CC1101_reset(self);
    // Check the version of the Chip as reported by the chip itself
    // Should be 20 and this guves us a way to check if the CC1101 is 
    // indeeed wireed corretly
    uint8_t version = CC1101_readStatusRegister(self, CC1101_VERSION);
    // CC1101 is not present or the wiring/pins is wrong
    if (version<20) return false;

    CC1101_setIDLEstate(self);

    // Configure GDO0 to assert only when valid packet with CRC OK is received
    CC1101_writeRegister(self, CC1101_IOCFG0, 0x07);

    // FIFO threshold settings
    CC1101_writeRegister(self, CC1101_FIFOTHR, 0x07);

    CC1101_writeRegister(self, CC1101_PKTCTRL1, 0x20);
    CC1101_writeRegister(self, CC1101_PKTLEN, MAX_PACKET_LEN);

    // Configure data rate (slower data rates work better with WOR) - 0.5kbps
    CC1101_writeRegister(self, CC1101_MDMCFG4, 0xF7);
    CC1101_writeRegister(self, CC1101_MDMCFG3, 0x83);
    CC1101_writeRegister(self, CC1101_MDMCFG2, 0x03); // 2-FSK, 30/32 sync word bits
    CC1101_writeRegister(self, CC1101_MDMCFG1, 0x72); // NUM_PREAMBLE=7 (192 bytes)

    // State machine configuration
    CC1101_writeRegister(self, CC1101_MCSM0, 0x18);
    CC1101_writeRegister(self, CC1101_MCSM1, 0x30); // CCA enabled TX->IDLE RX->IDLE

    CC1101_writeRegister(self, CC1101_WOREVT1, 0x87);
    CC1101_writeRegister(self, CC1101_WOREVT0, 0x6B);
    CC1101_writeRegister(self, CC1101_WORCTRL, 0x78); // WOR_RES=0

    // Frequency offset compensation
    CC1101_writeRegister(self, CC1101_FOCCFG, 0x16);

    // Receiver settings for better sensitivity
    CC1101_writeRegister(self, CC1101_AGCCTRL2, 0x07);    // Max gain, higher sensitivity (instead of 0x43)
    CC1101_writeRegister(self, CC1101_AGCCTRL0, 0x91);    // AGC filter settings for better sensitivity

    // Frequency synthesizer calibration
    CC1101_writeRegister(self, CC1101_FSCAL3, 0xE9);
    CC1101_writeRegister(self, CC1101_FSCAL2, 0x2A);
    CC1101_writeRegister(self, CC1101_FSCAL1, 0x00);
    CC1101_writeRegister(self, CC1101_FSCAL0, 0x1F);
    
    // Test settings (required for proper operation)
    CC1101_writeRegister(self, CC1101_TEST2, 0x81);
    CC1101_writeRegister(self, CC1101_TEST1, 0x35);
    CC1101_writeRegister(self, CC1101_TEST0, 0x09);

    // Increase output power for transmitter (within ISM regulations)
    // For EU 433MHz ISM: 10mW (10dBm) max
    // For US 915MHz ISM: 25mW (14dBm) max
    CC1101_writeRegister(self, CC1101_FREND0, 0x17);      // Higher power setting
    CC1101_writeRegister(self, CC1101_PATABLE, 0xC0);     // ~10dBm output (for 433MHz)

    // Disable address check
    CC1101_disableAddressCheck(self);

    return true;
}

void CC1101_reset(CC1101_t* self) {
    CC1101_chipDeselect(self);
    esp_rom_delay_us(50);
    CC1101_chipSelect(self);
    esp_rom_delay_us(50);
    CC1101_chipDeselect(self);
    esp_rom_delay_us(50);
    CC1101_chipSelect(self);
    CC1101_waitMiso(self);
    hV_HAL_SPI_transfer(CC1101_SRES);
    CC1101_waitMiso(self);
    CC1101_chipDeselect(self);
}

// Drives CSN HIGH and CC1101 ignores the SPI bus
// TODO not quite drives MISO
void CC1101_chipDeselect(CC1101_t* self) {
    gpio_set_level(self->CSNpin, HIGH);
}

// Drives CSN to LOW and according to the SPI standard,
// CC1101 starts listening to SPI bus
void CC1101_chipSelect(CC1101_t* self) {
    gpio_set_level(self->CSNpin, LOW);
}

// The pin is the actual MISO pin EXCEPT when the MCU cannot digitalRead(MISO)
// if SPI is active (esp8266). In this case we connect another pin with MISO
// and we digitalRead this instead
void CC1101_waitMiso(CC1101_t* self) {
    while (gpio_get_level(self->MISOpin) > 0);
}

// readStatus : read status register
uint8_t CC1101_readStatusRegister(CC1101_t* self, uint8_t addr) {
    uint8_t value,temp;
    temp = addr | READ_BURST;
    CC1101_chipSelect(self);
    CC1101_waitMiso(self);
    hV_HAL_SPI_transfer(temp);
    value = hV_HAL_SPI_transfer(0);
    CC1101_chipDeselect(self);
    return value;
}

// writes the register settings which are common to all
// modes this library supports. For the other registers there are
// specific commands
void CC1101_setCommonRegisters(CC1101_t* self)
{
    CC1101_setIDLEstate(self);
    //writeRegister(CC1101_IOCFG0, 0x01); // Rx report only. This is different than openelec and panstamp lib
    CC1101_writeRegister(self, CC1101_IOCFG0, 0x06); // Asserts when SyncWord is sent/received
    //
    CC1101_writeRegister(self, CC1101_FIFOTHR, 0x4F); // The "F" 0b1111 ensures that GDO0 assrets only if a full packet is received
    //
    CC1101_writeRegister(self, CC1101_MDMCFG3, 0x83);
    CC1101_writeRegister(self, CC1101_MCSM0, 0x18);
    CC1101_writeRegister(self, CC1101_FOCCFG, 0x16);
    CC1101_writeRegister(self, CC1101_AGCCTRL2, 0x43);
    CC1101_writeRegister(self, CC1101_WORCTRL, 0xFB);
    CC1101_writeRegister(self, CC1101_FSCAL3, 0xE9);
    CC1101_writeRegister(self, CC1101_FSCAL2, 0x2A);
    CC1101_writeRegister(self, CC1101_FSCAL1, 0x00);
    CC1101_writeRegister(self, CC1101_FSCAL0, 0x1F);
    CC1101_writeRegister(self, CC1101_TEST2, 0x81);
    CC1101_writeRegister(self, CC1101_TEST1, 0x35);
    CC1101_writeRegister(self, CC1101_TEST0, 0x09);
    //
    // max pkt size = 61. Dealing with larger packets is hard
    // and given the higher possibility of crc errors
    // probably not worth the effort. Generally the packets should be as
    // short as possible
    CC1101_writeRegister(self, CC1101_PKTLEN, MAX_PACKET_LEN); // 0x3D
    CC1101_writeRegister(self, CC1101_MCSM1,0x30); // CCA enabled TX->IDLE RX->IDLE
}

void CC1101_setIDLEstate(CC1101_t* self) {
    uint16_t timeout = 0;
    CC1101_strobe(self, CC1101_SIDLE);
    while (CC1101_getState(self)!=0 && timeout < 10000){
        timeout++;
    } // wait until state is IDLE(=0)

    if(timeout == 10000) {
        #ifdef CC1101_DEBUG
            ESP_LOGE(TAG, "CC1101_setIDLEstate: Timeout");
        #endif
    }
}

uint8_t CC1101_strobe(CC1101_t* self, uint8_t strobe) {
    CC1101_chipSelect(self);
    CC1101_waitMiso(self);
    // FIXME: check if any value is received - now it should be
    uint8_t reply = hV_HAL_SPI_transfer_strobe(strobe);
    CC1101_chipDeselect(self);
    return reply;
}

// return the state of the chip SWRS061I page 31
uint8_t CC1101_getState(CC1101_t* self) {
    uint8_t old_state = CC1101_strobe(self, CC1101_SNOP);
    uint8_t retry_count = 0;
    const uint8_t MAX_RETRIES = 5;
    
    while(retry_count < MAX_RETRIES) {
        uint8_t state = CC1101_strobe(self, CC1101_SNOP);
        
        if (state == old_state) {
            uint8_t extracted_state = (state >> 4) & 0x07;
            return extracted_state;
        }
        
        old_state = state;
        retry_count++;
    }
    
    // If we reach here, we couldn't get stable readings
    // Return the last state we read anyway
    return (old_state >> 4) & 0x07;
}

// writes a byte to a register address
void CC1101_writeRegister(CC1101_t *self, uint8_t addr, uint8_t value) {
    CC1101_chipSelect(self);
    CC1101_waitMiso(self);
    hV_HAL_SPI_transfer(addr);
    hV_HAL_SPI_transfer(value);
    CC1101_chipDeselect(self);
}

void CC1101_enableWhitening(CC1101_t *self) {
    CC1101_setIDLEstate(self);
    CC1101_writeRegister(self, CC1101_PKTCTRL0, 0x45); // WHITE_DATA=1 PKT_FORMAT=0(normal) CRC_EN=1 LENGTH_CONFIG=1(var len)
}

// calculate the value that is written to the register for settings the base frequency
// that the CC1101 should use for sending/receiving over the air.
void CC1101_setFrequency(CC1101_t* self, const uint32_t freq) {
    // We use uint64_t as the <<16 overflows uint32_t
    // however the division with 26000000 allows the final
    // result to be uint32 again
    uint32_t reg_freq = ((uint64_t)freq<<16) / CC1101_CRYSTAL_FREQUENCY;
    //
    // this is split into 3 bytes that are written to 3 different registers on the CC1101
    uint8_t FREQ2 = (reg_freq>>16) & 0xFF;   // high byte, bits 7..6 are always 0 for this register
    uint8_t FREQ1 = (reg_freq>>8) & 0xFF;    // middle byte
    uint8_t FREQ0 = reg_freq & 0xFF;         // low byte
    CC1101_setIDLEstate(self);
    CC1101_writeRegister(self, CC1101_CHANNR, 0);
    CC1101_writeRegister(self, CC1101_FREQ2, FREQ2);
    CC1101_writeRegister(self, CC1101_FREQ1, FREQ1);
    CC1101_writeRegister(self, CC1101_FREQ0, FREQ0);
    #ifdef CC1101_DEBUG
        ESP_LOGI(TAG, "FREQ2= %x", FREQ2);
        ESP_LOGI(TAG, "FREQ1= %x", FREQ1);
        ESP_LOGI(TAG, "FREQ0= %x", FREQ0);
        uint32_t realfreq=((uint32_t)FREQ2<<16)+((uint32_t)FREQ1<<8)+(uint32_t)FREQ0;
        realfreq=((uint64_t)realfreq*CC1101_CRYSTAL_FREQUENCY)>>16;
        ESP_LOGI(TAG, "Real frequency = %ld", realfreq);
    #endif
}

void CC1101_setBaudrate4800bps(CC1101_t* self) {
    CC1101_setIDLEstate(self);
    CC1101_writeRegister(self, CC1101_MDMCFG4, 0xC7);
    CC1101_writeRegister(self, CC1101_DEVIATN, 0x40);
}

// settings from RF studio. This is the default
void CC1101_optimizeSensitivity(CC1101_t* self) {
    CC1101_setIDLEstate(self);
    CC1101_writeRegister(self, CC1101_FSCTRL1, 0x06);
    CC1101_writeRegister(self, CC1101_MDMCFG2, 0x17); // 0b0-001-0-111 OptSensit-GFSK-MATCHESTER-32bitSyncWord+CarrSense
    CC1101_setRXstate(self);
}

// Sends the SRX strobe (if needed) and waits until the state actually goes RX
// flushes FIFOs if needed
void CC1101_setRXstate(CC1101_t* self) {
    const int MAX_ATTEMPTS = 10;
    int attempt = 0;
    
    while(attempt < MAX_ATTEMPTS) {
        uint8_t state = CC1101_getState(self);
        if (state == 0b001) {
            // Successfully in RX state
            return;
        }
        else if (state == 0b110) { // RXFIFO_OVERFLOW
            CC1101_strobe(self, CC1101_SFRX);
            #ifdef CC1101_DEBUG
                ESP_LOGI(TAG, "Cleared RX FIFO overflow");
            #endif
        }
        else if (state == 0b111) { // TXFIFO_UNDERFLOW
            CC1101_strobe(self, CC1101_SFTX);
            #ifdef CC1101_DEBUG
                ESP_LOGI(TAG, "Cleared TX FIFO underflow");
            #endif
        }
        
        // Send strobe to enter RX state
        CC1101_strobe(self, CC1101_SRX);
        
        // Give the chip a moment to change state
        esp_rom_delay_us(100);
        
        attempt++;
    }
    
    #ifdef CC1101_DEBUG
        ESP_LOGW(TAG, "Failed to enter RX state after %d attempts", MAX_ATTEMPTS);
    #endif
}

// 10mW
void CC1101_setPower10dbm(CC1101_t* self) {
    //setIDLEstate();
    CC1101_writeRegister(self, CC1101_PATABLE, 0xC5);
}

void CC1101_disableAddressCheck(CC1101_t* self) {
    CC1101_setIDLEstate(self);
    // two status bytes will be appended to the payload + no address check
    CC1101_writeRegister(self, CC1101_PKTCTRL1,CC1101_PKTCTRL1_DEFAULT_VAL+0);
}

// getPacket read sdata received from RXfifo. Assumes (1 byte PacketLength) + (payload) + (2bytes CRCok, RSSI, LQI)
// requires a buffer with 64 bytes to store the data (max payload = 61)
uint8_t CC1101_getPacket(CC1101_t* self, uint8_t *rxBuffer) {
    uint8_t state = CC1101_getState(self);
    if (state==1) { // RX
        return 0;
    }
    uint8_t rxbytes = CC1101_readStatusRegister(self, CC1101_RXBYTES);
    rxbytes = rxbytes & BYTES_IN_RXFIFO;
    uint8_t size=0;
    if(rxbytes) {
        size=CC1101_readRegister(self, CC1101_RXFIFO);
        if (size>0 && size<=MAX_PACKET_LEN) {
            if ( (size+3)<=rxbytes ) { // TODO
                CC1101_readBurstRegister(self, CC1101_RXFIFO, rxBuffer, size);
                CC1101_readBurstRegister(self, CC1101_RXFIFO, status, 2);
                uint8_t rem=rxbytes-(size+3);
                if (rem>0) {
                    #ifdef CC1101_DEBUG
                        ESP_LOGI(TAG, "FIFO STILL HAS BYTES : %u", rem);
                    #endif
                }
            } else {
                #ifdef CC1101_DEBUG
                    ESP_LOGI(TAG, "size+3<=rxbytes");
                #endif
                size=0;
            }
        } else { 
            #ifdef CC1101_DEBUG
                ESP_LOGI(TAG, "Wrong rx size= %u", size);
            #endif
            size=0;
        }
    }
    CC1101_setIDLEstate(self);
    CC1101_strobe(self, CC1101_SFRX);
    CC1101_setRXstate(self);
    if (size==0) memset(status,0,2); // sets the crc to be wrong and clears old LQI RSSI values
    return size;
}

// readRegister reads data from register address
uint8_t CC1101_readRegister(CC1101_t* self, uint8_t addr) {
    uint8_t temp, value;
    temp = addr|READ_SINGLE; // bit 7 is set for signe register read
    CC1101_chipSelect(self);
    CC1101_waitMiso(self);
    hV_HAL_SPI_transfer(temp);
    value=hV_HAL_SPI_transfer(0);
    CC1101_chipDeselect(self);
    return value;
}

// readBurstRegister reads burst data from register address
// and stores the data to buffer
void CC1101_readBurstRegister(CC1101_t* self, uint8_t addr, uint8_t *buffer, uint8_t num) {
    uint8_t i,temp;
    temp = addr | READ_BURST;
    CC1101_chipSelect(self);
    CC1101_waitMiso(self);
    hV_HAL_SPI_transfer(temp);
    for(i=0;i<num;i++) {
        buffer[i]=hV_HAL_SPI_transfer(0);
    }
    CC1101_chipDeselect(self);
}

// reports if the last packet has correct CRC
bool CC1101_crcok(CC1101_t* self) {
    return status[1]>>7;
}

// reports the signal strength of the last received packet in dBm
// it is always a negative number and can be -30 to -100 dbm sometimes even less.
int16_t CC1101_getRSSIdbm(CC1101_t* self) {
    // from TI app note
    uint8_t rssi_dec = status[0];
    int16_t rssi_dBm;
    // uint8_t rssi_offset = 74;
    const int16_t rssi_offset = 74;
    if (rssi_dec >= 128) {
        rssi_dBm = (int16_t)((int16_t)(rssi_dec - 256) / 2) - rssi_offset;
    } else {
        rssi_dBm = (rssi_dec / 2) - rssi_offset;
    }
    return rssi_dBm;
}

// reports how easily the last packet is demodulated (is read)
uint8_t CC1101_getLQI(CC1101_t* self) {
    return status[1]&0b01111111;;
    // return 0x3F - status[1]&0b01111111;;
}

bool CC1101_sendPacket(CC1101_t* self, const uint8_t *txBuffer, uint8_t size) {
    // vTaskDelay(pdMS_TO_TICKS(1000));
    
    // CC1101_strobe(self, CC1101_SIDLE);

    if (txBuffer==NULL || size==0) {
        #ifdef CC1101_DEBUG
            ESP_LOGI(TAG, "sendPacket called with wrong arguments");
        #endif
        return false;
    }
    if (size>MAX_PACKET_LEN) {
        #ifdef CC1101_DEBUG
            ESP_LOGI(TAG, "Warning, packet truncated");
        #endif
        size=MAX_PACKET_LEN;
    }
    uint8_t txbytes = CC1101_readStatusRegister(self, CC1101_TXBYTES); // contains Bit:8 FIFO_UNDERFLOW + other bytes FIFO bytes
    if (txbytes!=0 || CC1101_getState(self)!=1 ) {
        if (txbytes){
            #ifdef CC1101_DEBUG
                ESP_LOGI(TAG, "BYTES IN TX");
            #endif
        }
        else{
            #ifdef CC1101_DEBUG
                ESP_LOGI(TAG, "getState()!=RX %x", CC1101_getState(self));
            #endif
        }

        CC1101_setIDLEstate(self);
        CC1101_strobe(self, CC1101_SFTX);
        CC1101_strobe(self, CC1101_SFRX);
        CC1101_setRXstate(self);
    }
    esp_rom_delay_us(500); // it helps ?
    CC1101_strobe(self, CC1101_STX);
    uint8_t state = CC1101_getState(self);
    // CC1101_RF lib has register IOCFG0==0x01 which is good for RX
    // but does not give TX info. So we poll the state of the chip (state byte)
    // until state=IDLE_STATE=0
    // note that due to library setting the chip return to IDLE after TX
    if (state==1) {
        // high RSSI
        // No IDLE strobe here, we have potentially an incoming packet.
        #ifdef CC1101_DEBUG
            ESP_LOGI(TAG, "send=false");
        #endif
        return false;
    } else  {
        CC1101_writeRegister(self, CC1101_TXFIFO, size); // write the size of the packet
        CC1101_writeBurstRegister(self, CC1101_TXFIFO, txBuffer, size); // write the packet data to txbuffer
        esp_rom_delay_us(500); // it helps ?
        
        uint16_t timeout = 0;
        while(timeout < 10000) {
            state = CC1101_getState(self);
            #ifdef CC1101_DEBUG
                ESP_LOGI(TAG, "CC1101_getState");
            #endif
            if (state==0) break; // we wait for IDLE state
            
            timeout++;
        }
        if(timeout == 10000) {
            #ifdef CC1101_DEBUG
                ESP_LOGI(TAG, "send=false");
            #endif
            return false;
        }
    }
    CC1101_setIDLEstate(self);
    CC1101_strobe(self, CC1101_SFTX);
    CC1101_setRXstate(self);
    #ifdef CC1101_DEBUG
        ESP_LOGI(TAG, "true");
    #endif
    return true;
}

void CC1101_wor2rx(CC1101_t* self) {
    gpio_set_direction(self->CSNpin, GPIO_MODE_OUTPUT);
    gpio_set_direction(self->MISOpin, GPIO_MODE_INPUT);

    hV_HAL_SPI_begin(SPI_CLOCK_1M);
    
    // Exit WOR mode and enter IDLE state first
    CC1101_strobe(self, CC1101_SIDLE);
    
    // Configure for normal RX operation (non-WOR)
    CC1101_writeRegister(self, CC1101_MCSM2, 0x07);  // Default RX timeout behavior
    CC1101_writeRegister(self, CC1101_MCSM0, 0x18);  // Normal calibration settings
    
    // Enter RX mode
    CC1101_strobe(self, CC1101_SRX);
}

void CC1101_wor(CC1101_t* self, uint16_t timeout) {
    // Force to IDLE state first
    CC1101_strobe(self, CC1101_SIDLE);
    
    // Reset the radio state
    CC1101_strobe(self, CC1101_SFRX);
    CC1101_strobe(self, CC1101_SFTX);
    
    if (timeout < 15) timeout = 15;
    const uint16_t maxtimeout = 750ul * 0xffff / (CC1101_CRYSTAL_FREQUENCY/1000);
    if (timeout > maxtimeout) timeout = maxtimeout;
    
    // Calculate WOR event timing
    uint16_t evt01 = timeout * (CC1101_CRYSTAL_FREQUENCY/1000) / 750;
    
    // Set GDO0 to assert only on valid packet with CRC OK
    CC1101_writeRegister(self, CC1101_IOCFG0, 0x07);
    
    // Important: Set the RC oscillator calibration
    CC1101_writeRegister(self, CC1101_WORCTRL, 0x78); 
    
    // Ensure EVENT0 is set correctly
    CC1101_writeRegister(self, CC1101_WOREVT0, evt01 & 0xff);
    CC1101_writeRegister(self, CC1101_WOREVT1, evt01 >> 8);
    
    // Set RX time control (disable RX_TIME_QUAL for testing)
    CC1101_writeRegister(self, CC1101_MCSM2, 0x00); // Try disabling early RX termination
    
    // Auto-calibration settings
    CC1101_writeRegister(self, CC1101_MCSM0, 0x18); // Use standard calibration
    
    // Enter WOR mode
    CC1101_strobe(self, CC1101_SWOR);

    vTaskDelay(pdMS_TO_TICKS(20));
}

void CC1101_enableAddressCheck(CC1101_t* self, uint8_t addr) {
    CC1101_setIDLEstate(self);
    CC1101_writeRegister(self, CC1101_ADDR, addr);
    // two status bytes will be appended to the payload + address check
    CC1101_writeRegister(self, CC1101_PKTCTRL1, CC1101_PKTCTRL1_DEFAULT_VAL+1);
}

// writes a buffer to a register address
void CC1101_writeBurstRegister(CC1101_t* self, uint8_t addr, const uint8_t *buffer, uint8_t num) {
    uint8_t i, temp;
    temp = addr | WRITE_BURST;
    CC1101_chipSelect(self);
    CC1101_waitMiso(self);
    hV_HAL_SPI_transfer(temp);
    for (i = 0; i < num; i++) {
        hV_HAL_SPI_transfer(buffer[i]);
    }
    CC1101_chipDeselect(self);
}
