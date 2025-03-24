#ifndef __CC1101__H__
#define __CC1101__H__


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//***************************************CC1101 define**************************************************//
// CC1101 CONFIG REGISTERS
#define CC1101_IOCFG2       0x00        // GDO2 output pin configuration
#define CC1101_IOCFG1       0x01        // GDO1 output pin configuration
#define CC1101_IOCFG0       0x02        // GDO0 output pin configuration
#define CC1101_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1        0x04        // Sync word, high INT8U
#define CC1101_SYNC0        0x05        // Sync word, low INT8U
#define CC1101_PKTLEN       0x06        // Packet length
#define CC1101_PKTCTRL1     0x07        // Packet automation control
#define CC1101_PKTCTRL0     0x08        // Packet automation control
#define CC1101_ADDR         0x09        // Device address
#define CC1101_CHANNR       0x0A        // Channel number
#define CC1101_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC1101_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC1101_FREQ2        0x0D        // Frequency control word, high INT8U
#define CC1101_FREQ1        0x0E        // Frequency control word, middle INT8U
#define CC1101_FREQ0        0x0F        // Frequency control word, low INT8U
#define CC1101_MDMCFG4      0x10        // Modem configuration
#define CC1101_MDMCFG3      0x11        // Modem configuration
#define CC1101_MDMCFG2      0x12        // Modem configuration
#define CC1101_MDMCFG1      0x13        // Modem configuration
#define CC1101_MDMCFG0      0x14        // Modem configuration
#define CC1101_DEVIATN      0x15        // Modem deviation setting
#define CC1101_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CC1101_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CC1101_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CC1101_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CC1101_BSCFG        0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B        // AGC control
#define CC1101_AGCCTRL1     0x1C        // AGC control
#define CC1101_AGCCTRL0     0x1D        // AGC control
#define CC1101_WOREVT1      0x1E        // High INT8U Event 0 timeout
#define CC1101_WOREVT0      0x1F        // Low INT8U Event 0 timeout
#define CC1101_WORCTRL      0x20        // Wake On Radio control
#define CC1101_FREND1       0x21        // Front end RX configuration
#define CC1101_FREND0       0x22        // Front end TX configuration
#define CC1101_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27        // RC oscillator configuration
#define CC1101_RCCTRL0      0x28        // RC oscillator configuration
#define CC1101_FSTEST       0x29        // Frequency synthesizer calibration control
#define CC1101_PTEST        0x2A        // Production test
#define CC1101_AGCTEST      0x2B        // AGC test
#define CC1101_TEST2        0x2C        // Various test settings
#define CC1101_TEST1        0x2D        // Various test settings
#define CC1101_TEST0        0x2E        // Various test settings

// CC1101 Strobe commands
#define CC1101_SRES         0x30        // Reset chip.
#define CC1101_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CC1101_SXOFF        0x32        // Turn off crystal oscillator.
#define CC1101_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CC1101_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CC1101_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CC1101_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST      0x3C        // Reset real time clock.
#define CC1101_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // INT8Us for simpler software.
// CC1101 STATUS REGISTERS
#define CC1101_PARTNUM      0x30
#define CC1101_VERSION      0x31
#define CC1101_FREQEST      0x32
#define CC1101_LQI          0x33
#define CC1101_RSSI         0x34
#define CC1101_MARCSTATE    0x35
#define CC1101_WORTIME1     0x36
#define CC1101_WORTIME0     0x37
#define CC1101_PKTSTATUS    0x38
#define CC1101_VCO_VC_DAC   0x39
#define CC1101_TXBYTES      0x3A
#define CC1101_RXBYTES      0x3B

//CC1101 PATABLE,TXFIFO,RXFIFO
#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F
#define CC1101_RXFIFO       0x3F

// The library enforces this maximum packet size
// The internal CC1101 buffer is 64bytes but 3 bytes can be used for LQI RSSI an address check
#define MAX_PACKET_LEN 61
// Most modules come with 26Mhz crystal

#define  CC1101_CRYSTAL_FREQUENCY 26000000ul

// 0 (no preamble detection) - 7 max 4*PQT preamble detection
#define CC1101_PKTSTATUS_PQT 7

// TODO explanation
#define CC1101_PKTCTRL1_DEFAULT_VAL (CC1101_PKTSTATUS_PQT*32+4)

// FIXME: Configure this in the header file
#define RXADDR                          0x89ABCDEF   // Address of this device
#define TXADDR                          0x12345678   // Address of device to send to


typedef struct {
    uint8_t CSNpin;
    uint8_t GDO0pin;
    uint8_t MISOpin;
} CC1101_t;

extern bool CC1101_begin(CC1101_t* self, const uint32_t freq, bool only_pins);

extern void CC1101_reset(CC1101_t* self);

extern void CC1101_chipDeselect(CC1101_t* self);
extern void CC1101_chipSelect(CC1101_t* self);

extern void CC1101_waitMiso(CC1101_t* self);

extern uint8_t CC1101_readStatusRegister(CC1101_t* self, uint8_t addr);
extern void CC1101_setCommonRegisters(CC1101_t* self);
extern void CC1101_writeRegister(CC1101_t *self, uint8_t addr, uint8_t value);
extern uint8_t CC1101_readRegister(CC1101_t* self, uint8_t addr);
extern void CC1101_readBurstRegister(CC1101_t* self, uint8_t addr, uint8_t *buffer, uint8_t num);
extern void CC1101_writeBurstRegister(CC1101_t* self, uint8_t addr, const uint8_t *buffer, uint8_t num);

extern void CC1101_setIDLEstate(CC1101_t* self);
extern uint8_t CC1101_strobe(CC1101_t* self, uint8_t strobe);
extern uint8_t CC1101_getState(CC1101_t* self);
extern void CC1101_enableWhitening(CC1101_t *self);
extern void CC1101_setFrequency(CC1101_t* self, const uint32_t freq);
extern void CC1101_setBaudrate4800bps(CC1101_t* self);
extern void CC1101_optimizeSensitivity(CC1101_t* self);
extern void CC1101_setRXstate(CC1101_t* self);
extern void CC1101_setPower10dbm(CC1101_t* self);
extern void CC1101_disableAddressCheck(CC1101_t* self);
extern void CC1101_enableAddressCheck(CC1101_t* self, uint8_t addr);
extern bool CC1101_crcok(CC1101_t* self);
extern int16_t CC1101_getRSSIdbm(CC1101_t* self);
extern uint8_t CC1101_getLQI(CC1101_t* self);

extern bool CC1101_sendPacket(CC1101_t* self, const uint8_t *txBuffer, uint8_t size);
extern uint8_t CC1101_getPacket(CC1101_t* self, uint8_t *rxBuffer);

// Should be used immediatelly after WOR -> GDO0 assert
extern void CC1101_wor2rx(CC1101_t* self);
// Sets the chip to WakeOnRadio state. The chip sleeps for "timeout" milliseconds
// and briefly wakes up to check for incoming message or preamble. If no message is
// present it is going for sleep and the cycle repeats.
extern void CC1101_wor(CC1101_t* self, uint16_t timeout/*=1000*/); //  1000ms=1sec cycle


#endif
