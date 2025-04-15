/** ================================================================
 * @file    application/src/drivers/TESEO/teseo.cpp
 *
 * @brief   Implrement teseo base class, and it's action
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * Libs
 * -----------------------------------------------------------------
 */
// Associated header
#include "drivers/teseo.h"

// Zephyr
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

// External libs
#include "drivers/teseo/teseo_gnss_structs.h"
#include "drivers/teseo/NMEA_parser.h"
#include "drivers/teseo/gnss1a1_gnss.h"
#include "drivers/teseo/teseo_liv3f_conf.h"

// Internal libs
#include "init/init.h"
#include "config.h"
#include "drivers/teseo/teseo_struct.h"

// STD
#include <cstdint>
#include <stdlib.h> // C++ lib not present ??

/* -----------------------------------------------------------------
 * LOGGER MODULE
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Teseo, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * PRIVATE DEFINES
 * -----------------------------------------------------------------
 */
// Let's define standard commands
constexpr char *_NMEA_PPSEN = (char *)"$PSTMCFGPPSGEN,1,0,0,0*_\r\n";
constexpr char *_NMEA_PPSOF = (char *)"$PSTMCFGPPSGEN,0,0,0,0*_\r\n";
constexpr char *_NMEA_ODOEN = (char *)"$PSTMCFGODO,1,0,65535*_\r\n";
constexpr char *_NMEA_ODOOF = (char *)"$PSTMCFGODO,0,0,65535*_\r\n";
constexpr char *_NMEA_CONST = (char *)"$PSTMCFGCONST\0"; // No \r\n --> Formatted by after
constexpr char *_NMEA_EPHER = (char *)"$PSTMCLREPHS*_\r\n";
constexpr char *_NMEA_FREQR = (char *)"$PSTMSETRANGE,-57000,-37000*_\r\n";
constexpr char *_NMEA_LOCFG = (char *)"$PSTMINITFRQ,-47000*_\r\n";
constexpr char *_NMEA_AGPSC = (char *)"$PSTMCFGAGPS,1*_\r\n";
constexpr char *_NMEA_PPSCF = (char *)"$PSTMPPS,1,8*_\r\n"; // FIX COND to get PPS
constexpr int _NMEA_PPSCF_LEN = 17;
constexpr int _NMEA_PPSEN_LEN = 27;
constexpr int _NMEA_PPSOF_LEN = 27;
constexpr int _NMEA_ODOEN_LEN = 26;
constexpr int _NMEA_ODOOF_LEN = 26;
constexpr int _NMEA_CONST_LEN = 14;
constexpr int _NMEA_EPHER_LEN = 17;
constexpr int _NMEA_FREQR_LEN = 32;
constexpr int _NMEA_LOCFG_LEN = 24;
constexpr int _NMEA_AGPSC_LEN = 19;

// Define NMEA command delimiters
constexpr char CMD_Addr = '$';
constexpr char CMD_Chkm = '*';
constexpr char Chksm_placeholder = '_';

/* -----------------------------------------------------------------
 * PRIVATE FUNCTIONS
 * -----------------------------------------------------------------
 */

/*
 * Compute the checksum.
 *
 * This function shall be executed at compile time rather than on the nRF !
 */
const char *_TESEO_CHKSM(const char input[], uint8_t len)
{
    // Check that the input is well formed
    if (char2int(input[0]) != char2int('$'))
    {
        return (char *)' ';
    }

    char *placeholder;
    placeholder = strchr(input, CMD_Chkm);
    if (placeholder == nullptr)
    {
        return (char *)'E';
    }
    if (placeholder < (input + len))
    {
        return (char *)'R';
    }

    char *pos;
    pos = strchr(input, Chksm_placeholder);
    if (pos == nullptr)
    {
        return (char *)'T';
    }

    // Iterate over the string to compute the checksum
    uint8_t checksum = 0;
    for(int k = 0; k < len; k++)
    {
        if (input[k] == '*')
        {
            break;
        }
        checksum ^= input[k];
    }

    // Inserting the computed value into the char
    *placeholder = checksum;
    return input;
}

/* -----------------------------------------------------------------
 * PRIVATE COMMANDS DEFINES
 * -----------------------------------------------------------------
 */
// Theses values are computed by GCC at the compile time
const char *NMEA_PPSEN = _TESEO_CHKSM(_NMEA_PPSEN, _NMEA_PPSEN_LEN);
const char *NMEA_PPSOF = _TESEO_CHKSM(_NMEA_PPSOF, _NMEA_PPSOF_LEN);
const char *NMEA_ODOEN = _TESEO_CHKSM(_NMEA_ODOEN, _NMEA_ODOEN_LEN);
const char *NMEA_ODOOF = _TESEO_CHKSM(_NMEA_ODOOF, _NMEA_ODOOF_LEN);
const char *NMEA_EPHER = _TESEO_CHKSM(_NMEA_EPHER, _NMEA_EPHER_LEN);
const char *NMEA_FREQR = _TESEO_CHKSM(_NMEA_FREQR, _NMEA_FREQR_LEN);
const char *NMEA_LOCFG = _TESEO_CHKSM(_NMEA_LOCFG, _NMEA_LOCFG_LEN);
const char *NMEA_AGPSC = _TESEO_CHKSM(_NMEA_AGPSC, _NMEA_AGPSC_LEN);
const char *NMEA_PPSCF = _TESEO_CHKSM(_NMEA_PPSCF, _NMEA_PPSCF_LEN);

// Commands
const char *NMEA_PSTIM = (char *)"$PSTMTIM\r\n\0";
const char *NMEA_GPGLL = (char *)"$GPGLL\r\n\0";
const char *NMEA_GNGSA = (char *)"$GNGSA\r\n\0";
const char *NMEA_GPVTG = (char *)"$GPVTG\r\n\0";
constexpr int NMEA_PSTIM_LEN = 12;
constexpr int NMEA_GPGLL_LEN = 10;
constexpr int NMEA_GNGSA_LEN = 10;
constexpr int NMEA_GPVTG_LEN = 10;

// Return values
const char *PSTMCFGPPSGENOK = (char *)"$PSTMCFGPPSGENOK";
const char *PSTMCFGODOOK = (char *)"$PSTMCFGODOOK";
const char *PSTMCFGTHGNSSOK = (char *)"$PSTMCFGTHGNSSOK";
const char *PSTMCFGAGPSOK = (char*)"$PSTMCFGAGPSOK";
const char *PSTMSETRANGEOK = (char*)"$PSTMSETRANGEOK";
constexpr int PSTM_CFG_PPS_OK_LEN = 17;
constexpr int PSTM_CFG_ODO_OK_LEN = 14;
constexpr int PSTM_CFG_GNS_OK_LEN = 17;
constexpr int PSTM_CFG_GAG_OK_LEN = 15;
constexpr int PSTM_CFG_RAN_OK_LEN = 16;

/* -----------------------------------------------------------------
 * CONSTRUCTORS AND DESTRUCTORS
 * -----------------------------------------------------------------
 */

TESEO::TESEO()
{
    // Open the UART bus
    this->bus = INIT_GetAnUART(UARTS::GPS);
    if (this->bus == nullptr)
    {
        this->device_openned = false;
        return;
    }
    this->device_openned = true;

    // GNSS1A1 init
    int ret = 0;
    ret = GNSS1A1_GNSS_Init(GNSS1A1_TESEO_LIV3F);
    if (ret != 0)
    {
        LOG_ERR("Failed to initialize GNSS1A1. Aborting...");
        return;
    }

    // Initializing internal variables
    this->gnss_feature = 0x0;
    this->IO_DataReady = false;

    // Initializing parser
    ret = GNSS_PARSER_Init(&this->GNSSParser_Data);
    if (ret != 0)
    {
        LOG_ERR("Failed to initialize GNSS Parser. Aborting...");
        return;
    }

    // Then, register the bus communication ISR (and data)
    ret = uart_callback_set(this->bus,
                            TESEO::ISR_RX,
                            (void *)this);
    if (ret != 0)
    {
        LOG_ERR("Failed to set the RX callback ISR !");
        return;
    }

    // Enabling UART reception
    ret = uart_rx_enable(this->bus,
                         (uint8_t *)this->buf,
                         sizeof(this->buf),
                         2048);
    // 2048 us without no new data.
    // This correspond to 2 char at 9600 baud.
    // This delay is used to differentiate two messages from the others.

    if (ret != 0)
    {
        LOG_ERR("Failed to enable the UART RX !");
        return;
    }

    // Device is initialized, exiting !
    return;
}
TESEO::~TESEO()
{
    // Closing UART HAL !
    INIT_FreeAnUART(UARTS::GPS, this->bus);
}

/* -----------------------------------------------------------------
 * FUNCTIONS
 * -----------------------------------------------------------------
 */

/*
 * For each function, the use principle differ a little bit
 * from the original ST example.
 *
 * In the example, they used a separate thread, that respond
 * to user commands over UART and send them to the GPS.
 * The GPS thread is thus only a listener on this bus, where,
 * in our case we're also writter.
 */
uint8_t TESEO::getUTCTime(GPGGA_Info_t *const date)
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_PSTIM, NMEA_PSTIM_LEN);
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Start the parsing of the message
    int status = GNSS_PARSER_ParseMsg(&this->GNSSParser_Data, eNMEAMsg::GPGGA, (uint8_t *)this->buf);
    if (status == PARSE_FAIL)
    {
        return -3;
    }

    // If sucessfull, we copy the output of into the custom allocated struct
    memcpy((void *)date, (void *)&this->GNSSParser_Data.gpgga_data, sizeof(GPGGA_Info_t));

    // Exiting the function !
    return 0;
}

uint8_t TESEO::enablePPS()
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_PPSEN, _NMEA_PPSEN_LEN);
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGPPSGENOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGPPSGENOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGPPSGENOK) != 0)
    {
        return -3;
    }

    // Exit the function
    return 0;
}

uint8_t TESEO::disablePPS()
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_PPSOF, _NMEA_PPSOF_LEN);
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGPPSGENOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGPPSGENOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGPPSGENOK) != 0)
    {
        return -3;
    }

    // Exit the function
    return 0;
}

uint8_t TESEO::enableOdometer()
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_ODOEN, _NMEA_ODOEN_LEN);
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGODOOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGODOOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGODOOK) != 0)
    {
        return -2;
    }

    // Exit the function
    return 0;
}

uint8_t TESEO::disableOdometer()
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_ODOOF, _NMEA_ODOOF_LEN);
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGODOOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGODOOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGODOOK) != 0)
    {
        return -2;
    }

    // Exit the function
    return 0;
}

uint8_t TESEO::selectConstellations(bool GPS,
                                    bool GLONASS,
                                    bool GALILEO,
                                    bool QZSS,
                                    bool BEIDOU)
{
    // First, format the command and write it to the module
    char wbuf[32] = {};
    strcpy(wbuf, _NMEA_CONST);
    int count = sizeof(_NMEA_CONST);

    for (int k = 0; k < 5; k++)
    {
        wbuf[count] = ',';
        count++;
        
        switch (k)
        {
            case 0:
                wbuf[count] = (int)GPS + 0x30;
                break;
            case 1:
                wbuf[count] = (int)GLONASS + 0x30;
                break;
            case 2:
                wbuf[count] = (int)GALILEO + 0x30;
                break;
            case 3:
                wbuf[count] = (int)QZSS + 0x30;
                break;
            case 4:
                wbuf[count] = (int)BEIDOU + 0x30;
                break;
            }
        count++;
    }

    const char *writeval = _TESEO_CHKSM(wbuf, sizeof(wbuf));

    // Write the value
    int ret = this->write((uint8_t *)writeval, sizeof(writeval));
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGTHGNSSOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGTHGNSSOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGTHGNSSOK) != 0)
    {
        return -2;
    }

    // Exit the function
    return 0;
}

uint8_t TESEO::getPosition(GNS_Info_t *const pos)
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_GPGLL, NMEA_GPGLL_LEN);
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Start the parsing of the message
    int status = GNSS_PARSER_ParseMsg(&this->GNSSParser_Data, eNMEAMsg::GNS, (uint8_t *)this->buf);
    if (status == PARSE_FAIL)
    {
        return -3;
    }
    /*
     * Not sure about the correct parsing / command sent. To be checked !
     */

    // If sucessfull, we copy the output of into the custom allocated struct
    memcpy((void *)pos, (void *)&this->GNSSParser_Data.gns_data, sizeof(GNS_Info_t));

    // Exiting the function !
    return 0;
}

uint8_t TESEO::GetSatelites(struct Satellite *const sat,
                            uint8_t *const len)
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_GNGSA, NMEA_GNGSA_LEN);
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Start the parsing of the message
    int status = GNSS_PARSER_ParseMsg(&this->GNSSParser_Data, eNMEAMsg::GSA, (uint8_t *)this->buf);
    if (status == PARSE_FAIL)
    {
        return -3;
    }

    // If sucessfull, we copy the output of into the custom allocated struct
    memcpy((void *)sat, (void *)&this->GNSSParser_Data.gsa_data, sizeof(GSA_Info_t));
    *len = MAX_SAT_NUM; // Memory is initialized, so we always define to the constant.

    // Exiting the function !
    return 0;
}

uint8_t TESEO::GetGroundSpeed(struct VTG_Info_t *const vtg)
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_GPVTG, NMEA_GPVTG_LEN);
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // There is parser provided by ST, so we wrote one by ourselves
    char * pos;
    char * in;

    // TMGT field
    in = (char *)this->buf + (sizeof(NMEA_GPVTG) - 2) + 1; // Compute the shift required to point on the first char
    vtg->TrackDifReal = strtof( in, &pos);

    // TMGM field
    in = pos;
    vtg->TrackDifMag = strtof( in + 3, &pos);

    // SoGN field
    in = pos;
    vtg->KnotsSpeed = strtof( in + 3, &pos);

    // SoGK
    in = pos;
    vtg->KMSpeed = strtof( in + 3, &pos);

    // Mode field
    strncpy(&vtg->Mode, in + 3, 1);

    // Exiting the function !
    return 0;
}

/* -----------------------------------------------------------------
 * PRIVATES FUNCS
 * -----------------------------------------------------------------
 */

uint8_t TESEO::configureAGPS()
{
        // First, write the command to the TESEO module
        int ret = this->write((uint8_t *)NMEA_AGPSC, _NMEA_AGPSC_LEN);
        if (ret != 0)
        {
            return -1;
        }
    
        // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
        // This variable shall be configured to true by the ISR !
        while (this->IO_DataReady == false)
        {
            k_yield();
        }
        this->IO_DataReady = false;
    
        // Checking the sanity of the message
        // And returning if something when wrong...
        int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
        if (check != GNSS_PARSER_ERROR)
        {
            return -2;
        }
    
        // Checking if the message is correctly returned
        // Thus, we copy the N first characters
        char tmp[PSTM_CFG_GAG_OK_LEN] = {'\0'};
        memcpy((void *)tmp, (void *)&this->buf, PSTM_CFG_GAG_OK_LEN);
    
        // Compare the two strings
        if (strcmp(tmp, PSTMCFGAGPSOK) != 0)
        {
            return -2;
        }
    
        // Exit the function
        return 0;
}

uint8_t TESEO::configurePPS()
{
        // First, write the command to the TESEO module
        int ret = this->write((uint8_t *)NMEA_PPSCF, _NMEA_PPSCF_LEN);
        if (ret != 0)
        {
            return -1;
        }
    
        // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
        // This variable shall be configured to true by the ISR !
        while (this->IO_DataReady == false)
        {
            k_yield();
        }
        this->IO_DataReady = false;
    
        // Checking the sanity of the message
        // And returning if something when wrong...
        int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
        if (check != GNSS_PARSER_ERROR)
        {
            return -2;
        }
    
        // Checking if the message is correctly returned
        // Thus, we copy the N first characters
        char tmp[sizeof(PSTM_CFG_PPS_OK_LEN)] = {'\0'};
        memcpy((void *)tmp, (void *)&this->buf, PSTM_CFG_PPS_OK_LEN);
    
        // Compare the two strings
        if (strcmp(tmp, PSTMCFGPPSGENOK) != 0)
        {
            return -2;
        }
    
        // Exit the function
        return 0;
}

uint8_t TESEO::setFreqRange()
{
        // First, write the command to the TESEO module
        int ret = this->write((uint8_t *)NMEA_FREQR, _NMEA_FREQR_LEN);
        if (ret != 0)
        {
            return -1;
        }
    
        // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
        // This variable shall be configured to true by the ISR !
        while (this->IO_DataReady == false)
        {
            k_yield();
        }
        this->IO_DataReady = false;
    
        // Checking the sanity of the message
        // And returning if something when wrong...
        int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
        if (check != GNSS_PARSER_ERROR)
        {
            return -2;
        }
    
        // Checking if the message is correctly returned
        // Thus, we copy the N first characters
        char tmp[sizeof(PSTM_CFG_RAN_OK_LEN)] = {'\0'};
        memcpy((void *)tmp, (void *)&this->buf, PSTM_CFG_RAN_OK_LEN);
    
        // Compare the two strings
        if (strcmp(tmp, PSTMSETRANGEOK) != 0)
        {
            return -2;
        }
    
        // Exit the function
        return 0;
}

uint8_t TESEO::setLocalOscillator()
{
        // First, write the command to the TESEO module
        int ret = this->write((uint8_t *)NMEA_LOCFG, _NMEA_LOCFG_LEN);
        if (ret != 0)
        {
            return -1;
        }
    
        // Exit the function
        return 0;
}

uint8_t TESEO::resetEphemeris()
{
        // First, write the command to the TESEO module
        int ret = this->write((uint8_t *)NMEA_EPHER, _NMEA_EPHER_LEN);
        if (ret != 0)
        {
            return -1;
        }
    
        // Exit the function
        return 0;
}

/* -----------------------------------------------------------------
 * BUS IO
 * -----------------------------------------------------------------
 */
uint8_t TESEO::write(const uint8_t *cmd, uint16_t len)
{

    // Ask for Zephyr to transmit the bytes
    int ret = uart_tx(this->bus, cmd, len, SYS_FOREVER_US);
    if (ret)
    {
        return -1;
    }

    return 0;
}

void TESEO::ISR_RX(const struct device *dev,
                   struct uart_event *evt,
                   void *user_data)
{
    switch (evt->type)
    {
    case UART_RX_RDY:
    {
        // Configure the variable to be ready !
        bool *DataReady = (bool *)user_data;
        *DataReady = true;
        break;

        /*
         * This variable may be triggered by any error while receiving, for example a cable breakdown.
         * Make sure that the link is reliable enough before attempting any operations on it, or the
         * data may be corrupted.
         *
         */
    }

    default:
    {
        LOG_WRN("Unhandled UART RX Event %d", evt->type);
    }
    }

    return;
}