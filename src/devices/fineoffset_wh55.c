/** @file
    Fine Offset Electronics WH55 water leak sensor.
*/

#include "data.h"
#include "decoder.h"
#include "decoder_util.h"

/**
rtl_433 -c 0 -R 0 -X "n=WH55,m=FSK_PCM,s=56,l=56,r=1500,preamble=aa2dd4" -f 868.3M

time      : 2023-12-06 20:44:46
model     : DP250        count     : 1             num_rows  : 1             rows      :
len       : 91           data      : 550107a40502dfbea449204
codes     : {91}550107a40502dfbea449204

55 0 107a4 05 02 df be a4 49 20 4   # channel 1
55 0 107a4 05 02 e4 be 0c b6 20 4
55 0 107a4 05 02 e4 be 0c b6 10 2
55 0 107a4 05 02 e4 be 0c b6 10 2
55 0 107a4 05 02 e4 be 0c b6 10 4
55 0 107a4 05 02 df be a4 49 20 4
55 0 107a4 05 02 e4 be 0c b6 10 20
55 0 107a4 05 02 e4 be 0c b6 10 2
55 2 107a4 05 02 e6 be fa c6 20 4   # channel 3
55 3 107a4 05 02 f1 3e cf 36 10 2   # channel 4 / high sensitivity
55 3 107a4 05 03 d8 7d c2 86 40 8
55 3 107a4 05 02 13 3e 60 e9 20 4
55 3 107a4 05 02 e9 3e 96 f5 20 4   # channel 4 / high sensitivity / warm
55 3 107a4 02 80 ac bf 78 7f 08 48  # channel 4 / high sensitivity / ALARM
55 3 107a4 02 80 b9 bf 61 75 0c 090
55 3 107a4 05 02 e1 be db b2 10 2   # channel 4 / low sensitivity
55 3 107a4 05 02 f0 3e 3b a1 20 4

0  1  2 3  4  5  6  7  8  9
55 3 107a4 05 01 94 fe 60 29 10 48  # channel 4 / low sensitivity / ALARM
YY C IIIII 0B AA       XX CC

Y: 8 bit fixed sensor type 0x55
C: 4 bit channel (setting - 1)
I: 20 bit device ID

*/

/**
Fine Offset Electronics WH55 water leak sensor,

- also Ecowitt WH55

Preamble is aaaa aaaa, sync word is 2dd4.

Packet layout:

     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14
    YY II II II 0T TT HH Bp pp BP PP CC CC XX AA
    45 00 36 60 02 7e 36 40 23 00 29 02 29 07 4f

- Y: 8 bit fixed sensor type 0x45
- I: 24 bit device ID
- T: 11 bit temperature, offset 40, scale 10
- H: 8 bit humidity
- B: 1 bit MSB of battery bars out of 5 (a value of 6 indicates external power via USB)
- p: 14 bit PM2.5 reading in ug/m3 * 10
- B: 2 bits LSBs of battery bars out of 5
- P: 14 bit PM10 reading in ug/m3 * 10
- C: 16 bit CO2 reading in ppm
- X: 8 bit CRC
- A: 8 bit checksum

*/

static int fineoffset_wh55_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble[] = {0xaa, 0x2d, 0xd4}; // 24 bit, part of preamble and sync word
    uint8_t b[10];

    // bit counts have been observed between 187 and 222
    if (bitbuffer->bits_per_row[0] < 150 || bitbuffer->bits_per_row[0] > 220) {
        decoder_log(decoder, 1, "fineoffset_wh55_decode", "bitbuffer out of range");
        return DECODE_ABORT_LENGTH;
    }

    // Find a data package and extract data buffer
    unsigned bit_offset = bitbuffer_search(bitbuffer, 0, 0, preamble, 24) + 24;
    if (bit_offset + sizeof(b) * 8 > bitbuffer->bits_per_row[0]) { // Did not find a big enough package
        decoder_logf_bitbuffer(decoder, 2, __func__, bitbuffer, "short package at %u", bit_offset);
        return DECODE_ABORT_LENGTH;
    }

    // Extract package data
    bitbuffer_extract_bytes(bitbuffer, 0, bit_offset, b, sizeof(b) * 8);

    if (b[0] != 0x55) // Check for family code 0x55
        return DECODE_ABORT_EARLY;

    decoder_log_bitrow(decoder, 1, __func__, b, sizeof(b) * 8, "");

    // Verify checksum and CRC
    uint8_t crc = crc8(b, 8, 0x31, 0x00);
    uint8_t chk = add_bytes(b, 9);
    if (crc != b[8] || chk != b[9]) {
        decoder_logf(decoder, 1, __func__, "Checksum error: %02x %02x", crc, chk);
        return DECODE_FAIL_MIC;
    }

    int id           = ((b[1] & 0x0F) << 16) | (b[2] << 8) | (b[3]);
    int channel      = (b[1] >> 4) + 1;
    int alarm        = !(b[5] & 0x02);
    int battery_bars = (b[4] & 0x0F);
    int alarm_raw    = (b[4] << 8) | b[5];
    int raw          = (b[6] << 8) | b[7];
    float battery_ok  = battery_bars * 0.2f; //convert out of 5 bars to 0 (0 bars) to 1 (5 bars)

    //  Battery level is indicated with 5 bars. Convert to 0 (0 bars) to 1 (5 or 6 bars)
    // float battery_ok  = MIN(battery_bars * 0.2f, 1.0f);
    // A battery bars value of 6 means the sensor is powered via USB (the Ecowitt WS View app shows 'DC')
    // int ext_power     = battery_bars == 6 ? 1 : 0;

    data_t *data = data_make(
            "model", "", DATA_STRING, "Fineoffset-wh55",
            "id", "ID", DATA_FORMAT, "%06x", DATA_INT, id,
            "channel", "Channel", DATA_INT, channel,
            "battery_ok",       "Battery Level",  DATA_FORMAT, "%.1f", DATA_DOUBLE, battery_ok,
            "alarm", "Alarm", DATA_INT, alarm,
            "alarm_raw", "Raw1", DATA_FORMAT, "%04x", DATA_INT, alarm_raw,
            "raw", "Raw2", DATA_FORMAT, "%04x", DATA_INT, raw,
            "mic", "Integrity", DATA_STRING, "CRC",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "channel",
        "battery_ok",
        "alarm",
        "alarm_raw",
        "raw",
        "mic",
        NULL,
};

r_device const fineoffset_wh55 = {
        .name        = "Fine Offset Electronics WH55 water leak sensor",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 58,
        .long_width  = 58,
        .reset_limit = 2500,
        .decode_fn   = &fineoffset_wh55_decode,
        .fields      = output_fields,
};
