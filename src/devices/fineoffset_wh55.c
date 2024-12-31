/** @file
    Fine Offset Electronics WH55 water leak sensor.
*/

#include "data.h"
#include "decoder.h"
#include "decoder_util.h"

/**
Fine Offset Electronics WH55 water leak sensor,

- also Ecowitt WH55

Samples captured via [1]:

55 0 107a4 05 02 df be a4 49 20 4   # channel 1
55 2 107a4 05 02 e6 be fa c6 20 4   # channel 3
55 3 107a4 05 02 f1 3e cf 36 10 2   # channel 4 / high sensitivity
55 3 107a4 02 80 ac bf 78 7f 08 48  # channel 4 / high sensitivity / ALARM
55 3 107a4 02 80 b9 bf 61 75 0c 090
55 3 107a4 05 02 e1 be db b2 10 2   # channel 4 / low sensitivity
55 3 107a4 05 02 f0 3e 3b a1 20 4

[1] rtl_433 -c 0 -R 0 -X "n=WH55,m=FSK_PCM,s=56,l=56,r=1500,preamble=aa2dd4" -f 868.3M

Preamble is aaaa aaaa, sync word is 2dd4.

Packet layout:

0  1  2 3  4  5  6  7  8  9
55 3 107a4 05 01 94 fe 60 29 10 48  # channel 4 / low sensitivity / ALARM
YY C IIIII 0B 0A UU UU XX CC

Y: 8 bit fixed sensor type 0x55
C: 4 bit channel (setting - 1)
I: 20 bit device ID
B: 3 bit battery bars (0-5)
A: 1 bit leakage alarm (0: alarm, 1: no alarm)
U: unkown data
*/

#define MIN_LEN 150
#define MAX_LEN 220

static int fineoffset_wh55_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble[] = {0xaa, 0x2d, 0xd4}; // 24 bit, part of preamble and sync word
    uint8_t b[10];

    if (bitbuffer->bits_per_row[0] < MIN_LEN || bitbuffer->bits_per_row[0] > MAX_LEN) {
        decoder_logf(decoder, 2, "fineoffset_wh55_decode", "bitbuffer len %d out of range[%d..%d]",
                bitbuffer->bits_per_row[0], MIN_LEN, MAX_LEN);
        return DECODE_ABORT_LENGTH;
    }

    unsigned bit_offset = bitbuffer_search(bitbuffer, 0, 0, preamble, 24) + 24;
    if (bit_offset + sizeof(b) * 8 > bitbuffer->bits_per_row[0]) {
        decoder_logf_bitbuffer(decoder, 1, __func__, bitbuffer, "short package at %u", bit_offset);
        return DECODE_ABORT_LENGTH;
    }

    bitbuffer_extract_bytes(bitbuffer, 0, bit_offset, b, sizeof(b) * 8);

    if (b[0] != 0x55) // Check for family code 0x55
        return DECODE_ABORT_EARLY;

    decoder_log_bitrow(decoder, 1, __func__, b, sizeof(b) * 8, "");

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
    int unknown1     = (b[4] << 8) | b[5];
    int unknown2     = (b[6] << 8) | b[7];
    float battery_ok = battery_bars * 0.2f;

    data_t *data = data_make(
            "model", "", DATA_STRING, "Fineoffset-wh55",
            "id", "ID", DATA_FORMAT, "%06x", DATA_INT, id,
            "channel", "Channel", DATA_INT, channel,
            "battery_ok", "Battery Level", DATA_FORMAT, "%.1f", DATA_DOUBLE, battery_ok,
            "alarm", "Alarm", DATA_INT, alarm,
            "unknown1", "Unknown 1", DATA_FORMAT, "%04x", DATA_INT, unknown1,
            "unknown2", "Unknown 2", DATA_FORMAT, "%04x", DATA_INT, unknown2,
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
        "unknown1",
        "unknown2",
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
