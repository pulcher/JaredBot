#include "persistence.h"

#include <EEPROM.h>
#include <stddef.h>

namespace {

static const uint16_t kMagic = 0x4242;
static const uint8_t kVersion = 4;

struct SettingsRecord
{
    uint16_t magic;
    uint8_t version;
    uint32_t seq;
    ControlSettings settings;
    uint16_t crc;
};

static uint16_t crc16_xmodem_update(uint16_t crc, uint8_t data)
{
    crc = crc ^ (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc = (crc << 1);
        }
    }
    return crc;
}

static uint16_t crc16_xmodem(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc = crc16_xmodem_update(crc, data[i]);
    }
    return crc;
}

static uint16_t computeCrc(const SettingsRecord &record)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
    return crc16_xmodem(bytes, offsetof(SettingsRecord, crc));
}

static bool isValid(const SettingsRecord &record)
{
    return record.magic == kMagic &&
           record.version == kVersion &&
           computeCrc(record) == record.crc;
}

static int slotAddress(uint8_t slot)
{
    return (slot == 0) ? 0 : (int)sizeof(SettingsRecord);
}

static_assert(sizeof(SettingsRecord) <= 512, "Settings record unexpectedly large");

} // namespace

bool loadPersistedSettings(ControlSettings &out, uint32_t &seqOut, uint8_t &slotOut)
{
    SettingsRecord a;
    SettingsRecord b;
    EEPROM.get(slotAddress(0), a);
    EEPROM.get(slotAddress(1), b);

    const bool aOk = isValid(a);
    const bool bOk = isValid(b);

    if (!aOk && !bOk) {
        slotOut = 255;
        seqOut = 0;
        return false;
    }

    const SettingsRecord *best = nullptr;
    uint8_t bestSlot = 255;

    if (aOk && bOk) {
        if (b.seq >= a.seq) {
            best = &b;
            bestSlot = 1;
        } else {
            best = &a;
            bestSlot = 0;
        }
    } else if (aOk) {
        best = &a;
        bestSlot = 0;
    } else {
        best = &b;
        bestSlot = 1;
    }

    out = best->settings;
    seqOut = best->seq;
    slotOut = bestSlot;
    return true;
}

bool savePersistedSettings(const ControlSettings &in, uint32_t &seqInOut, uint8_t &slotInOut)
{
    SettingsRecord record;
    record.magic = kMagic;
    record.version = kVersion;
    record.seq = seqInOut + 1;
    record.settings = in;
    record.crc = computeCrc(record);

    uint8_t nextSlot = 0;
    if (slotInOut == 0) {
        nextSlot = 1;
    } else if (slotInOut == 1) {
        nextSlot = 0;
    }

    EEPROM.put(slotAddress(nextSlot), record);

    SettingsRecord verify;
    EEPROM.get(slotAddress(nextSlot), verify);
    if (!isValid(verify) || verify.seq != record.seq) {
        return false;
    }

    seqInOut = record.seq;
    slotInOut = nextSlot;
    return true;
}
