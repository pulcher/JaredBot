#include "settings_eeprom.h"

#include <EEPROM.h>
#include <stddef.h>

namespace {

static const uint16_t kMagic = 0x4A42; // 'J''B'
static const uint8_t kVersion = 1;

struct __attribute__((packed)) SettingsRecord
{
    uint16_t magic;
    uint8_t version;
    uint32_t seq;
    PersistedSettings settings;
    uint16_t crc;
};

static uint16_t crc16_xmodem_update(uint16_t crc, uint8_t data)
{
    crc = crc ^ (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8; i++) {
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
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc = crc16_xmodem_update(crc, data[i]);
    }
    return crc;
}

static uint16_t computeCrc(const SettingsRecord &r)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&r);
    return crc16_xmodem(bytes, offsetof(SettingsRecord, crc));
}

static bool isValid(const SettingsRecord &r)
{
    if (r.magic != kMagic) return false;
    if (r.version != kVersion) return false;
    return computeCrc(r) == r.crc;
}

static int slotAddress(uint8_t slot)
{
    return (slot == 0) ? 0 : (int)sizeof(SettingsRecord);
}

static_assert(sizeof(SettingsRecord) <= 512, "SettingsRecord unexpectedly large");

} // namespace

bool loadPersistedSettings(PersistedSettings &out, uint32_t &seqOut, uint8_t &slotOut)
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

bool savePersistedSettings(const PersistedSettings &in, uint32_t &seqInOut, uint8_t &slotInOut)
{
    SettingsRecord r;
    r.magic = kMagic;
    r.version = kVersion;
    r.seq = seqInOut + 1;
    r.settings = in;
    r.crc = computeCrc(r);

    uint8_t nextSlot = 0;
    if (slotInOut == 0) nextSlot = 1;
    else if (slotInOut == 1) nextSlot = 0;
    else nextSlot = 0;

    EEPROM.put(slotAddress(nextSlot), r);

    // Readback validate (cheap safety net)
    SettingsRecord verify;
    EEPROM.get(slotAddress(nextSlot), verify);
    if (!isValid(verify) || verify.seq != r.seq) {
        return false;
    }

    seqInOut = r.seq;
    slotInOut = nextSlot;
    return true;
}
