#include "bms_store.h"

#include "stm32g4xx_hal.h"

#include <string.h>

static volatile bool s_ecc_hit;
static volatile bool s_reading;
static uint32_t s_last_seq;
static uint32_t s_save_count;
static int8_t s_cur_page = -1;     /* page holding the newest record */
static uint16_t s_next_slot;       /* first erased slot in s_cur_page */

#define SLOTS_PER_PAGE  ((uint16_t)(BMS_STORE_PAGE_BYTES / BMS_STORE_RECORD_BYTES))

static uint32_t slot_addr(uint8_t page, uint16_t slot)
{
    return BMS_STORE_BASE + ((uint32_t)page * BMS_STORE_PAGE_BYTES) +
           ((uint32_t)slot * BMS_STORE_RECORD_BYTES);
}

static bool single_bank(void)
{
    /* The linker map (contiguous 256 KB) requires DBANK=0: 4 KB pages. */
    return (FLASH->OPTR & FLASH_OPTR_DBANK) == 0U;
}

bool BmsStore_HandleEccNmi(void)
{
    uint32_t eccr = FLASH->ECCR;

    if ((eccr & FLASH_ECCR_ECCD) != 0U) {
        /* ADDR_ECC is the double-word offset inside the bank. */
        uint32_t addr = FLASH_BASE + (eccr & FLASH_ECCR_ADDR_ECC);
        bool in_store = (addr >= BMS_STORE_BASE) &&
                        (addr < (BMS_STORE_BASE + BMS_STORE_PAGES * BMS_STORE_PAGE_BYTES));

        if (in_store || s_reading) {
            FLASH->ECCR = eccr | FLASH_ECCR_ECCD; /* write-1-to-clear */
            s_ecc_hit = true;
            return true;
        }
    }
    return false;
}

static bool read_slot(uint8_t page, uint16_t slot, uint8_t buf[BMS_STORE_RECORD_BYTES])
{
    s_ecc_hit = false;
    s_reading = true;
    memcpy(buf, (const void *)slot_addr(page, slot), BMS_STORE_RECORD_BYTES);
    __DSB();
    s_reading = false;
    return !s_ecc_hit;
}

bool BmsStore_Load(BmsStore_Record_t *out)
{
    uint8_t buf[BMS_STORE_RECORD_BYTES];
    bool found = false;
    uint8_t page;
    uint16_t slot;

    s_cur_page = -1;
    s_next_slot = 0U;
    s_last_seq = 0U;
    if (!single_bank()) {
        return false;
    }
    for (page = 0U; page < BMS_STORE_PAGES; page++) {
        for (slot = 0U; slot < SLOTS_PER_PAGE; slot++) {
            BmsStore_Record_t rec;

            if (!read_slot(page, slot, buf)) {
                continue; /* torn write: skip */
            }
            if (BmsStore_IsErased(buf)) {
                break;    /* records are appended; rest of page is empty */
            }
            if (BmsStore_Decode(buf, &rec) && (!found || (rec.seq > s_last_seq))) {
                found = true;
                s_last_seq = rec.seq;
                s_cur_page = (int8_t)page;
                if (out != NULL) {
                    *out = rec;
                }
            }
        }
    }
    if (found) {
        /* Find the first erased slot after the newest record's page head. */
        for (slot = 0U; slot < SLOTS_PER_PAGE; slot++) {
            if (read_slot((uint8_t)s_cur_page, slot, buf) && BmsStore_IsErased(buf)) {
                break;
            }
        }
        s_next_slot = slot;
    }
    return found;
}

static bool erase_page(uint8_t page)
{
    FLASH_EraseInitTypeDef er;
    uint32_t err = 0U;

    memset(&er, 0, sizeof(er));
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.Banks = FLASH_BANK_1;
    er.Page = (BMS_STORE_BASE - FLASH_BASE) / BMS_STORE_PAGE_BYTES + page;
    er.NbPages = 1U;
    return (HAL_FLASHEx_Erase(&er, &err) == HAL_OK) && (err == 0xFFFFFFFFUL);
}

bool BmsStore_Save(const BmsGauge_Persist_t *g, bool clean)
{
    BmsStore_Record_t rec;
    uint8_t buf[BMS_STORE_RECORD_BYTES];
    uint8_t page;
    uint16_t slot;
    uint32_t addr;
    uint32_t k;
    bool ok = true;

    if ((g == NULL) || !single_bank()) {
        return false;
    }
    if (s_cur_page < 0) {
        page = 0U;
        slot = 0U;
        /* No valid record: make sure page 0 is really empty. */
        if (!read_slot(0U, 0U, buf) || !BmsStore_IsErased(buf)) {
            slot = SLOTS_PER_PAGE; /* force erase below */
        }
    } else {
        page = (uint8_t)s_cur_page;
        slot = s_next_slot;
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    if (slot >= SLOTS_PER_PAGE) {
        page = (s_cur_page < 0) ? 0U : (uint8_t)((page + 1U) % BMS_STORE_PAGES);
        ok = erase_page(page);
        slot = 0U;
    }
    if (ok) {
        memset(&rec, 0, sizeof(rec));
        rec.seq = s_last_seq + 1U;
        rec.clean = clean;
        rec.gauge = *g;
        BmsStore_Encode(&rec, buf);
        addr = slot_addr(page, slot);
        for (k = 0U; (k < BMS_STORE_RECORD_BYTES) && ok; k += 8U) {
            uint64_t dw;

            memcpy(&dw, &buf[k], sizeof(dw));
            ok = (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + k, dw) == HAL_OK);
        }
    }
    HAL_FLASH_Lock();

    if (ok) {
        BmsStore_Record_t back;
        uint8_t rb[BMS_STORE_RECORD_BYTES];

        ok = read_slot(page, slot, rb) && BmsStore_Decode(rb, &back) &&
             (back.seq == rec.seq);
    }
    if (ok) {
        s_last_seq = rec.seq;
        s_cur_page = (int8_t)page;
        s_next_slot = (uint16_t)(slot + 1U);
        s_save_count++;
    } else {
        /* Skip the damaged slot next time. */
        s_cur_page = (int8_t)page;
        s_next_slot = (uint16_t)(slot + 1U);
    }
    return ok;
}

uint32_t BmsStore_LastSeq(void)
{
    return s_last_seq;
}

uint32_t BmsStore_SaveCount(void)
{
    return s_save_count;
}
