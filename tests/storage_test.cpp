#include "storage_integrity.h"
#include <cassert>
#include <cstdio>
int main() {
    const char* text = "123456789";
    assert(StorageIntegrity::Crc32(text, 9) == 0xcbf43926U);
    uint16_t table[] = {0, 1, 51199};
    auto crc = StorageIntegrity::Crc32(table, sizeof table);
    StorageIntegrity::CalibrationHeader h{StorageIntegrity::Magic, 1, sizeof table, crc, StorageIntegrity::Committed};
    assert(StorageIntegrity::Valid(h, table, sizeof table));
    h.commit = 0xffffffffU;
    assert(!StorageIntegrity::Valid(h, table, sizeof table));
    h.commit = StorageIntegrity::Committed;
    table[1] ^= 1;
    assert(!StorageIntegrity::Valid(h, table, sizeof table));
    table[1] ^= 1;
    h.version = 2;
    assert(!StorageIntegrity::Valid(h, table, sizeof table));
    std::puts("storage: CRC vector, torn commit, corruption, version guards passed");
}
