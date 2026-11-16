#include "telemetry/NhatKyNghienCuu.h"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
void check(bool value, const char* label) {
    if (!value) { std::cerr << "FAIL: " << label << '\n'; std::exit(1); }
}
struct SerialNho {
    int capacity = 64;
    size_t partial = 64;
    std::string output;
    int availableForWrite() const { return capacity; }
    size_t write(const uint8_t* data, size_t length) {
        check(length <= static_cast<size_t>(capacity), "never exceed reported free serial space");
        const auto sent = std::min(length, partial);
        output.append(reinterpret_cast<const char*>(data), sent);
        return sent;
    }
};
}

int main() {
    NhatKyNghienCuu log;
    SerialNho serial;
    const std::string header = "NC_HEADER," + std::string(310, 'h') + "\n";
    const std::string sample = "NC," + std::string(390, '1') + "\n";
    check(log.them(header.data(), header.size()), "long header accepted independently of USB 64-byte capacity");
    serial.capacity = 0;
    log.phat(serial);
    check(serial.output.empty() && log.conDuLieu(), "USB backpressure preserves queued data");
    serial.capacity = 64;
    serial.partial = 17;
    log.phat(serial);
    check(log.them(sample.data(), sample.size()), "can append while a prior line is partially sent");
    size_t iterations = 0;
    while (log.conDuLieu() && ++iterations < 1000) log.phat(serial);
    check(!log.conDuLieu() && serial.output == header + sample, "partial writes preserve complete ordered records");

    NhatKyNghienCuu full;
    const std::string block(2000, 'a');
    check(full.them(block.data(), block.size()), "finite buffer accepts near-capacity record");
    check(!full.them(sample.data(), sample.size()), "full queue rejects a complete record without partial enqueue");
    SerialNho drain;
    while (full.conDuLieu()) full.phat(drain);
    check(drain.output == block, "rejected record does not corrupt prior data");
    check(full.them("OK", 2, true), "queue reusable after draining");
    while (full.conDuLieu()) full.phat(drain);
    check(drain.output == block + "OK\n", "appended newline is exact");
    std::cout << "PASS: research serial framing, 64-byte USB chunks, short writes and backpressure\n";
}
