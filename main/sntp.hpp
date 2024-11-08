#ifndef SNTP_H
#define SNTP_H
#include <cstdint>

void init_sntp();
int64_t get_timestamp_microseconds();
#endif
