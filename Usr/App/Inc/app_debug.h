#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>


#ifdef DEBUG

#define DEBUG_PRINT(fmt, ...) \
do{printf("[DEBUG] %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__);}while(0)

#endif

#ifdef __cplusplus
}
#endif
