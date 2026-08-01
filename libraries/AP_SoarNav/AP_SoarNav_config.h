#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Soaring/AP_Soaring_config.h>

#ifndef HAL_SOARNAV_ENABLED
#define HAL_SOARNAV_ENABLED (HAL_SOARING_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024))
#endif

#if HAL_SOARNAV_ENABLED && !HAL_SOARING_ENABLED
#error HAL_SOARNAV_ENABLED requires HAL_SOARING_ENABLED
#endif
