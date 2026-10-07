#ifndef BSP_DELAY_HPP
#define BSP_DELAY_HPP

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void delay_init(void);
    bool delay_is_ready(void);
    void delay_us(uint16_t nus);
    void delay_ms(uint16_t nms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DELAY_HPP */