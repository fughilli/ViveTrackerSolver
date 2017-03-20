#ifndef REGISTERS_GPIO_H
#define REGISTERS_GPIO_H

#include "utils/register_utils.h"
#include "utils/static_utils.h"

#include <stdint.h>

namespace RegistersGPIO
{
    struct Map
    {
        uint8_t DATA[0x400];

        uint32_t DIR;           // Offset 0x400
        uint32_t IS;            // Offset 0x404
        uint32_t IBE;           // Offset 0x408
        uint32_t IEV;           // Offset 0x40C
        uint32_t IM;            // Offset 0x410
        uint32_t RIS;           // Offset 0x414
        uint32_t MIS;           // Offset 0x418
        uint32_t ICR;           // Offset 0x41C
        uint32_t AFSEL;         // Offset 0x420

        REGMAP_EMPTY(0x500-0x424, pad0);

        uint32_t DR2R;          // Offset 0x500
        uint32_t DR4R;
        uint32_t DR8R;
        uint32_t ODR;
        uint32_t PUR;
        uint32_t PDR;
        uint32_t SLR;
        uint32_t DEN;
        uint32_t LOCK;
        uint32_t CR;
        uint32_t AMSEL;
        uint32_t PCTL;
        uint32_t ADCCTL;
        uint32_t DMACTL;        // Offset 0x534

        REGMAP_EMPTY(0xFD0-0x538, pad1);

        uint32_t PeriphlD4;     // Offset 0xFD0
        uint32_t PeriphlD5;
        uint32_t PeriphlD6;
        uint32_t PeriphlD7;
        uint32_t PeriphlD0;
        uint32_t PeriphlD1;
        uint32_t PeriphlD2;
        uint32_t PeriphlD3;
        uint32_t PCellD0;
        uint32_t PCellD1;
        uint32_t PCellD2;
        uint32_t PCellD3;       // Offset 0xFFC
    };

    constexpr static Map* const portAMap =
                                RegisterUtil::declareAt<Map, 0x40004000>();
    constexpr static Map* const portBMap =
                                RegisterUtil::declareAt<Map, 0x40005000>();
    constexpr static Map* const portCMap =
                                RegisterUtil::declareAt<Map, 0x40006000>();
    constexpr static Map* const portDMap =
                                RegisterUtil::declareAt<Map, 0x40007000>();
    constexpr static Map* const portEMap =
                                RegisterUtil::declareAt<Map, 0x40024000>();
    constexpr static Map* const portFMap =
                                RegisterUtil::declareAt<Map, 0x40025000>();
}

#endif
