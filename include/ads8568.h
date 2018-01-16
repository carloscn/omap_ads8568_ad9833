// OMAPL138 & TMS320C674X Project.
/****************************************************************************/
/*                                                                          */
/*  @file       : *_ad8568.c                                                */
/*  @Copyright  : Lifi.MLT (c) 2018 MULTIBEANS_ORG All rights reserved.     */
/*  @Revision   : Ver 1.0.                                                  */
/*  @Git        : https://github.com/lifimlt/omapl_ads8568_ad9833.git       */
/*  @Data       : 2018.01.16 Realse.                                        */
/*  @Belong     : PROJECT.                                                  */
/*                                                                          */
/*  **code : (GBK/GB2312) in Ubuntu 16.04. CCS v7.4 platform.               */
/****************************************************************************/
/*  @Attention:                                                             */
/*  ---------------------------------------------------------------------   */
/*  |    Data    |  Behavior |     Offer     |          Content         |   */
/*  | 2018.01.16 |   create  |Carlos Wei (M) | ceate the document.      |   */
/*  ---------------------------------------------------------------------   */
/*                                                            MULTIBEANS.   */
/****************************************************************************/
#ifndef INCLUDE_ADS8568_H_
#define INCLUDE_ADS8568_H_

#define AD8568_WRITE_ENABLE     (0x80000000u)
#define AD8568_READ_ENABLE      (0x40000000u)
#define AD8568_CLKSEL_INTERNAL  (0x00000000u)
#define AD8568_CLKSEL_EXTERNAL  (0x20000000u)
#define AD8568_BUSY_PIN_MODE    (0x00000000u)
#define AD8568_INT_PIN_MODE     (0x08000000u)
#define AD8568_BUSY_POL_HIGH    (0x00000000u)
#define AD8568_BUSY_POL_LOW     (0x04000000u)
#define AD8568_PowerDown        (0x02000000u)
#define AD8568_RangeA_4VREF     (0x00000000u)
#define AD8568_RangeA_2VREF     (0x01000000u)
#define AD8568_RangeB_4VREF     (0x00000000u)
#define AD8568_RangeB_2VREF     (0x00800000u)
#define AD8568_RangeC_4VREF     (0x00000000u)
#define AD8568_RangeC_2VREF     (0x00200000u)
#define AD8568_RangeD_4VREF     (0x00000000u)
#define AD8568_RangeD_2VREF     (0x00080000u)
#define AD8568_PowerUpB         (0x00000000u)
#define AD8568_PowerDownB       (0x00400000u)
#define AD8568_PowerUpC         (0x00000000u)
#define AD8568_PowerDownC       (0x00100000u)
#define AD8568_PowerUpD         (0x00000000u)
#define AD8568_PowerDownD       (0x00040000u)
#define AD8568_REF_DISABLE      (0x00000000u)
#define AD8568_REF_ENABLE       (0x00008000u)
#define AD8568_REFBUF_ENABLE    (0x00000000u)
#define AD8568_REFBUF_DISABLE   (0x00004000u)
#define AD8568_REF_SET2V5       (0x00000000u)
#define AD8568_REF_SET3V        (0x00002000u)
#define AD8568_REFDAC_FULL      (0x000003FFu)


extern void ads8568_gpio_init(void);
extern void ads8568_device_init(unsigned int SamplingRate);
extern void ads8568_device_reset(void);
extern void ads8568_start_convert(void);

extern void ads8568_write_reg(unsigned int value);
extern void Delay(volatile unsigned int count);
extern void EMIFAInit(void);
extern void GPIOPinMuxSetup(void);
extern void TimerInit(unsigned int period);
extern void TimerInterruptInit(void);
#endif /* INCLUDE_ADS8568_H_ */
