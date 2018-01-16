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
#include "global.h"
#include "ads8568.h"

unsigned int TMR_PERIOD_LSB32   =   0;
unsigned int TMR_PERIOD_MSB32   =   0;


void ads8568_gpio_init(void)
{

    // ���� GPIO5[11]/BUSY Ϊ����ģʽ
    GPIODirModeSet(SOC_GPIO_0_REGS, 92, GPIO_DIR_INPUT);

    // ���� GPIO5[12]/RESET Ϊ���ģʽ
    GPIODirModeSet(SOC_GPIO_0_REGS, 93, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 93, GPIO_PIN_HIGH);

    // ���� GPIO5[13]/CONVST Ϊ���ģʽ
    GPIODirModeSet(SOC_GPIO_0_REGS, 94, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 94, GPIO_PIN_LOW);
}

void ads8568_device_init(unsigned int SamplingRate)
{
    SamplingRate = SamplingRate * SAMPLE_RATE_KHZ;
    // GPIO �ܽŸ�������
    GPIOPinMuxSetup();

    // ��ʱ�� / ��������ʼ��
    if(SamplingRate > 510000) SamplingRate = 510000;
    TMR_PERIOD_LSB32 = 228000000/SamplingRate;
    TimerInit(TMR_PERIOD_LSB32);
    //��ʱ�� / �������жϳ�ʼ��
    TimerInterruptInit();
    // AD8568  IO��ʼ��
    ads8568_gpio_init();
    // EMIF ��ʼ��
    EMIFAInit();
    ads8568_device_reset();
    // ʹ��REF,��������Ϊ+-12V
    ads8568_write_reg(AD8568_WRITE_ENABLE | AD8568_REF_ENABLE |
                    AD8568_REF_SET3V | AD8568_REFDAC_FULL);
}

void ads8568_device_reset(void)
{
    GPIOPinWrite(SOC_GPIO_0_REGS, 93, GPIO_PIN_HIGH);
    Delay(0x1FFF);
    GPIOPinWrite(SOC_GPIO_0_REGS, 93, GPIO_PIN_LOW);
}

void ads8568_start_convert(void)
{

    GPIOPinWrite(SOC_GPIO_0_REGS, 94, GPIO_PIN_LOW);
    GPIOPinWrite(SOC_GPIO_0_REGS, 94, GPIO_PIN_HIGH);
}

void ads8568_write_reg(unsigned int value)
{
    ((short *)SOC_EMIFA_CS2_ADDR)[0] = (value >> 16) & 0x0000FFFF;
    ((short *)SOC_EMIFA_CS2_ADDR)[0] = value & 0x0000FFFF;
}

void Delay(volatile unsigned int count)
{
    while(count--);
}


void EMIFAInit(void)
{
    // ���� EMIFA ��ظ�������
    EMIFAPinMuxSetup();
    // ������������ 16bit
    EMIFAAsyncDevDataBusWidthSelect(SOC_EMIFA_0_REGS,EMIFA_CHIP_SELECT_2, EMIFA_DATA_BUSWITTH_16BIT);
    // ѡ�� Normal ģʽ
    EMIFAAsyncDevOpModeSelect(SOC_EMIFA_0_REGS,EMIFA_CHIP_SELECT_2, EMIFA_ASYNC_INTERFACE_STROBE_MODE );//EMIFA_ASYNC_INTERFACE_NORMAL_MODE
    // ��ֹWAIT����
    EMIFAExtendedWaitConfig(SOC_EMIFA_0_REGS,EMIFA_CHIP_SELECT_2, EMIFA_EXTENDED_WAIT_DISABLE);
    // ���� W_SETUP/R_SETUP W_STROBE/R_STROBE W_HOLD/R_HOLD   TA �Ȳ���
    EMIFAWaitTimingConfig(SOC_EMIFA_0_REGS,EMIFA_CHIP_SELECT_2, EMIFA_ASYNC_WAITTIME_CONFIG(1, 5, 1, 1, 5, 1, 0 ));
}


void TimerInit(unsigned int period)
{
    // ���� ��ʱ�� / ������ 2 Ϊ 64 λģʽ
    TimerConfigure(SOC_TMR_2_REGS, TMR_CFG_64BIT_CLK_INT);
    // ��������
    TimerPeriodSet(SOC_TMR_2_REGS, TMR_TIMER12, period);
    TimerPeriodSet(SOC_TMR_2_REGS, TMR_TIMER34, TMR_PERIOD_MSB32);
    // ʹ�� ��ʱ�� / ������ 2
    TimerEnable(SOC_TMR_2_REGS, TMR_TIMER12, TMR_ENABLE_CONT);
}

void GPIOPinMuxSetup(void)
{
    volatile unsigned int savePinMux = 0;
    savePinMux = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(11)) & \
                         ~(SYSCFG_PINMUX11_PINMUX11_19_16 | SYSCFG_PINMUX11_PINMUX11_15_12 | SYSCFG_PINMUX11_PINMUX11_11_8);
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(11)) = \
             (PINMUX11_BUSY_ENABLE | PINMUX11_CONVST_ENABLE | PINMUX11_RESRT_ENABLE | savePinMux);

}

extern void TimerIsr(void);
void TimerInterruptInit(void)
{
    // ע���жϷ�����
    IntRegister(C674X_MASK_INT5, TimerIsr);
    // ӳ���жϵ� DSP �������ж�
    IntEventMap(C674X_MASK_INT5, SYS_INT_T64P2_TINTALL);
    // ʹ�� DSP �������ж�
    IntEnable(C674X_MASK_INT5);
    // ʹ�� ��ʱ�� / ������ �ж�
    TimerIntEnable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);
}

