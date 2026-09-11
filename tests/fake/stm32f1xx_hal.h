#ifndef FAKE_STM32F1XX_HAL_H
#define FAKE_STM32F1XX_HAL_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

typedef struct { int port; } GPIO_TypeDef;
extern GPIO_TypeDef *GPIOA, *GPIOB, *GPIOC, *GPIOD;

#define GPIO_PIN_0 (1u<<0)
#define GPIO_PIN_1 (1u<<1)
#define GPIO_PIN_2 (1u<<2)
#define GPIO_PIN_4 (1u<<4)
#define GPIO_PIN_5 (1u<<5)
#define GPIO_PIN_6 (1u<<6)
#define GPIO_PIN_7 (1u<<7)
#define GPIO_PIN_8 (1u<<8)
#define GPIO_PIN_9 (1u<<9)
#define GPIO_PIN_10 (1u<<10)
#define GPIO_PIN_11 (1u<<11)
#define GPIO_PIN_12 (1u<<12)
#define GPIO_PIN_13 (1u<<13)
#define GPIO_PIN_14 (1u<<14)
#define GPIO_PIN_15 (1u<<15)

#define CAN_ID_STD 0u
#define CAN_ID_EXT 4u
#define CAN_RTR_DATA 0u
#define CAN_RX_FIFO0 0u
#define CAN_FILTERMODE_IDMASK 0u
#define CAN_FILTERMODE_IDLIST 1u
#define CAN_FILTERSCALE_16BIT 0u
#define CAN_FILTERSCALE_32BIT 1u
#define CAN_IT_RX_FIFO0_MSG_PENDING 2u
#define CAN_TX_MAILBOX0 1u
#define CAN_TX_MAILBOX1 2u
#define CAN_TX_MAILBOX2 4u
#define CAN_MCR_NART 0x10u
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 4u
#define TIM_CHANNEL_3 8u
#define TIM_CHANNEL_4 12u
#define TIM_INPUTCHANNELPOLARITY_RISING 0u
#define TIM_ICSELECTION_DIRECTTI 1u
#define TIM_EGR_UG (1u<<0)
#define UART_IT_IDLE 4u

typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC; FunctionalState TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, Timestamp, FilterMatchIndex; } CAN_RxHeaderTypeDef;
typedef struct {
    uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, FilterBank, FilterMode, FilterScale;
    uint32_t FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct { uint32_t MCR; } CAN_InstanceTypeDef;
typedef struct {
    CAN_InstanceTypeDef *Instance;
    struct { FunctionalState AutoRetransmission; } Init;
} CAN_HandleTypeDef;

typedef struct { uint32_t ICPolarity, ICSelection, ICFilter; } TIM_IC_InitTypeDef;
typedef struct { uint32_t ARR, CCR3, EGR; } TIM_InstanceTypeDef;
typedef struct { TIM_InstanceTypeDef *Instance; } TIM_HandleTypeDef;
typedef enum { HAL_TIM_CHANNEL_STATE_RESET = 0, HAL_TIM_CHANNEL_STATE_READY = 1,
               HAL_TIM_CHANNEL_STATE_BUSY = 2 } HAL_TIM_ChannelStateTypeDef;
typedef struct { int dummy; } DMA_HandleTypeDef;
typedef struct { int dummy; } UART_InstanceTypeDef;
typedef struct { UART_InstanceTypeDef *Instance; } UART_HandleTypeDef;
typedef struct { int dummy; } ADC_HandleTypeDef;

/* Enough of the NVIC/ADC surface for app.c to link into a host test. */
typedef enum {
    DMA1_Channel1_IRQn = 11, DMA1_Channel4_IRQn = 14, DMA1_Channel5_IRQn = 15,
    USART1_IRQn = 37, CAN1_RX0_IRQn = 20, CAN2_RX0_IRQn = 64
} IRQn_Type;

extern uint32_t uwTick;
extern uint32_t uwTickFreq;

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t pre, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *buf, uint32_t len);

uint32_t HAL_GetTick(void);
void     HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void     HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t it);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *hdr,
                                       uint8_t *data, uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mailboxes);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *hdr, uint8_t *data);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *h, TIM_IC_InitTypeDef *cfg, uint32_t ch);
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *h, uint32_t ch);
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *h, uint32_t ch);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *h, uint32_t ch);
HAL_TIM_ChannelStateTypeDef TIM_CHANNEL_STATE_GET(TIM_HandleTypeDef *h, uint32_t ch);
void     Error_Handler(void);

#define __HAL_TIM_GET_AUTORELOAD(h)        ((h)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(h, ch, v)    Fake_SetCompare((h), (ch), (v))
#define __HAL_TIM_SET_COUNTER(h, v)        ((void)(v))
#define __HAL_TIM_ENABLE(h)                ((void)(h))
#define __HAL_UART_CLEAR_IDLEFLAG(h)       ((void)(h))
#define CLEAR_BIT(reg, bit)                ((reg) &= ~(bit))
#define SET_BIT(reg, bit)                  ((reg) |= (bit))
#define UNUSED(x)                          ((void)(x))

/* Fake controls, for tests only. */
void     Fake_Reset(void);
void     Fake_SetTick(uint32_t ms);
uint32_t Fake_TxCount(void);
int      Fake_FindTx(uint32_t stdId, uint8_t out[8], uint32_t *atTick);
uint32_t Fake_TxCountFor(uint32_t stdId);
GPIO_PinState Fake_PinState(GPIO_TypeDef *port, uint16_t pin);
void     Fake_SetPin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
/* GPIO init surface: app.c's fatal path re-opens GPIOB's clock gate and
   reconfigures RED_LD, because the SystemClock_Config fault sites run before
   MX_GPIO_Init. Recorded so a test can assert it happened. */
#define GPIO_MODE_OUTPUT_PP   (0x01u)
#define GPIO_NOPULL           (0x00u)
#define GPIO_SPEED_FREQ_LOW   (0x00u)
#define __HAL_RCC_GPIOB_CLK_ENABLE()  Fake_EnableGpioClock(GPIOB)

typedef struct { uint32_t Pin; uint32_t Mode; uint32_t Pull; uint32_t Speed; } GPIO_InitTypeDef;

void     HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *cfg);
void     Fake_EnableGpioClock(GPIO_TypeDef *port);
uint8_t  Fake_GpioClockEnabled(GPIO_TypeDef *port);
uint32_t Fake_GpioConfiguredPins(GPIO_TypeDef *port);

uint32_t Fake_LastCompare(void);
void     Fake_SetCompare(TIM_HandleTypeDef *h, uint32_t ch, uint32_t v);
void     Fake_QueueUartRx(const uint8_t *data, uint16_t len);
uint16_t Fake_LastUartTx(uint8_t *out, uint16_t cap);
void     Fake_ForceUartTxFail(void);
/* Queues a received frame, dropped immediately if no configured filter bank
   owned by h accepts stdId - this is what models hardware filtering. */
void     Fake_QueueCanRx(CAN_HandleTypeDef *h, uint32_t stdId, const uint8_t *data, uint8_t dlc);
/* Number of not-yet-drained frames queued for h. */
uint32_t Fake_RxPending(CAN_HandleTypeDef *h);
/* TX mailboxes of h not currently holding a frame (0..3). */
uint32_t Fake_CanTxMailboxesFree(CAN_HandleTypeDef *h);
/* Hold every TX mailbox forever: an unacknowledged frame under retransmission. */
void     Fake_HoldCanTx(int on);
/* HAL_CAN_AbortTxRequest calls seen since the last Fake_Reset. */
uint32_t Fake_CanAbortCount(void);
#endif
