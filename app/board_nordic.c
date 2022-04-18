
#include "board_nordic.h"


nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

static uint64_t m_tickLast = 0;
static uint64_t m_elpsedTime_us = 0;
APP_TIMER_DEF(m_one_sec_timer);

static volatile bool m_xfer_done = false;

volatile uint8_t fifo_flag = 0;

static inline uint64_t calcElapsedTicks(uint64_t tickNow, uint64_t tickLast)
{
    if (tickNow >= tickLast)
        return tickNow - tickLast;
    else
        return APP_TIMER_MAX_CNT_VAL - tickLast + tickNow;
}

static uint64_t platsystickTicksToUs(uint64_t tick)
{
    return (uint64_t)(tick * 1000000 / APP_TIMER_CLOCK_FREQ);
}

static void systickUpdate()
{
    uint64_t tickNow;
    uint64_t delta_ticks;

    tickNow = app_timer_cnt_get();
    delta_ticks = calcElapsedTicks(tickNow, m_tickLast);
    m_elpsedTime_us += platsystickTicksToUs(delta_ticks);
    m_tickLast = tickNow;
}

static void one_sec_timer_handle(void *p_context)
{
    systickUpdate();
}

static void timer_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_one_sec_timer, APP_TIMER_MODE_REPEATED, one_sec_timer_handle);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_one_sec_timer, TIMER_PERIOD_1_SEC, NULL);
    APP_ERROR_CHECK(err_code);
}

void uart_error_handle(app_uart_evt_t *p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
    { RX_PIN_NUMBER,
      TX_PIN_NUMBER,
      RTS_PIN_NUMBER,
      CTS_PIN_NUMBER,
      UART_HWFC,
      false,
			UART_BAUD_SIZE
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/** 配置溢出中断事件
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_OVERFLOW)
    {
       nrf_gpio_pin_toggle(BSP_LED_0);
			 nrf_rtc_task_trigger(rtc.p_reg,NRF_RTC_TASK_TRIGGER_OVERFLOW);//重新触发一次溢出事件
    }

}

static void rtc_config(void)
{
    uint32_t err_code;

    //初始化RTC
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);
	
	  nrf_rtc_task_trigger(rtc.p_reg,NRF_RTC_TASK_TRIGGER_OVERFLOW);//触发溢出事件
    //使能溢出通道和溢出中断
    nrf_drv_rtc_overflow_enable(&rtc,true);
    APP_ERROR_CHECK(err_code);

    //使能RTC模块
    nrf_drv_rtc_enable(&rtc);

}




void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
    switch (p_event->type)
    {
    case NRF_DRV_TWI_EVT_DONE:
        m_xfer_done = true;
        break;
    default:
        break;
    }
}

static void i2c_init(void)
{
    ret_code_t err_code;
#ifdef aikemu52832
    const nrf_drv_twi_config_t twi_config = {
        .scl = IMU_SCL_PIN,
        .sda = IMU_SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init = false};	
#else
    const nrf_drv_twi_config_t twi_config = {
        .scl = ARDUINO_SCL_PIN,
        .sda = ARDUINO_SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init = false};	
#endif


    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    fifo_flag = 1;
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(EVB_INT1, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(EVB_INT1, true);
}

static void nordic_system_init(void)
{
    bsp_board_init(BSP_INIT_NONE);
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void board_init(void)
{
    nordic_system_init();
    timer_init();
    uart_init();
    i2c_init();
    gpio_init();
		rtc_config();
}

int CWM_OS_i2cRead(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *readData, int readDataSize, int argument)
{
	 uint16_t cnt = 0;  
	 int err_code;
	 m_xfer_done = false;
	 err_code = nrf_drv_twi_tx(&m_twi, slaveAddr, (uint8_t *)&reg, regLength, true);
	 while (m_xfer_done == false)
	 {
			cnt++;
			__nop();
			if (cnt > 10000)
			break;
	 }
	 if (err_code != NRF_SUCCESS)
	 return err_code;

	 m_xfer_done = false;
	 err_code = nrf_drv_twi_rx(&m_twi, slaveAddr, readData, readDataSize);
	 while (m_xfer_done == false)
	 {
			cnt++;
			__nop();
			if (cnt > 10000)
			break;
	 }
	 return err_code;
}

int CWM_OS_i2cWrite(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, int argument)
{
		uint16_t cnt = 0;  
		int err_code;
		uint8_t data[20];
		memset(data, 0, sizeof(data));
		memcpy(data, &reg, regLength);
		memcpy(&data[regLength], writeData, writeDataSize);

		m_xfer_done = false;
		err_code = nrf_drv_twi_tx(&m_twi, slaveAddr, data, writeDataSize + regLength, false);
		while (m_xfer_done == false)
		{
		 cnt++;
		 __nop();
		 if (cnt > 10000)
		 break;
		}
		return err_code;
}

void CWM_OS_uSleep(uint32_t time)
{
    uint32_t ms = time / 1000;

    if (ms == 0)
        ms = 1;
    nrf_delay_ms(ms);
}

uint64_t CWM_OS_GetTimeNs(void)
{
    uint64_t tick;
    uint64_t delta_ticks;
    uint64_t timeNow_us;

		__disable_irq();
    tick = app_timer_cnt_get();
    delta_ticks = calcElapsedTicks(tick, m_tickLast);
    timeNow_us = m_elpsedTime_us + platsystickTicksToUs(delta_ticks);
		__enable_irq();
	
    return timeNow_us * 1000;
}

void *CWM_OS_malloc(int size)
{
    if (size == 0)
        size = 1;
    return malloc(size);
}

void CWM_OS_free(void *ptr)
{
    if (ptr == NULL)
        return;

    free(ptr);
}

#if USE_JLINK_PRINTF
int CWM_OS_dbgOutput(const char *format)
{
    NRF_LOG_INFO("%s", format);
    NRF_LOG_FLUSH();

    return 0;
}

int CWM_OS_dbgOutput_checkLog(const char *format)
{
    static int s_sn = 0;

    NRF_LOG_INFO("#_#|%d|%s", s_sn, format);
    s_sn++;
    NRF_LOG_FLUSH();

    return 0;
}

int CWM_OS_dbgPrintf(const char *format, ...)
{
    char InputBuff[255];
    va_list args;

    va_start(args, format);

    vsprintf(InputBuff, format, args);

    va_end(args);
    CWM_OS_dbgOutput(InputBuff);
    return 0;
}
#else
int CWM_OS_dbgOutput(const char *format)
{
    printf("%s", format);
    return 0;
}

int CWM_OS_dbgOutput_checkLog(const char *format)
{
    static int s_sn = 0;

    printf("#_#|%d|%s", s_sn, format);
    s_sn++;

    return 0;
}

int CWM_OS_dbgPrintf(const char *format, ...)
{
    va_list argList;
    int size = 0;
    static char tBuffer[255];

    va_start(argList, format);
    size = vsnprintf(tBuffer, sizeof(tBuffer), format, argList);
    va_end(argList);

    CWM_OS_dbgOutput(tBuffer);
    return size;
}
#endif
