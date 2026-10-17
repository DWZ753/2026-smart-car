#ifndef TEST_MSP_DL_CONFIG_H
#define TEST_MSP_DL_CONFIG_H

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

/* 主机测试使用的外设占位类型，不访问 MCU 寄存器。 */
typedef struct { uint32_t TXDATA; uint32_t RXDATA; } UART_Regs;
typedef struct { uint32_t unused; } SPI_Regs;
typedef struct { uint32_t unused; } I2C_Regs;
typedef struct { uint32_t unused; } GPIO_Regs;
typedef struct { uint32_t unused; } DMA_Regs;
typedef int IRQn_Type;

/* 各外设状态和调用计数，用于模拟成功、等待超时和恢复路径。 */
typedef struct
{
	UART_Regs uart[4];
	SPI_Regs spi[2];
	I2C_Regs i2c[2];
	DMA_Regs dma;
	bool uart_tx_full;
	uint32_t uart_tx_polls;
	uint8_t uart_rx_byte;
	uint32_t uart_rx_fifo[32];
	uint8_t uart_rx_count;
	uint8_t uart_rx_position;
	int uart_pending;
	bool uart_fifo_enabled;
	uint32_t uart_rx_threshold;
	uint32_t uart_rx_timeout;
	uint32_t uart_interrupts;
	uint8_t uart_tx[2048];
	uint16_t uart_tx_count;
	uint16_t dma_len;
	uint32_t dma_starts;
	bool spi_tx_full;
	bool spi_rx_empty;
	bool spi_busy;
	uint8_t spi_rx_byte;
	uint32_t spi_tx_polls;
	uint32_t spi_rx_polls;
	uint32_t spi_busy_polls;
	uint32_t spi_sends;
	bool i2c_idle;
	bool i2c_done;
	bool i2c_nack;
	bool i2c_flush_stuck;
	uint32_t i2c_idle_delay;
	uint32_t i2c_done_delay;
	uint32_t i2c_flush_polls;
	uint32_t i2c_flush_starts;
	uint32_t i2c_flush_stops;
	uint32_t i2c_starts;
	uint32_t i2c_resets;
	uint32_t i2c_init_calls;
} TestHardware;

extern TestHardware test_hardware;

#define UART0 (&test_hardware.uart[0])
#define UART1 (&test_hardware.uart[1])
#define UART2 (&test_hardware.uart[2])
#define UART3 (&test_hardware.uart[3])
#define SPI0 (&test_hardware.spi[0])
#define SPI1 (&test_hardware.spi[1])
#define I2C0 (&test_hardware.i2c[0])
#define I2C_OLED_INST I2C0
#define GPIO_I2C_OLED_SCL_PORT ((GPIO_Regs *)0)
#define GPIO_I2C_OLED_SDA_PORT ((GPIO_Regs *)0)
#define GPIO_I2C_OLED_SCL_PIN 0U
#define GPIO_I2C_OLED_SDA_PIN 0U
#define GPIO_I2C_OLED_IOMUX_SCL 0U
#define GPIO_I2C_OLED_IOMUX_SDA 0U
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC 0U
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC 0U
static inline void SYSCFG_DL_I2C_OLED_init(void) {}
#define I2C1 (&test_hardware.i2c[1])
#define DMA (&test_hardware.dma)
#define DL_UART_IIDX_DMA_DONE_TX 1
#define DL_UART_IIDX_RX 2
#define DL_UART_IIDX_RX_TIMEOUT_ERROR 3
#define DL_UART_RX_FIFO_LEVEL_ONE_ENTRY 0U
#define DL_UART_INTERRUPT_RX 1U
#define DL_UART_INTERRUPT_RX_TIMEOUT_ERROR 2U
#define DL_UART_ERROR_FRAMING (1U << 8)
#define DL_UART_ERROR_PARITY (1U << 9)
#define DL_UART_ERROR_BREAK (1U << 10)
#define DL_UART_ERROR_OVERRUN (1U << 11)
#define DL_I2C_CONTROLLER_STATUS_IDLE 1U
#define DL_I2C_INTERRUPT_CONTROLLER_TX_DONE 1U
#define DL_I2C_INTERRUPT_CONTROLLER_NACK 2U
#define DL_I2C_CONTROLLER_SDA_LOW 0U
#define DL_I2C_CONTROLLER_DIRECTION_TX 0
#define DL_GPIO_INVERSION_DISABLE 0
#define DL_GPIO_RESISTOR_NONE 0
#define DL_GPIO_HYSTERESIS_DISABLE 0
#define DL_GPIO_WAKEUP_DISABLE 0

static inline void NVIC_ClearPendingIRQ(IRQn_Type irq) {}
static inline void NVIC_EnableIRQ(IRQn_Type irq) {}

static inline uint32_t __get_PRIMASK(void)
{
	return 0U;
}

static inline void __disable_irq(void)
{
}

static inline void __set_PRIMASK(uint32_t value)
{
	(void)value;
}

static inline void DL_UART_enableFIFOs(UART_Regs *uart)
{
	test_hardware.uart_fifo_enabled = true;
}

static inline void DL_UART_setRXFIFOThreshold(UART_Regs *uart, uint32_t level)
{
	test_hardware.uart_rx_threshold = level;
}

static inline void DL_UART_setRXInterruptTimeout(UART_Regs *uart, uint32_t value)
{
	test_hardware.uart_rx_timeout = value;
}

static inline void DL_UART_enableInterrupt(UART_Regs *uart, uint32_t mask)
{
	test_hardware.uart_interrupts |= mask;
}

static inline void DL_UART_disableInterrupt(UART_Regs *uart, uint32_t mask)
{
	test_hardware.uart_interrupts &= ~mask;
}

static inline bool DL_UART_isRXFIFOEmpty(UART_Regs *uart)
{
	if (test_hardware.uart_rx_position == test_hardware.uart_rx_count)
	{
		return true;
	}

	uart->RXDATA = test_hardware.uart_rx_fifo[test_hardware.uart_rx_position++];

	return false;
}

static inline bool DL_UART_isTXFIFOFull(const UART_Regs *uart)
{
	test_hardware.uart_tx_polls++;

	return test_hardware.uart_tx_full;
}

static inline void DL_UART_transmitData(UART_Regs *uart, uint8_t data)
{
	assert(test_hardware.uart_tx_count < sizeof(test_hardware.uart_tx));
	test_hardware.uart_tx[test_hardware.uart_tx_count++] = data;
}

static inline int DL_UART_getPendingInterrupt(UART_Regs *uart)
{
	return test_hardware.uart_pending == 0 ?
		DL_UART_IIDX_RX : test_hardware.uart_pending;
}

static inline uint8_t DL_UART_receiveData(UART_Regs *uart)
{
	return test_hardware.uart_rx_byte;
}

static inline void DL_DMA_setSrcAddr(DMA_Regs *dma, uint8_t ch,
									uint32_t addr) {}
static inline void DL_DMA_setDestAddr(DMA_Regs *dma, uint8_t ch,
										 uint32_t addr) {}

static inline void DL_DMA_setTransferSize(DMA_Regs *dma, uint8_t ch,
											 uint16_t len)
{
	test_hardware.dma_len = len;
}

static inline void DL_DMA_enableChannel(DMA_Regs *dma, uint8_t ch)
{
	test_hardware.dma_starts++;
}

static inline bool DL_SPI_isTXFIFOFull(const SPI_Regs *spi)
{
	test_hardware.spi_tx_polls++;

	return test_hardware.spi_tx_full;
}

static inline bool DL_SPI_isRXFIFOEmpty(const SPI_Regs *spi)
{
	test_hardware.spi_rx_polls++;

	return test_hardware.spi_rx_empty;
}

static inline bool DL_SPI_isBusy(const SPI_Regs *spi)
{
	test_hardware.spi_busy_polls++;

	return test_hardware.spi_busy;
}

static inline void DL_SPI_transmitData8(SPI_Regs *spi, uint8_t data)
{
	test_hardware.spi_sends++;
}

static inline uint8_t DL_SPI_receiveData8(SPI_Regs *spi)
{
	return test_hardware.spi_rx_byte;
}

static inline void DL_I2C_reset(I2C_Regs *i2c)
{
	test_hardware.i2c_resets++;
}

static inline uint32_t DL_I2C_getControllerStatus(I2C_Regs *i2c)
{
	if (test_hardware.i2c_idle_delay > 0U)
	{
		test_hardware.i2c_idle_delay--;

		return 0U;
	}

	return test_hardware.i2c_idle ? DL_I2C_CONTROLLER_STATUS_IDLE : 0;
}

static inline bool DL_I2C_getRawInterruptStatus(I2C_Regs *i2c,
													  uint32_t mask)
{
	if (mask == DL_I2C_INTERRUPT_CONTROLLER_TX_DONE &&
		test_hardware.i2c_done_delay > 0U)
	{
		test_hardware.i2c_done_delay--;

		return false;
	}

	return mask == DL_I2C_INTERRUPT_CONTROLLER_NACK ?
		test_hardware.i2c_nack : test_hardware.i2c_done;
}

static inline void DL_I2C_startControllerTransfer(I2C_Regs *i2c,
		uint8_t addr, int dir, uint16_t len)
{
	test_hardware.i2c_starts++;
}

static inline uint16_t DL_I2C_fillControllerTXFIFO(I2C_Regs *i2c,
		const uint8_t *data, uint16_t len)
{
	return len;
}

static inline void DL_I2C_flushControllerTXFIFO(I2C_Regs *i2c) {}

static inline void DL_I2C_startFlushControllerTXFIFO(I2C_Regs *i2c)
{
	test_hardware.i2c_flush_starts++;
}

static inline bool DL_I2C_isControllerTXFIFOEmpty(I2C_Regs *i2c)
{
	test_hardware.i2c_flush_polls++;

	return !test_hardware.i2c_flush_stuck;
}

static inline void DL_I2C_stopFlushControllerTXFIFO(I2C_Regs *i2c)
{
	test_hardware.i2c_flush_stops++;
}
static inline void DL_I2C_resetControllerTransfer(I2C_Regs *i2c) {}
static inline uint32_t DL_I2C_getSDAStatus(I2C_Regs *i2c) { return 1U; }

static inline void DL_I2C_clearInterruptStatus(I2C_Regs *i2c,
		uint32_t mask) {}
static inline void DL_I2C_enablePower(I2C_Regs *i2c) {}
static inline void DL_GPIO_initDigitalOutput(uint32_t iomux) {}
static inline void DL_GPIO_initDigitalInputFeatures(uint32_t iomux,
		int inversion, int resistor, int hysteresis, int wakeup) {}
static inline void DL_GPIO_initPeripheralInputFunctionFeatures(
		uint32_t iomux, uint32_t func, int inversion, int resistor,
		int hysteresis, int wakeup) {}
static inline void DL_GPIO_clearPins(GPIO_Regs *port, uint32_t pin) {}
static inline void DL_GPIO_setPins(GPIO_Regs *port, uint32_t pin) {}
static inline void DL_GPIO_enableOutput(GPIO_Regs *port, uint32_t pin) {}
static inline void DL_GPIO_enableHiZ(uint32_t iomux) {}

static inline uint32_t DL_GPIO_readPins(GPIO_Regs *port, uint32_t pin)
{
	return 1;
}

#endif
