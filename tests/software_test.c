#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "protocol.h"
#include "cobs.h"
#include "i2c.h"
#include "spi.h"

TestHardware test_hardware;

/* 保存回调结果，确认错误帧不会进入应用层。 */
typedef struct
{
	uint32_t frames;
	uint8_t cmd;
	uint8_t len;
	uint8_t payload[PROTOCOL_MAX_PAYLOAD];
} TestFrames;

static TestFrames test_frames;

/** @brief 比较控制输出，允许浮点舍入误差。 */
static void Test_ExpectFloat(float actual, float expected)
{
	assert(fabsf(actual - expected) < 0.0001f);
}

/** @brief 验证增量累加、积分限幅、输出限幅和复位。 */
static void Test_PID(void)
{
	PID_Controller pid;

	PID_Init(&pid, 2.0f, 0.5f, 0.25f, 100.0f, 100.0f);
	Test_ExpectFloat(PID_Compute(&pid, -1.0f), 2.75f);
	Test_ExpectFloat(PID_Compute(&pid, -1.0f), 3.0f);
	Test_ExpectFloat(PID_Compute(&pid, -2.0f), 6.25f);

	PID_Init(&pid, 0.0f, 0.0f, 1.0f, 100.0f, 100.0f);
	Test_ExpectFloat(PID_Compute(&pid, 0.0f), 0.0f);
	Test_ExpectFloat(PID_Compute(&pid, -1.0f), 1.0f);
	Test_ExpectFloat(PID_Compute(&pid, -2.0f), 1.0f);

	PID_Init(&pid, 0.0f, 1.0f, 0.0f, 2.0f, 100.0f);
	Test_ExpectFloat(PID_Compute(&pid, -2.0f), 2.0f);
	Test_ExpectFloat(PID_Compute(&pid, -2.0f), 2.0f);
	Test_ExpectFloat(PID_Compute(&pid, 1.0f), 1.0f);

	PID_Init(&pid, 10.0f, 0.0f, 0.0f, 100.0f, 3.0f);
	Test_ExpectFloat(PID_Compute(&pid, -1.0f), 3.0f);
	Test_ExpectFloat(PID_Compute(&pid, -2.0f), 3.0f);
	PID_Reset(&pid);
	Test_ExpectFloat(pid.output, 0.0f);
	Test_ExpectFloat(pid.integral, 0.0f);
	Test_ExpectFloat(PID_Compute(&pid, 1.0f), -3.0f);
}

/** @brief 将一个字节交给真实 UART 接收中断处理。 */
static void Test_FeedByte(UART_Handle *uart_p, uint8_t byte)
{
	test_hardware.uart_rx_byte = byte;
	UART_HandleIRQ(uart_p);
}

/** @brief 记录协议回调，检查载荷上限。 */
static void Test_OnFrame(uint8_t cmd, const uint8_t *payload_p,
						 uint8_t len)
{
	assert(len <= sizeof(test_frames.payload));
	test_frames.frames++;
	test_frames.cmd = cmd;
	test_frames.len = len;
	memcpy(test_frames.payload, payload_p, len);
}

/** @brief 验证所有合法载荷长度、超长帧丢弃及分隔符恢复。 */
static void Test_Protocol(UART_Handle *uart_p)
{
	uint8_t payload[PROTOCOL_MAX_PAYLOAD];
	uint16_t len;
	uint16_t i;
	uint32_t frames;
	uint16_t wire_len;

	Protocol_Init(Test_OnFrame, uart_p);

	for (i = 0; i < sizeof(payload); i++)
	{
		payload[i] = (i % 3 == 0) ? 0 : (uint8_t)i;
	}

	for (len = 0; len <= sizeof(payload); len++)
	{
		test_hardware.uart_tx_count = 0;
		frames = test_frames.frames;
		Protocol_SendFrame(0x25, payload, (uint8_t)len);

		for (i = 0; i < test_hardware.uart_tx_count; i++)
		{
			Test_FeedByte(uart_p, test_hardware.uart_tx[i]);
		}

		Protocol_Update();
		assert(test_frames.frames == frames + 1);
		assert(test_frames.cmd == 0x25 && test_frames.len == len);
		assert(memcmp(test_frames.payload, payload, len) == 0);
	}

	test_hardware.uart_tx_count = 0;
	Protocol_SendFrame(1, payload, sizeof(payload) + 1);
	Protocol_SendFrame(1, NULL, 1);
	assert(test_hardware.uart_tx_count == 0);

	/* 超长帧末尾含合法帧片段，也必须整帧丢弃。 */
	Protocol_SendFrame(0x30, payload, 1);
	wire_len = test_hardware.uart_tx_count;
	frames = test_frames.frames;

	for (i = 0; i < 68; i++)
	{
		Test_FeedByte(uart_p, 1);
	}

	for (i = 0; i < wire_len; i++)
	{
		Test_FeedByte(uart_p, test_hardware.uart_tx[i]);
	}

	Protocol_Update();
	assert(test_frames.frames == frames);

	for (i = 0; i < wire_len; i++)
	{
		Test_FeedByte(uart_p, test_hardware.uart_tx[i]);
	}

	Protocol_Update();
	assert(test_frames.frames == frames + 1);

	/* 短块与错误校验不能触发应用回调。 */
	Test_FeedByte(uart_p, 5);
	Test_FeedByte(uart_p, 1);
	Test_FeedByte(uart_p, 0);
	Test_FeedByte(uart_p, 3);
	Test_FeedByte(uart_p, 1);
	Test_FeedByte(uart_p, 2);
	Test_FeedByte(uart_p, 0);
	Protocol_Update();
	assert(test_frames.frames == frames + 1);
}

/** @brief 验证串口行边界、格式化截断与等待超时。 */
static void Test_UART(UART_Handle *uart_p)
{
	uint16_t i;
	char text[300];
	uint8_t data[] = {1, 2, 3};
	uint32_t dma_starts;

	UART_StartReceive(uart_p);
	uart_p->rxRawHead = 123;
	uart_p->rxRawTail = 45;

	for (i = 0; i < sizeof(uart_p->rxBuf) - 1; i++)
	{
		Test_FeedByte(uart_p, 'a');
	}

	Test_FeedByte(uart_p, '\n');
	assert(uart_p->rxDone && uart_p->rxLen == 255);
	assert(uart_p->rxBuf[255] == 0);

	UART_StartReceive(uart_p);

	for (i = 0; i < 300; i++)
	{
		Test_FeedByte(uart_p, 'b');
	}

	Test_FeedByte(uart_p, '\n');
	assert(!uart_p->rxDone && uart_p->rxPos == 0);
	assert(uart_p->rxRawHead == 123 && uart_p->rxRawTail == 45);
	Test_FeedByte(uart_p, 'x');
	Test_FeedByte(uart_p, '\n');
	assert(uart_p->rxDone && uart_p->rxLen == 1);
	assert(uart_p->rxBuf[0] == 'x' && uart_p->rxBuf[1] == 0);

	memset(text, 'c', sizeof(text) - 1);
	text[sizeof(text) - 1] = 0;
	test_hardware.uart_tx_count = 0;
	assert(UART_Printf(uart_p, "%s", text) == 255);
	assert(test_hardware.uart_tx_count == 255);
	UART_PrintfDMA(uart_p, "%s", text);
	assert(test_hardware.dma_len == 255);
	dma_starts = test_hardware.dma_starts;
	UART_PrintfDMA(uart_p, "%s", "must not overwrite active DMA");
	UART_SendStrDMA(uart_p, "x", 1);
	assert(test_hardware.dma_starts == dma_starts);
	assert(!uart_p->txDMADone);

	test_hardware.uart_tx_full = true;
	test_hardware.uart_tx_polls = 0;
	test_hardware.uart_tx_count = 0;
	UART_SendData(uart_p, data, sizeof(data));
	assert(test_hardware.uart_tx_count == 0);
	assert(test_hardware.uart_tx_polls <= UART_BUSY_TIMEOUT);
	assert(UART_SendStr(uart_p, "timeout") == 0);
	UART_SendByte(uart_p, 1);
	test_hardware.uart_tx_full = false;
}

/** @brief 记录 I2C 超时后外设重新初始化。 */
static void Test_I2CInit(void)
{
	test_hardware.i2c_init_calls++;
}

void delay_ms(unsigned long ms) {}

/** @brief 验证外设永久忙、无回包和正常传输均能结束。 */
static void Test_BusTimeouts(void)
{
	SPI_Config spi_config = {.spi = SPI0};
	SPI_Handle *spi_p = SPI_Init(&spi_config);
	I2C_Config i2c_config = {.i2c = I2C0, .syscfgInit = Test_I2CInit};
	I2C_Handle *i2c_p = I2C_Init(&i2c_config);
	uint8_t data[] = {0, 0xAF};

	assert(spi_p != NULL && i2c_p != NULL);
	test_hardware.spi_rx_byte = 0xA5;
	assert(SPI_Transfer(spi_p, 1) == 0xA5);
	test_hardware.spi_tx_full = true;
	assert(SPI_Transfer(spi_p, 1) == 0);
	assert(test_hardware.spi_sends == 1);
	test_hardware.spi_tx_full = false;
	test_hardware.spi_rx_empty = true;
	assert(SPI_Transfer(spi_p, 1) == 0);
	test_hardware.spi_rx_empty = false;
	test_hardware.spi_busy = true;
	assert(SPI_Transfer(spi_p, 1) == 0);
	test_hardware.spi_busy = false;
	assert(SPI_Transfer(spi_p, 1) == 0xA5);

	I2C_Write(i2c_p, 0x3C, data, sizeof(data));
	assert(test_hardware.i2c_starts == 0);
	assert(test_hardware.i2c_init_calls == 1);
	test_hardware.i2c_idle = true;
	I2C_Write(i2c_p, 0x3C, data, sizeof(data));
	assert(test_hardware.i2c_starts == 1);
	assert(test_hardware.i2c_init_calls == 2);
	test_hardware.i2c_done = true;
	I2C_Write(i2c_p, 0x3C, data, sizeof(data));
	assert(test_hardware.i2c_starts == 2);
	assert(test_hardware.i2c_init_calls == 2);
}

int main(void)
{
	UART_Config config = {.uart = UART0};
	UART_Handle *uart_p = UART_Init(&config);

	assert(uart_p != NULL);
	Test_PID();
	Test_Protocol(uart_p);
	Test_UART(uart_p);
	Test_BusTimeouts();
	puts("PASS: PID, protocol boundaries, UART boundaries, bus timeouts");

	return 0;
}
