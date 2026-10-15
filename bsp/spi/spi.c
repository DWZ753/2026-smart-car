#include "spi.h"
#include "ti_msp_dl_config.h"

#define SPI_MAX_INSTANCES 2
#define SPI_TIMEOUT_CNT 100000U

/* ========== 句柄注册表 ========== */
static SPI_Handle  g_handle_pool[SPI_MAX_INSTANCES];
static SPI_Handle *g_spi_instances[SPI_MAX_INSTANCES];
static uint8_t     g_spi_count;

static uint8_t get_spi_index(SPI_Regs *spi)
{
    if (spi == SPI0) return 0;
    if (spi == SPI1) return 1;
    return SPI_MAX_INSTANCES;
}

/* ========== 通用 API ========== */

SPI_Handle* SPI_Init(const SPI_Config *config)
{
    uint8_t idx;

    if (config == NULL || config->spi == NULL) return NULL;
    idx = get_spi_index(config->spi);
    if (idx >= SPI_MAX_INSTANCES) return NULL;
    if (g_spi_instances[idx] != NULL) return NULL;
    if (g_spi_count >= SPI_MAX_INSTANCES) return NULL;

    SPI_Handle *h = &g_handle_pool[g_spi_count];
    h->spi = config->spi;

    g_spi_instances[idx] = h;
    g_spi_count++;

    return h;
}

uint8_t SPI_Transfer(SPI_Handle *h, uint8_t tx_data)
{
    uint32_t timeout = SPI_TIMEOUT_CNT;
    uint8_t rx;

    while (DL_SPI_isTXFIFOFull(h->spi))
    {
        if (--timeout == 0)
        {
            return 0;
        }
    }

    DL_SPI_transmitData8(h->spi, tx_data);
    timeout = SPI_TIMEOUT_CNT;
    while (DL_SPI_isRXFIFOEmpty(h->spi))
    {
        if (--timeout == 0)
        {
            return 0;
        }
    }

    rx = DL_SPI_receiveData8(h->spi);
    timeout = SPI_TIMEOUT_CNT;
    while (DL_SPI_isBusy(h->spi))
    {
        if (--timeout == 0)
        {
            return 0;
        }
    }

    return rx;
}
