#ifndef VISION_H
#define VISION_H

#include <stdbool.h>
#include <stdint.h>

#define VISION_FRAME_SIZE 13U
#define VISION_SEQUENCE_RESET_MS 100U
#define VISION_RAW_BYTES 24U

/** 当前参考路径观测；sample_id 仅在接受独立新测量后递增。 */
typedef struct
{
	float dev;
	float k;
	uint32_t received_ms;
	uint32_t sample_id;
	uint8_t seq;
	uint8_t elem;
	uint8_t phase;
	uint8_t conf;
	uint8_t valid;
	bool lost;
} vision_sample_t;

/** 链路诊断；跳号仅表示不连续，不直接等同物理串口丢包。 */
typedef struct
{
	vision_sample_t sample;
	uint32_t accepted;
	uint32_t rejected;
	uint32_t duplicates;
	uint32_t discontinuities;
	uint32_t bytes_received;
	uint32_t last_byte_ms;
	uint32_t sync_hits;
	uint32_t checked_frames;
	uint32_t tail_errors;
	uint32_t checksum_errors;
	uint32_t range_errors;
	uint32_t sequence_errors;
	uint8_t raw[VISION_RAW_BYTES];
	uint8_t raw_length;
} vision_status_t;

/** @brief 清空接收状态。 @retval 无。 */
void Vision_Init(void);

/** @brief 输入一个原始字节，在主循环调用。
 * @param byte 原始字节。
 * @param now_ms 本地接收毫秒时刻。
 * @retval true 接受了一个独立新测量。
 */
bool Vision_FeedByte(uint8_t byte, uint32_t now_ms);

/** @brief 获取只读状态副本。 @retval 当前接收状态。 */
vision_status_t Vision_GetStatus(void);

#endif
