#include "vision.h"
#include <string.h>

/** 接收窗口、序号判定和发布观测均由同一主循环拥有。 */
typedef struct
{
	vision_status_t status;
	uint8_t bytes[VISION_FRAME_SIZE];
	uint8_t length;
	bool has_sample;
} vision_state_t;

static vision_state_t vision_state;

/** @brief 校验定长窗口。 @retval true 字节及字段范围合法。 */
static bool Vision_CheckFrame(void)
{
	const uint8_t *frame_p = vision_state.bytes;
	uint8_t sum = 0U;
	uint8_t index = 0U;
	int32_t dev_raw = ((uint32_t)frame_p[5] << 8) | frame_p[6];

	if (dev_raw >= 32768)
	{
		dev_raw -= 65536;
	}

	for (index = 2U; index <= 10U; index++)
	{
		sum = (uint8_t)(sum + frame_p[index]);
	}

	return frame_p[0] == 0x55U && frame_p[1] == 0xAAU &&
		frame_p[12] == 0x0DU && frame_p[11] == sum &&
		frame_p[3] <= 6U && frame_p[4] <= 3U &&
		frame_p[8] <= 100U && frame_p[9] <= 100U &&
		frame_p[10] <= 1U && dev_raw >= -1000 && dev_raw <= 1000;
}

/** @brief 发布新测量，拒绝重复和短时间内的倒序帧。
 * @param now_ms 当前接收时刻。
 * @retval true 已更新观测。
 */
static bool Vision_Publish(uint32_t now_ms)
{
	const uint8_t *frame_p = vision_state.bytes;
	vision_status_t *status_p = &vision_state.status;
	uint8_t delta = (uint8_t)(frame_p[2] - status_p->sample.seq);
	int32_t dev_raw = ((uint32_t)frame_p[5] << 8) | frame_p[6];
	int16_t k_raw = frame_p[7];

	if (vision_state.has_sample && delta == 0U)
	{
		status_p->duplicates++;

		return false;
	}

	if (vision_state.has_sample && delta >= 128U &&
		(uint32_t)(now_ms - status_p->sample.received_ms) <
		VISION_SEQUENCE_RESET_MS)
	{
		status_p->rejected++;

		return false;
	}

	if (vision_state.has_sample && delta != 1U)
	{
		status_p->discontinuities++;
	}

	if (dev_raw >= 32768)
	{
		dev_raw -= 65536;
	}

	if (k_raw >= 128)
	{
		k_raw -= 256;
	}

	status_p->sample.dev = (float)dev_raw / 1000.0f;
	status_p->sample.k = (float)k_raw / 100.0f;
	status_p->sample.seq = frame_p[2];
	status_p->sample.elem = frame_p[3];
	status_p->sample.phase = frame_p[4];
	status_p->sample.conf = frame_p[8];
	status_p->sample.valid = frame_p[9];
	status_p->sample.lost = frame_p[10] != 0U;
	status_p->sample.received_ms = now_ms;
	status_p->sample.sample_id++;
	status_p->accepted++;
	vision_state.has_sample = true;

	return true;
}

void Vision_Init(void)
{
	memset(&vision_state, 0, sizeof(vision_state));
}

bool Vision_FeedByte(uint8_t byte, uint32_t now_ms)
{
	bool accepted = false;

	vision_state.bytes[vision_state.length++] = byte;

	while (vision_state.length >= 2U &&
		(vision_state.bytes[0] != 0x55U ||
		vision_state.bytes[1] != 0xAAU))
	{
		vision_state.length--;
		memmove(vision_state.bytes, &vision_state.bytes[1],
			vision_state.length);
	}

	if (vision_state.length == sizeof(vision_state.bytes))
	{
		if (Vision_CheckFrame())
		{
			accepted = Vision_Publish(now_ms);
			vision_state.length = 0U;
		}
		else
		{
			vision_state.status.rejected++;
			vision_state.length--;
			memmove(vision_state.bytes, &vision_state.bytes[1],
				vision_state.length);
		}
	}

	return accepted;
}

vision_status_t Vision_GetStatus(void)
{
	return vision_state.status;
}
