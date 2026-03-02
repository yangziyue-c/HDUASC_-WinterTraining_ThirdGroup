#include "path_recorder.h"
#include "pid.h"
#include "mpu6050.h"
#include "Encoder.h"
#include <string.h>
#include <math.h>

extern float yaw;
extern float LeftSpeed, RightSpeed;
extern PID_t SpeedPID, TurnPID;
extern uint8 Recorder_Flag, Tracking_Flag;

PathManager_t path_manager;
float Navigation_Speed, Navigation_Turn;
float Yaw_offset = 0;           // 新增：yaw偏移量
float Yaw_start_record = 0;     // 新增：录制起始yaw

// Flash头部结构
typedef struct {
    uint32_t magic;
    uint16_t count;
    uint16_t checksum;
	float yaw_start;            // 新增：保存录制起始yaw
    uint32_t reserved;
} FlashHeader_t;

static float Normalize_Angle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static uint16_t Calculate_Checksum(void)
{
    uint32_t sum = 0;
    uint8_t *data = (uint8_t*)path_manager.points;
    uint32_t len = path_manager.count * sizeof(PathPoint_t);
    for (uint32_t i = 0; i < len; i++)
        sum += data[i];
    return (uint16_t)(sum & 0xFFFF);
}

void PathTracking_Init(void)
{
    memset(&path_manager, 0, sizeof(PathManager_t));
    Navigation_Speed = 0;
    Navigation_Turn = 0;
	Yaw_offset = 0;
    Yaw_start_record = 0;
}

// 开始录制时调用
void PathRecording_Start(void)
{
    PathTracking_Init();
    Yaw_start_record = yaw;     // 记录起始yaw
}

// 开始回放时调用
void PathTracking_Start(void)
{
    path_manager.current_index = 0;
    // 计算yaw偏移：当前yaw - 录制时的起始yaw
    Yaw_offset = yaw - Yaw_start_record;
}

void Record_PathPoint(void)
{
    if (path_manager.count >= MAX_PATH_POINTS)
    {
        Recorder_Flag = 0;
        return;
    }
    
    PathPoint_t *p = &path_manager.points[path_manager.count];
    p->left_speed = LeftSpeed;
    p->right_speed = RightSpeed;
    p->yaw = yaw;
    path_manager.count++;
}

void Navigation_Calculate(void)
{
    if (Tracking_Flag == 1)
    {
        if (path_manager.current_index >= path_manager.count)
        {
            Navigation_Speed = 0;
            Navigation_Turn = 0;
            SpeedPID.Target = 0;
            TurnPID.Target = 0;
            SpeedPID.ErrorInt = 0;
            TurnPID.ErrorInt = 0;
            Tracking_Flag = 0;
            path_manager.current_index = 0;
            return;
        }
        
        PathPoint_t *target = &path_manager.points[path_manager.current_index];
        
        float target_ave_speed = (target->left_speed + target->right_speed) / 2.0f;
        float target_dif_speed = target->left_speed - target->right_speed;
		
		float yaw_correction = 0;
		
		if (fabsf(target->left_speed) > 0.1f || fabsf(target->right_speed) > 0.1f)
        {
           // 简化：目标yaw = 录制的yaw + 偏移
            float target_yaw = target->yaw + Yaw_offset;
            float yaw_error = Normalize_Angle(target_yaw - yaw);
		
		
        // 如果速度很小，减小航向修正（静止时不修正）
		if (fabsf(yaw_error) > 5.0f)
        {
            // 限制修正量，避免过猛
            if (yaw_error > 30.0f) yaw_error = 30.0f;
            if (yaw_error < -30.0f) yaw_error = -30.0f;
            
            yaw_correction = -YAW_CORRECTION_KP * yaw_error;
            
            // 限制修正输出
            if (yaw_correction > 0.3f) yaw_correction = 0.3f;
            if (yaw_correction < -0.3f) yaw_correction = -0.3f;
			}
		}
        SpeedPID.Target = target_ave_speed;
        TurnPID.Target = target_dif_speed + yaw_correction;
        
        Navigation_Speed = target_ave_speed;
        Navigation_Turn = target_dif_speed + yaw_correction;
        
        path_manager.current_index++;
    }
}

void Path_SaveToFlash(void)
{
    if (path_manager.count == 0) return;
    
    FlashHeader_t header;
    header.magic = FLASH_MAGIC_WORD;
    header.count = path_manager.count;
    header.checksum = Calculate_Checksum();
	header.yaw_start = Yaw_start_record;    // 保存起始yaw
    header.reserved = 0;
	
    uint32_t page_size = FLASH_DATA_BUFFER_SIZE * 4;
    uint32_t data_size = path_manager.count * sizeof(PathPoint_t);
    uint32_t total_size = sizeof(FlashHeader_t) + data_size;
    uint32_t pages_needed = (total_size + page_size - 1) / page_size;

    for (int i = 0; i < PATH_FLASH_SECTOR_COUNT; i++)
    {
        for (int j = 0; j < FLASH_PAGE_NUM; j++)
        {
            flash_erase_page(PATH_FLASH_SECTOR_START + i, j);
        }
    }

    uint8_t *src_ptr = (uint8_t*)&header;
    uint32_t src_remaining = sizeof(FlashHeader_t);
    uint32_t data_written = 0;
    uint8_t phase = 0;

    for (uint32_t i = 0; i < pages_needed; i++)
    {
        flash_buffer_clear();
        uint32_t buf_pos = 0;

        if (phase == 0 && src_remaining > 0)
        {
            uint32_t copy_len = (src_remaining > page_size) ? page_size : src_remaining;
            for (uint32_t j = 0; j < copy_len / 4; j++)
                flash_union_buffer[buf_pos++].uint32_type = ((uint32_t*)src_ptr)[j];
            src_ptr += copy_len;
            src_remaining -= copy_len;
            if (src_remaining == 0)
            {
                phase = 1;
                src_ptr = (uint8_t*)path_manager.points;
            }
        }

        if (phase == 1 && data_written < data_size)
        {
            uint32_t space = page_size - buf_pos * 4;
            uint32_t copy_len = (data_size - data_written > space) ? space : (data_size - data_written);
            for (uint32_t j = 0; j < copy_len / 4; j++)
                flash_union_buffer[buf_pos++].uint32_type = ((uint32_t*)src_ptr)[data_written / 4 + j];
            data_written += copy_len;
        }

        uint32_t sector = PATH_FLASH_SECTOR_START + (i / FLASH_PAGE_NUM);
        flash_page_enum page = (flash_page_enum)(i % FLASH_PAGE_NUM);
        flash_write_page_from_buffer(sector, page);
    }
}

uint8_t Path_LoadFromFlash(void)
{
    FlashHeader_t header;
    uint32_t page_size = FLASH_DATA_BUFFER_SIZE * 4;
    
    flash_read_page_to_buffer(PATH_FLASH_SECTOR_START, FLASH_PAGE_0);
    
    for (uint32_t i = 0; i < sizeof(FlashHeader_t) / 4; i++)
        ((uint32_t*)&header)[i] = flash_union_buffer[i].uint32_type;

    if (header.magic != FLASH_MAGIC_WORD || header.count > MAX_PATH_POINTS || header.count == 0)
    {
        PathTracking_Init();
        return 0;
    }

    uint32_t data_size = header.count * sizeof(PathPoint_t);
    uint32_t total_size = sizeof(FlashHeader_t) + data_size;
    uint32_t pages_needed = (total_size + page_size - 1) / page_size;
    uint32_t data_read = 0;
    uint32_t header_size = sizeof(FlashHeader_t);

    for (uint32_t i = 0; i < pages_needed && data_read < data_size; i++)
    {
        uint32_t sector = PATH_FLASH_SECTOR_START + (i / FLASH_PAGE_NUM);
        flash_page_enum page = (flash_page_enum)(i % FLASH_PAGE_NUM);
        flash_read_page_to_buffer(sector, page);

        uint32_t start_pos = (i == 0) ? (header_size / 4) : 0;
        uint32_t available = FLASH_DATA_BUFFER_SIZE - start_pos;
        uint32_t needed = (data_size - data_read) / 4;
        uint32_t copy_count = (available < needed) ? available : needed;

        for (uint32_t j = 0; j < copy_count; j++)
            ((uint32_t*)path_manager.points)[data_read / 4 + j] = flash_union_buffer[start_pos + j].uint32_type;
        data_read += copy_count * 4;
    }

    path_manager.count = header.count;
    path_manager.current_index = 0;
	Yaw_start_record = header.yaw_start;    // 恢复起始yaw
	
    if (Calculate_Checksum() != header.checksum)
    {
        PathTracking_Init();
        return 0;
    }

    return 1;
}