typedef struct
{
    uint8_t   header1;
    uint8_t   header2;
    int16_t   linear_speed;
    int16_t   angular_speed;
    uint8_t   airflow;
    uint8_t   pump;
    uint8_t   light;
    uint8_t   checksum;
} MPU_REC_PACK;

typedef struct
{
    uint8_t   header1;
    uint8_t   header2;
    int16_t   linear_speed;
    int16_t   angular_speed;
    uint8_t   airflow;
    uint8_t   pump;
    uint8_t   light;
    uint8_t   checksum;
} MPU_REC_RECEIVED;

MPU_REC_RECEIVED received_data;
uint8_t checksum_sum = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) // 根据实际情况修改 huart1 为你的UART句柄
    {
        if (checksum_sum == received_data.checksum)
        {
            // 校验和有效，可以处理 received_data 结构体的数据
            // ...
        }
        else
        {
            // 校验和无效，处理错误
            // ...
        }

        // 重置校验和和准备接收下一个数据包
        checksum_sum = 0;
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)&received_data, sizeof(MPU_REC_PACK));
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) // 根据实际情况修改 huart1 为你的UART句柄
    {
        // 不执行任何操作
    }
}

void HAL_UART_RxCompleteCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        uint8_t *data_ptr = (uint8_t*)&received_data;

        // 计算校验和
        for (uint32_t i = 0; i < sizeof(MPU_REC_PACK) - 1; i++)
        {
            checksum_sum += data_ptr[i];
        }
    }
}
