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
    if (huart == &huart1) // ����ʵ������޸� huart1 Ϊ���UART���
    {
        if (checksum_sum == received_data.checksum)
        {
            // У�����Ч�����Դ��� received_data �ṹ�������
            // ...
        }
        else
        {
            // У�����Ч���������
            // ...
        }

        // ����У��ͺ�׼��������һ�����ݰ�
        checksum_sum = 0;
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)&received_data, sizeof(MPU_REC_PACK));
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) // ����ʵ������޸� huart1 Ϊ���UART���
    {
        // ��ִ���κβ���
    }
}

void HAL_UART_RxCompleteCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        uint8_t *data_ptr = (uint8_t*)&received_data;

        // ����У���
        for (uint32_t i = 0; i < sizeof(MPU_REC_PACK) - 1; i++)
        {
            checksum_sum += data_ptr[i];
        }
    }
}
