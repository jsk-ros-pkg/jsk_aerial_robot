

#ifdef __cplusplus
 extern "C" {
#endif

void Rosserial_RxCpltCallback(UART_HandleTypeDef *huart);

void GPS_RxCpltCallback(UART_HandleTypeDef *huart);
void GPS_ErrorCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
