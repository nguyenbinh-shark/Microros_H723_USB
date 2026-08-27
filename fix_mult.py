import re

# Fix bsp_sbus.c
with open('User/Bsp/bsp_sbus.c', 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace('\nUART_HandleTypeDef huart5;\n', '\nextern UART_HandleTypeDef huart5;\n')
content = content.replace('\nDMA_HandleTypeDef hdma_uart5_rx;\n', '\nextern DMA_HandleTypeDef hdma_uart5_rx;\n')

with open('User/Bsp/bsp_sbus.c', 'w', encoding='utf-8') as f:
    f.write(content)

# Fix bsp_navkey.c
with open('User/Bsp/bsp_navkey.c', 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace('\nADC_HandleTypeDef hadc1;\n', '\nextern ADC_HandleTypeDef hadc1;\n')

with open('User/Bsp/bsp_navkey.c', 'w', encoding='utf-8') as f:
    f.write(content)

print('Fixed multiple definitions.')
