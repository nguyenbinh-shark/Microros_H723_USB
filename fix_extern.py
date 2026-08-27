with open('User/Bsp/bsp_navkey.c', 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace('extern extern ADC_HandleTypeDef', 'extern ADC_HandleTypeDef')

with open('User/Bsp/bsp_navkey.c', 'w', encoding='utf-8') as f:
    f.write(content)

with open('User/Bsp/bsp_sbus.c', 'r', encoding='utf-8') as f:
    content = f.read()

content = content.replace('extern extern UART_HandleTypeDef', 'extern UART_HandleTypeDef')
content = content.replace('extern extern DMA_HandleTypeDef', 'extern DMA_HandleTypeDef')

with open('User/Bsp/bsp_sbus.c', 'w', encoding='utf-8') as f:
    f.write(content)
print('Fixed extern extern.')
