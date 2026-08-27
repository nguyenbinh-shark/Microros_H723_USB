import sys

with open('Core/Src/stm32h7xx_it.c', 'r', encoding='utf-8') as f:
    content = f.read()

extern_decl = "\nextern DMA_HandleTypeDef hdma_uart5_rx;\n"
start_ev = content.find('/* USER CODE BEGIN EV */\n') + len('/* USER CODE BEGIN EV */\n')

content = content[:start_ev] + extern_decl + content[start_ev:]

with open('Core/Src/stm32h7xx_it.c', 'w', encoding='utf-8') as f:
    f.write(content)
print('Fixed stm32h7xx_it.c')
