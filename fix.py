import os

with open('Core/Src/freertos.c', 'r', encoding='utf-8') as f:
    content = f.read()

start_var = content.find('/* ── micro-ROS State Machine Definitions')
end_var = content.find('#define MSTAT_CMD_TIMEOUT   (1U << 5)\n') + len('#define MSTAT_CMD_TIMEOUT   (1U << 5)\n')
variables = content[start_var:end_var]
content = content[:start_var] + content[end_var:]

start_var_block = content.find('/* USER CODE BEGIN Variables */\n') + len('/* USER CODE BEGIN Variables */\n')
content = content[:start_var_block] + variables + content[start_var_block:]

start_func = content.find('/* ── Create & Destroy Entities Helper Functions')
end_func_str = 'Waiting for Agent...\\r\\n\");\n}'
end_func = content.find(end_func_str) + len(end_func_str) + 1
functions = content[start_func:end_func]
content = content[:start_func] + content[end_func:]

start_app_block = content.find('/* USER CODE BEGIN Application */\n') + len('/* USER CODE BEGIN Application */\n')
content = content[:start_app_block] + functions + '\n' + content[start_app_block:]

with open('Core/Src/freertos.c', 'w', encoding='utf-8') as f:
    f.write(content)

print('Moved blocks successfully.')
