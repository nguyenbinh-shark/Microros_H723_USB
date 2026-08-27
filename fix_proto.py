import sys

with open('Core/Src/freertos.c', 'r', encoding='utf-8') as f:
    content = f.read()

prototypes = "\n/* micro-ROS Entities Helper Functions */\nstatic bool create_entities(void);\nstatic void destroy_entities(void);\n"
start_proto = content.find('/* USER CODE BEGIN FunctionPrototypes */\n') + len('/* USER CODE BEGIN FunctionPrototypes */\n')

content = content[:start_proto] + prototypes + content[start_proto:]

with open('Core/Src/freertos.c', 'w', encoding='utf-8') as f:
    f.write(content)
print("Prototypes added successfully.")
