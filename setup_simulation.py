import os
import shutil
import xml.etree.ElementTree as ET

CLOVER_LAUNCH_DIR = os.path.expanduser('~/catkin_ws/src/clover/clover/launch')
SIM_LAUNCH_DIR = os.path.expanduser('~/catkin_ws/src/clover/clover_simulation/launch')

CUSTOM_WORLD_NAME = "nto.world"
CUSTOM_WORLD_PATH = f'$(find clover_simulation)/resources/worlds/{CUSTOM_WORLD_NAME}'

CONFIGS = {
    'clover.launch': {
        'dir': CLOVER_LAUNCH_DIR,
        'updates': {'aruco': 'true'}
    },
    'aruco.launch': {
        'dir': CLOVER_LAUNCH_DIR,
        'updates': {
            'aruco_detect': 'true',
            'aruco_map': 'true',
            'aruco_vpe': 'true',
            'placement': 'floor',
            'length': '0.22',
            'map': 'cmit.txt'
        }
    },
    'simulator.launch': {
        'dir': SIM_LAUNCH_DIR,
        'updates': {
            'world_name': CUSTOM_WORLD_PATH
        }
    }
}

def modify_launch_file(filename, directory, updates):
    full_path = os.path.join(directory, filename)
    
    if not os.path.exists(full_path):
        print(f"[ERROR] Файл не найден: {full_path}")
        return

    # Создаем бэкап
    backup_path = full_path + '.bak'
    if not os.path.exists(backup_path):
        shutil.copy2(full_path, backup_path)
        print(f"[{filename}] [INFO] Создан бэкап.")
    
    try:
        tree = ET.parse(full_path)
        root = tree.getroot()
        modified = False
        
        if filename == 'simulator.launch':
            search_areas = root.findall('include')
        else:
            search_areas = [root]

        for area in search_areas:
            for arg in area.findall('arg'):
                name = arg.get('name')
                if name in updates:
                    new_value = updates[name]
                    
                    attr_to_set = 'value' if 'value' in arg.attrib else 'default'
                    current_value = arg.get(attr_to_set)

                    if current_value != new_value:
                        arg.set(attr_to_set, new_value)
                        modified = True
                    
        if modified:
            tree.write(full_path, encoding='utf-8', xml_declaration=True)
            print(f"[{filename}] [OK] Успешно обновлен.")
        else:
            print(f"[{filename}] [SKIP] Изменения не требуются.")

    except ET.ParseError as e:
        print(f"[{filename}] [ERROR] Не удалось распарсить: {e}")

def main():
    print("--- Настройка окружения Clover (3 файла) ---")
    
    for filename, config in CONFIGS.items():
        modify_launch_file(filename, config['dir'], config['updates'])
    
    print("\n--- Настройка завершена. Переходим к Генерации мира ---")

if __name__ == '__main__':
    main()


### NEED TESTS ###