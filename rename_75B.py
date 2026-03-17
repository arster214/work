import os

src_dir = '/home/fang/catkin_wsfrj/src'
for root, dirs, files in os.walk(src_dir):
    for file in files:
        if file.endswith('.xml') or file.endswith('.launch') or file.endswith('.txt') or file.endswith('.py') or file.endswith('.yaml') or file.endswith('.urdf') or file.endswith('.srdf') or file.endswith('.xacro'):
            filepath = os.path.join(root, file)
            with open(filepath, 'r') as f:
                content = f.read()
            if '75B' in content:
                new_content = content.replace('75B', '75b')
                with open(filepath, 'w') as f:
                    f.write(new_content)
                print(f"Updated content in {filepath}")
