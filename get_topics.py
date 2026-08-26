import subprocess
try:
    output = subprocess.check_output(['ign', 'topic', '-l']).decode('utf-8')
except Exception as e:
    output = str(e)
with open('/home/ubuntu/px4_ros2_ws/scratch_topics.txt', 'w') as f:
    f.write(output)
