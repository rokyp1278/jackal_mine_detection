from setuptools import find_packages, setup
import os

package_name = 'jackal_mine_detection'

# ── install share 폴더에 포함할 데이터 파일 ────────────────────────────
data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# launch/, config/, worlds/, models/, scripts/ 폴더를 재귀적으로 포함
for top_dir in ('launch', 'config', 'worlds', 'models', 'scripts', 'urdf'):
    for root, dirs, files in os.walk(top_dir):
        # __pycache__ 제외
        dirs[:] = [d for d in dirs if d != '__pycache__']
        if files:
            rel = os.path.relpath(root, '.')
            data_files.append((
                os.path.join('share', package_name, rel),
                [os.path.join(root, f) for f in files],
            ))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='park minseok',
    maintainer_email='rokyp1278@gmail.com',
    description='Jackal UGV Indoor Mine (AprilTag) Detection pipeline',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_mine_publisher_node = jackal_mine_detection.fake_mine_publisher_node:main',
            'mine_recorder_node       = jackal_mine_detection.mine_recorder_node:main',
            'mine_cluster_node        = jackal_mine_detection.mine_cluster_node:main',
            'exploration_manager_node = jackal_mine_detection.exploration_manager_node:main',
            'mine_goal_sender_node    = jackal_mine_detection.mine_goal_sender_node:main',
            'waypoint_follower_node       = jackal_mine_detection.waypoint_follower_node:main',
            'sim_apriltag_detector_node   = jackal_mine_detection.sim_apriltag_detector_node:main',
            'sim_mine_detector_node       = jackal_mine_detection.sim_mine_detector_node:main',
            'frontier_explorer_node       = jackal_mine_detection.frontier_explorer_node:main',
            'tag_yolo_detector_node       = jackal_mine_detection.tag_yolo_detector_node:main',
            'tag_recorder_node            = jackal_mine_detection.tag_recorder_node:main',
        ],
    },
)
