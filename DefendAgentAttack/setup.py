from setuptools import find_packages, setup

package_name = 'DefendAgentAttack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanimn',
    maintainer_email='hanimn@uchicago.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reward-simulator = DefendAgentAttack.reward_simlator', 
            'multi-agent = DefendAgentAttack.multiagent.py',
            'fight-now = DefendAgentAttack.fightnow.py '
        ],
    },
)
