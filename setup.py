from __future__ import print_function
from setuptools import setup
from pydar import __version__ as VERSION
from build_utils import BuildCommand
from build_utils import PublishCommand
from build_utils import BinaryDistribution


PACKAGE_NAME = 'pydar'
BuildCommand.pkg = PACKAGE_NAME
# BuildCommand.py2 = False  #
PublishCommand.pkg = PACKAGE_NAME
PublishCommand.version = VERSION
README = open('readme.md').read()

setup(
	name=PACKAGE_NAME,
	version=VERSION,
	author="Kevin Walchko",
	keywords=['lidar', 'robot'],
	author_email="walchko@users.noreply.github.com",
	description="Some python lidar drivers",
	long_description=README,
	long_description_content_type="text/markdown",
	license="MIT",
	classifiers=[
		'Development Status :: 4 - Beta',
		'License :: OSI Approved :: MIT License',
		'Programming Language :: Python :: 2.7',
		'Programming Language :: Python :: 3.7',
		'Operating System :: Unix',
		'Operating System :: POSIX :: Linux',
		'Operating System :: MacOS :: MacOS X',
		'Operating System :: POSIX',
		'Topic :: Scientific/Engineering',
		'Topic :: Scientific/Engineering :: Artificial Intelligence',
		'Topic :: Scientific/Engineering :: Image Recognition',
		'Topic :: Software Development :: Libraries :: Python Modules'
	],
	install_requires=[
		# 'pyyaml',
		# 'simplejson',
		'pyserial',
		'build_utils'
	],
	url="https://github.com/MomsFriendlyRobotCompany/{}".format(PACKAGE_NAME),
	packages=[PACKAGE_NAME],
	cmdclass={
		'publish': PublishCommand,
		'make': BuildCommand
	},
	# scripts=[
	# 	# 'bin/mjpeg_server.py',  # why? use opencvutils instead
	# 	# 'bin/bag_play.py',
	# 	# 'bin/bag_record.py',
	# 	# 'bin/camera_calibrate.py',
	# 	# 'bin/image_view.py',
	# 	# 'bin/service.py',  # fix
	# 	# 'bin/topic_echo.py',
	# 	# 'bin/topic_pub.py',
	# 	# 'bin/twist_keyboard.py'
	# 	# 'bin/video.py',
	# 	# 'bin/webserver.py'
	# ]
)
