## hacky Makefile for robotpkg 
all: build

build:
	python setup.py build

install: 
ifndef ROBOTPKG_BASE
	echo "environement varaible ROBOTPKG_BASE not defined"
else
	python setup.py install --prefix ${ROBOTPKG_BASE}
	make doc	
	test -d ${ROBOTPKG_BASE}/share/doc/robot-viewer || mkdir ${ROBOTPKG_BASE}/share/doc/robot-viewer
	cp -r docs/build/html ${ROBOTPKG_BASE}/share/doc/robot-viewer
	rm -f ${ROBOTPKG_BASE}/bin/robotviewer
	cp examples/robotviewer.py ${ROBOTPKG_BASE}/bin/robotviewer
	cp examples/centipede_player.py ${ROBOTPKG_BASE}/bin/rv-centipede-player
	mkdir -p $HOME/.robotviewer
	cp data/floor.py $HOME/.robotviewer
	cat <<EOG
[robots]
hrp $ROBOTPKG_BASE/OpenHRP/Controller/IOserver/robot/HRP2JRL/model/HRP2JRLmain.wrl

[joint_rank]
hrp $ROBOTPKG_BASE/share/hrp2_14/HRP2LinkJointRank.xml

[scripts]
floor $HOME/.robotviewer/floor.py
EOF > $HOME/.robotviewer/config
endif

doc:
	cd docs && make html