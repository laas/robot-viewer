#! /bin/sh
test -d ${ROBOTPKG_BASE}/share/doc/robot-viewer || mkdir ${ROBOTPKG_BASE}/share/doc/robot-viewer
cp -r docs/build/html ${ROBOTPKG_BASE}/share/doc/robot-viewer
rm -f ${ROBOTPKG_BASE}/bin/robotviewer
cp examples/robotviewer.py ${ROBOTPKG_BASE}/bin/robotviewer
cp examples/centipede_player.py ${ROBOTPKG_BASE}/bin/rv-centipede-player
cp examples/rv-sot-bridge.py ${ROBOTPKG_BASE}/bin/rv-sot-bridge
mkdir -p $HOME/.robotviewer
cp data/floor.py $HOME/.robotviewer
cat > $HOME/.robotviewer/config <<heredoc
[robots]
hrp $ROBOTPKG_BASE/OpenHRP/Controller/IOserver/robot/HRP2JRL/model/HRP2JRLmain.wrl

[joint_rank]
hrp $ROBOTPKG_BASE/share/hrp2_14/HRP2LinkJointRank.xml

[scripts]
floor $HOME/.robotviewer/floor.py
heredoc
