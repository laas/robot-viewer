#! /bin/sh
test -d ${LOCALBASE}/share/doc/robot-viewer || mkdir ${LOCALBASE}/share/doc/robot-viewer
cp -r docs/build/html ${LOCALBASE}/share/doc/robot-viewer
rm -f ${LOCALBASE}/bin/robotviewer
cp examples/rv-server.py ${LOCALBASE}/bin/robotviewer
cp examples/centipede_player.py ${LOCALBASE}/bin/rv-centipede-player
cp examples/rv-sot-bridge.py ${LOCALBASE}/bin/rv-sot-bridge
mkdir -p $HOME/.robotviewer
cp data/floor.py $HOME/.robotviewer
cat > $HOME/.robotviewer/config <<heredoc
[robots]
hrp=$LOCALBASE/OpenHRP/Controller/IOserver/robot/HRP2JRL/model/HRP2JRLmain.wrl

[joint_rank]
hrp=$LOCALBASE/share/hrp2_14/HRP2LinkJointRank.xml

[scripts]
floor=$HOME/.robotviewer/floor.py
heredoc
