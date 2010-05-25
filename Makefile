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
	cp src/robotviewer ${ROBOTPKG_BASE}/bin
endif

doc:
	cd docs && make html