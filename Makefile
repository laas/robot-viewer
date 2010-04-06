## hacky Makefile for robotpkg 
all: doc build

build:
	python setup.py build

install: doc
ifndef ROBOTPKG_BASE
	echo "environement varaible ROBOTPKG_BASE not defined"
else
	test -d ${ROBOTPKG_BASE}/share/doc/robot-viewer || mkdir ${ROBOTPKG_BASE}/share/doc/robot-viewer
	cp -r docs/build/html ${ROBOTPKG_BASE}/share/doc/robot-viewer
	python setup.py install --prefix ${ROBOTPKG_BASE}
endif

doc:
	cd docs && make html